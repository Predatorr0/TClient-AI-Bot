/* Avoiding Freeze - DDNet TAS Component Implementation
 * Uses DDNet physics simulation to predict future positions
 * and automatically avoids freeze/death/tele tiles.
 */

#include "avoid_freeze.h"

#include <engine/shared/config.h>
#include <engine/console.h>
#include <base/log.h>
#include <game/client/gameclient.h>
#include <game/client/prediction/entities/character.h>
#include <game/collision.h>
#include <game/mapitems.h>

#include <algorithm>
#include <fstream>

// Local constants for easier danger checking
enum
{
	TILE_DANGER_FLAG = 1 << 0,
};

void CAvoidFreeze::OnInit()
{
	m_DangerLevel = DANGER_SAFE;
	m_ActiveScenario = SCENARIO_CURRENT;
	m_WasModifying = false;
	m_aLastMapName[0] = 0;
	m_LastLogTime = 0;
	m_History.clear();
}

int CAvoidFreeze::GetTilesAt(vec2 Pos, float Proximity) const
{
	CCollision *pCollision = GameClient()->Collision();
	if(!pCollision) return 0;

	int Flags = 0;
	const float p = Proximity;
	vec2 aPoints[] = { Pos, vec2(Pos.x+p, Pos.y), vec2(Pos.x-p, Pos.y), vec2(Pos.x, Pos.y+p), vec2(Pos.x, Pos.y-p) };

	for(const vec2 &pt : aPoints)
	{
		int Index = pCollision->GetPureMapIndex(pt);
		int Tile = pCollision->GetTileIndex(Index);
		int Front = pCollision->GetFrontTileIndex(Index);
		
		#define IS_DANGER_TILE(t) ((t) == TILE_FREEZE || (t) == TILE_DFREEZE || (t) == TILE_LFREEZE || (t) == TILE_DEATH)

		if(IS_DANGER_TILE(Tile) || IS_DANGER_TILE(Front))
		{
			Flags |= TILE_DANGER_FLAG;
			break;
		}
		#undef IS_DANGER_TILE
	}
	return Flags;
}

bool CAvoidFreeze::IsActive() const
{
	return g_Config.m_TcAvoidFreeze > 0;
}

// Ping-adaptive lookahead for Legit mode
int CAvoidFreeze::CalcLookaheadTicks() const
{
	bool IsLegit = g_Config.m_TcAvoidFreeze == MODE_LEGIT;
	
	if(!g_Config.m_TcAvoidFreezeAuto)
		return g_Config.m_TcAvoidFreezeTicks;

	// Get current ping in ms, convert to ticks (1 tick = 20ms)
	float PingMs = GameClient()->m_Snap.m_pLocalInfo ? (float)GameClient()->m_Snap.m_pLocalInfo->m_Latency : 0.0f;
	int PingTicks = (int)(PingMs / 20.0f) + 1;

	// Velocity-based lookahead bonus
	CCharacter *pLocalChar = GameClient()->m_PredictedWorld.GetCharacterById(GameClient()->m_Snap.m_LocalClientId);
	int SpeedTicks = 0;
	if(pLocalChar)
	{
		const CCharacterCore &Core = pLocalChar->GetCore();
		float Speed = length(Core.m_Vel);
		// At high speeds, we need to look further ahead
		SpeedTicks = (int)(Speed / 2.5f);
	}

	// Base ticks: 12 for Legit (less intrusive), 25 for Rage
	int BaseTicks = IsLegit ? 12 : 25;
	
	// Final dynamic lookahead
	return std::clamp(PingTicks + BaseTicks + SpeedTicks, IsLegit ? 10 : 25, 100);
}

// ============================================================
// Tile Danger Checks
// ============================================================

bool CAvoidFreeze::IsFreezeTileAt(vec2 Pos) const
{
	CCollision *pCollision = GameClient()->Collision();
	if(!pCollision)
		return false;

	// Check at multiple points around the tee (using proximity radius)
	const float Radius = 14.0f; // Tee collision radius
	const vec2 aCheckPoints[] = {
		Pos,
		vec2(Pos.x + Radius, Pos.y + Radius),
		vec2(Pos.x + Radius, Pos.y - Radius),
		vec2(Pos.x - Radius, Pos.y + Radius),
		vec2(Pos.x - Radius, Pos.y - Radius),
	};

	for(const vec2 &CheckPos : aCheckPoints)
	{
		int Index = pCollision->GetPureMapIndex(CheckPos);
		int TileIndex = pCollision->GetTileIndex(Index);
		int FrontIndex = pCollision->GetFrontTileIndex(Index);
		int SwitchType = pCollision->GetSwitchType(Index);

		if(TileIndex == TILE_FREEZE || TileIndex == TILE_DFREEZE || TileIndex == TILE_LFREEZE)
			return true;
		if(FrontIndex == TILE_FREEZE || FrontIndex == TILE_DFREEZE || FrontIndex == TILE_LFREEZE)
			return true;
		if(SwitchType == TILE_FREEZE || SwitchType == TILE_DFREEZE || SwitchType == TILE_LFREEZE)
			return true;
	}
	return false;
}

bool CAvoidFreeze::IsDeathTileAt(vec2 Pos) const
{
	if(!g_Config.m_TcAvoidFreezeDeath)
		return false;

	CCollision *pCollision = GameClient()->Collision();
	if(!pCollision)
		return false;

	const float Radius = 14.0f;
	const vec2 aCheckPoints[] = {
		vec2(Pos.x + Radius, Pos.y + Radius),
		vec2(Pos.x + Radius, Pos.y - Radius),
		vec2(Pos.x - Radius, Pos.y + Radius),
		vec2(Pos.x - Radius, Pos.y - Radius),
	};

	for(const vec2 &CheckPos : aCheckPoints)
	{
		if(pCollision->GetCollisionAt(CheckPos.x, CheckPos.y) == TILE_DEATH)
			return true;
		if(pCollision->GetFrontCollisionAt(CheckPos.x, CheckPos.y) == TILE_DEATH)
			return true;
	}
	return false;
}

bool CAvoidFreeze::IsTeleTileAt(vec2 Pos) const
{
	if(!g_Config.m_TcAvoidFreezeTele)
		return false;

	// For now, we don't actively avoid teleports unless the user enables it
	return false;
}

bool CAvoidFreeze::IsDangerousTileAt(vec2 Pos) const
{
	return IsFreezeTileAt(Pos) || IsDeathTileAt(Pos) || IsTeleTileAt(Pos);
}

float CAvoidFreeze::DistanceToNearestFreeze(vec2 Pos) const
{
	CCollision *pCollision = GameClient()->Collision();
	if(!pCollision)
		return 999.0f;

	float MinDist = 999.0f;
	const int SearchRadius = 5; // Search 5 tiles in each direction
	const float TileSize = 32.0f;

	int TileX = (int)(Pos.x / TileSize);
	int TileY = (int)(Pos.y / TileSize);

	for(int dy = -SearchRadius; dy <= SearchRadius; dy++)
	{
		for(int dx = -SearchRadius; dx <= SearchRadius; dx++)
		{
			vec2 CheckPos = vec2((TileX + dx) * TileSize + TileSize / 2.0f,
				(TileY + dy) * TileSize + TileSize / 2.0f);

			if(IsFreezeTileAt(CheckPos) || IsDeathTileAt(CheckPos))
			{
				float Dist = distance(Pos, CheckPos);
				if(Dist < MinDist)
					MinDist = Dist;
			}
		}
	}
	return MinDist;
}

// ============================================================
// KoG Tight Passage Helpers
// ============================================================

bool CAvoidFreeze::IsInTightPassage(vec2 Pos) const
{
	const float TileSize = 32.0f;
	bool FreezeLeft = false, FreezeRight = false, FreezeUp = false, FreezeDown = false;

	for(int i = 1; i <= 2; i++)
	{
		if(IsFreezeTileAt(vec2(Pos.x - TileSize * i, Pos.y)))
			FreezeLeft = true;
		if(IsFreezeTileAt(vec2(Pos.x + TileSize * i, Pos.y)))
			FreezeRight = true;
		if(IsFreezeTileAt(vec2(Pos.x, Pos.y - TileSize * i)))
			FreezeUp = true;
		if(IsFreezeTileAt(vec2(Pos.x, Pos.y + TileSize * i)))
			FreezeDown = true;
	}

	return (FreezeLeft && FreezeRight) || (FreezeUp && FreezeDown);
}

CAvoidFreeze::CSimulationResult CAvoidFreeze::PredictStoppingPath(const CCharacterCore &Core) const
{
	CSimulationResult Result;
	CCharacterCore SimCore = Core;

	Result.m_WillFreeze = false;
	Result.m_WillDie = false;
	Result.m_NearestFreezeDist = 999.0f;

	for(int i = 0; i < 100; i++) // Increased from 50 to 100 for better high-speed prediction
	{
		CNetObj_PlayerInput Input;
		Input.m_Direction = 0;
		Input.m_Jump = 0;
		Input.m_Hook = 0;

		SimCore.m_Input = Input;
		SimCore.Tick(true);
		SimCore.Move();

		int DangerFlags = GetTilesAt(SimCore.m_Pos, (float)g_Config.m_TcAvoidFreezeRadius);

		if(DangerFlags & TILE_DANGER_FLAG)
		{
			if(IsFreezeTileAt(SimCore.m_Pos))
				Result.m_WillFreeze = true;
			if(IsDeathTileAt(SimCore.m_Pos))
				Result.m_WillDie = true;
		}

		float d = distance(Core.m_Pos, SimCore.m_Pos);
		if(DangerFlags & TILE_DANGER_FLAG)
			Result.m_NearestFreezeDist = std::min(Result.m_NearestFreezeDist, d);

		if(Result.m_WillFreeze || Result.m_WillDie)
		{
			Result.m_FinalPos = SimCore.m_Pos;
			return Result;
		}
		
		if(length(SimCore.m_Vel) < 0.1f && i > 5)
			break;
	}

	Result.m_FinalPos = SimCore.m_Pos;
	return Result;
}

bool CAvoidFreeze::CanPassSafely(const CCharacterCore &Core, const CNetObj_PlayerInput &Input, int Ticks) const
{
	CCharacterCore SimCore = Core;
	for(int t = 0; t < Ticks; t++)
	{
		CNetObj_PlayerInput TestInput = Input;
		SimCore.m_Input = TestInput;
		SimCore.Tick(true);
		SimCore.Move();

		if(IsDangerousTileAt(SimCore.m_Pos))
			return false;
	}
	return true;
}

vec2 CAvoidFreeze::FindSafeHookTarget(vec2 Pos, vec2 Vel) const
{
	CCollision *pCollision = GameClient()->Collision();
	if(!pCollision)
		return Pos;

	const float TileSize = 32.0f;
	float BestScore = -99999.0f;
	vec2 BestTarget = Pos;

	int TileX = (int)(Pos.x / TileSize);
	int TileY = (int)(Pos.y / TileSize);

	// Determine ideal hook direction (opposite to velocity)
	vec2 HookDir(0, -1); // Default to hooking up
	if(length(Vel) > 0.1f)
		HookDir = normalize(-Vel);

	for(int dy = -12; dy <= 12; dy++)
	{
		for(int dx = -12; dx <= 12; dx++)
		{
			if(dx == 0 && dy == 0)
				continue;

			vec2 CheckPos = vec2((TileX + dx) * TileSize + TileSize / 2.0f,
				(TileY + dy) * TileSize + TileSize / 2.0f);

			if(!pCollision->CheckPoint(CheckPos.x, CheckPos.y))
				continue;

			if(IsFreezeTileAt(CheckPos) || IsDeathTileAt(CheckPos))
				continue;

			float Dist = distance(Pos, CheckPos);
			if(Dist > 380.0f) // Max hook length
				continue;

			vec2 DirToTarget = normalize(CheckPos - Pos);
			
			// We want a hook that is opposite to our movement to slow us down (momentum cancellation)
			float DirectionMatch = dot(DirToTarget, HookDir);
			
			// Score logic: 
			// 1. Extreme priority: Don't hook if it pulls us into freeze
			// 2. High priority: Pull against velocity to brake
			// 3. Medium priority: Prefer hooking UP to stay airborne longer
			float Score = (DirectionMatch * 2000.0f) - (Dist * 0.5f) + (DirToTarget.y < -0.5f ? 500.0f : 0.0f);

			if(Score > BestScore)
			{
				BestScore = Score;
				BestTarget = CheckPos;
			}
		}
	}

	return BestTarget;
}

// ============================================================
// Scenario Simulation
// ============================================================

void CAvoidFreeze::SimulateScenario(const CCharacterCore &CurrentCore, const CNetObj_PlayerInput &BaseInput,
	EScenario Scenario, int NumTicks, CSimulationResult &Result) const
{
	Result.m_WillFreeze = false;
	Result.m_WillDie = false;
	Result.m_WillTele = false;
	Result.m_WillLaser = false;
	Result.m_FirstDangerTick = -1;
	Result.m_NearestFreezeDist = 999.0f;
	Result.m_ModifiedInput = BaseInput;

	CNetObj_PlayerInput SimInput = BaseInput;
	switch(Scenario)
	{
	case SCENARIO_CURRENT:
		break;
	case SCENARIO_STOP:
		SimInput.m_Direction = 0;
		break;
	case SCENARIO_REVERSE:
		SimInput.m_Direction = -BaseInput.m_Direction;
		if(SimInput.m_Direction == 0)
			SimInput.m_Direction = -1;
		break;
	case SCENARIO_JUMP:
		SimInput.m_Jump = 1;
		break;
	case SCENARIO_HOOK:
	{
		vec2 HookTarget = FindSafeHookTarget(CurrentCore.m_Pos, CurrentCore.m_Vel);
		SimInput.m_Hook = 1;
		SimInput.m_TargetX = (int)(HookTarget.x - CurrentCore.m_Pos.x);
		SimInput.m_TargetY = (int)(HookTarget.y - CurrentCore.m_Pos.y);
		if(SimInput.m_TargetX == 0 && SimInput.m_TargetY == 0)
			SimInput.m_TargetY = -1;
		break;
	}
	case SCENARIO_STOP_HOOK:
		SimInput.m_Hook = 0;
		break;
	default:
		break;
	}

	Result.m_ModifiedInput = SimInput;
	CCharacterCore SimCore = CurrentCore;
	
	// Ensure collision and tuning are valid before simulating ticks
	if(!SimCore.Collision())
		SimCore.SetCoreWorld(nullptr, GameClient()->Collision(), nullptr);
	
	if(!SimCore.Collision()) return;

	for(int t = 0; t < NumTicks; t++)
	{
		// To allow double jumping, release jump on tick 1, and press it again on tick 2 if it's the JUMP scenario
		if(Scenario == SCENARIO_JUMP)
		{
			if(t == 1) SimInput.m_Jump = 0;
			if(t == 2) SimInput.m_Jump = 1;
		}

		SimCore.m_Input = SimInput;
		SimCore.Tick(true);
		SimCore.Move();

		float FreezeDist = DistanceToNearestFreeze(SimCore.m_Pos);
		// Subtract radius from distance to get "surface" distance
		FreezeDist = std::max(0.0f, FreezeDist - (float)g_Config.m_TcAvoidFreezeRadius);
		if(FreezeDist < Result.m_NearestFreezeDist)
			Result.m_NearestFreezeDist = FreezeDist;

		if(IsFreezeTileAt(SimCore.m_Pos))
		{
			Result.m_WillFreeze = true;
			if(Result.m_FirstDangerTick == -1)
				Result.m_FirstDangerTick = t;
		}

		if(IsDeathTileAt(SimCore.m_Pos))
		{
			Result.m_WillDie = true;
			if(Result.m_FirstDangerTick == -1)
				Result.m_FirstDangerTick = t;
		}

		if(IsTeleTileAt(SimCore.m_Pos))
		{
			Result.m_WillTele = true;
			if(Result.m_FirstDangerTick == -1)
				Result.m_FirstDangerTick = t;
		}
	}
	Result.m_FinalPos = SimCore.m_Pos;
}

bool CAvoidFreeze::TestInputSafety(const CCharacterCore &Core, const CNetObj_PlayerInput &Input, int Ticks) const
{
	CSimulationResult Result;
	SimulateScenario(Core, Input, SCENARIO_CURRENT, Ticks, Result);
	return !Result.m_WillFreeze && !Result.m_WillDie;
}

CAvoidFreeze::EScenario CAvoidFreeze::ChooseBestScenario(const CCharacterCore &Core, const CSimulationResult *pResults) const
{
	bool IsRage = g_Config.m_TcAvoidFreeze == MODE_RAGE;
	float SafetyMargin = (float)g_Config.m_TcAvoidFreezeMargin;

	EScenario BestScenario = SCENARIO_CURRENT;
	float BestScore = -99999.0f;

	for(int i = 0; i < NUM_SCENARIOS; i++)
	{
		const CSimulationResult &R = pResults[i];
		bool IsSafe = !R.m_WillFreeze && !R.m_WillDie && !R.m_WillTele;
		bool HasMargin = R.m_NearestFreezeDist > SafetyMargin;

		float Score = 0.0f;

		// 1. HYSTERESIS: Prevent jitter by strongly preferring the last chosen strategy
		if(i == m_ActiveScenario) 
			Score += 500.0f; // Reduced from 1500 to 500 for better responsiveness

		// 1b. Direction Stability: Penalize toggling directions between scenarios
		if(i != SCENARIO_CURRENT && R.m_ModifiedInput.m_Direction != 0 && R.m_ModifiedInput.m_Direction == -Core.m_Input.m_Direction)
			Score -= 500.0f;

		// 2. MOMENTUM / ESCAPE REWARD: Favor scenarios that physically move us away from danger
		float DistMoved = distance(Core.m_Pos, R.m_FinalPos);
		Score += DistMoved * 0.5f;

		// Primary: reward safety
		if(IsSafe && HasMargin) Score += 5000.0f;
		else if(IsSafe) Score += 1000.0f;

		// Rage mode: absolute intolerance for hazards
		if(IsRage && (R.m_WillFreeze || R.m_WillDie || R.m_WillTele))
			Score -= 100000.0f;

		// Margin-based scoring: reward keeping distance from freeze
		Score += R.m_NearestFreezeDist * (IsRage ? 10.0f : 2.0f);

		// Penalize dangerously low margins
		if(R.m_NearestFreezeDist < 10.0f) Score -= 2000.0f;
		if(R.m_NearestFreezeDist < 5.0f) Score -= 5000.0f;

		// Scenario preference
		float SafetyThresholdActual = IsRage ? 128.0f : (SafetyMargin + 32.0f);
		bool NearDanger = (R.m_NearestFreezeDist < SafetyThresholdActual);

		if(NearDanger)
		{
			if(IsRage)
			{
				// Rage: Safe movement regardless of user intent
				if(i == SCENARIO_STOP) Score += 500.0f;
				else if(i == SCENARIO_REVERSE) Score += 400.0f;
				else if(i == SCENARIO_JUMP) Score += 300.0f;
				else if(i == SCENARIO_HOOK) Score += 200.0f;
				else if(i == SCENARIO_CURRENT) Score -= 100.0f; // Punish potentially dangerous current input
			}
			else
			{
				// Legit: Prioritize user intent until absolutely necessary
				if(i == SCENARIO_CURRENT) Score += 400.0f; 
				else if(i == SCENARIO_STOP_HOOK) Score += 150.0f;
				else if(i == SCENARIO_STOP) Score += 100.0f;
			}
		}
		else
		{
			// When safe: prefer keeping current direction (both modes)
			if(i == SCENARIO_CURRENT) Score += 1000.0f;
		}
		
		// 3. ALWAYS reward surviving longer heavily. 
		// HOOK TOLERANCE: If we are hooking but danger is 10+ ticks away, don't penalize as heavily so we can SWING.
		if(R.m_WillFreeze || R.m_WillDie || R.m_WillTele) {
			float Penalty = 10000.0f;
			if((i == SCENARIO_HOOK || i == SCENARIO_CURRENT) && R.m_FirstDangerTick > 10 && !IsRage)
				Penalty = 2000.0f; // Soften penalty for swings
				
			Score -= Penalty;
			if(R.m_FirstDangerTick != -1)
				Score += R.m_FirstDangerTick * 1000.0f;
		}

		if(Score > BestScore)
		{
			BestScore = Score;
			BestScenario = (EScenario)i;
		}
	}
	return BestScenario;
}

// ============================================================
// Main Logic — Called before input is sent
// ============================================================

void CAvoidFreeze::OnBeforeInput(CNetObj_PlayerInput *pInput)
{
	if(!IsActive())
	{
		m_DangerLevel = DANGER_SAFE;
		m_ActiveScenario = SCENARIO_CURRENT;
		m_WasModifying = false;
		return;
	}

	static int64_t s_LastLogTime = 0;
	static int s_TickCounter = 0;
	s_TickCounter++;
	bool ShouldLog = time_get() > s_LastLogTime + time_freq() / 2;

	CCharacter *pLocalChar = GameClient()->m_PredictedWorld.GetCharacterById(GameClient()->m_Snap.m_LocalClientId);
	if(!pLocalChar)
	{
		m_DangerLevel = DANGER_SAFE;
		return;
	}

	const CCharacterCore &Core = pLocalChar->GetCore();

	// Death tracking mechanism
	static vec2 s_LastPos = vec2(0, 0);
	static vec2 s_LastVel = vec2(0, 0);
	static EScenario s_LastScenario = SCENARIO_CURRENT;
	static int s_TicksInDanger = 0;
	static bool s_WasFrozen = false;
	
	if(m_DangerLevel == DANGER_CRITICAL)
		s_TicksInDanger++;
	else if(m_DangerLevel == DANGER_SAFE)
		s_TicksInDanger = 0; 

	bool IsFrozen = (Core.m_FreezeEnd > 0 || Core.m_DeepFrozen || Core.m_LiveFrozen);
	
	// MAP TRACKING
	const char *pMapName = GameClient()->Map()->BaseName();
	if(str_comp(m_aLastMapName, pMapName) != 0)
	{
		str_copy(m_aLastMapName, pMapName, sizeof(m_aLastMapName));
		log_info("avoid_freeze", "Map changed to: %s", pMapName);
	}

	// DEATH / FREEZE ANALYSIS
	if (IsFrozen && !s_WasFrozen && s_TicksInDanger > 0)
	{
		std::ofstream logfile("C:\\Users\\ataka\\ddnet_ai_logs.txt", std::ios_base::app);
		if (logfile.is_open())
		{
			logfile << "\n!!! FAILURE ANALYSIS !!!\n";
			logfile << "Map: " << pMapName << " | Reason: " << (Core.m_DeepFrozen ? "DEEP_FREEZE" : "FREEZE") << "\n";
			logfile << "Pos: " << Core.m_Pos.x/32.0f << "," << Core.m_Pos.y/32.0f << " | Vel: " << Core.m_Vel.x << "," << Core.m_Vel.y << "\n";
			logfile << "Recent History (Last 20 ticks):\n";
			for (const auto &h : m_History)
			{
				const char *pDS = h.m_Danger == DANGER_SAFE ? "SAFE" : (h.m_Danger == DANGER_WARNING ? "WARN" : "CRIT");
				const char *pSC = h.m_Scenario == SCENARIO_CURRENT ? "CUR" : "MOD";
				logfile << "  T=" << h.m_Tick << " P=" << h.m_Pos.x/32.0f << "," << h.m_Pos.y/32.0f 
					<< " V=" << h.m_Vel.x << "," << h.m_Vel.y 
					<< " Inp=" << h.m_Input.m_Direction << "," << h.m_Input.m_Jump << "," << h.m_Input.m_Hook
					<< " D=" << pDS << " S=" << pSC << "\n";
			}
			logfile << "!!! END ANALYSIS !!!\n\n";
		}
	}
	s_WasFrozen = IsFrozen;

	// Update History
	CHistoryEntry Entry;
	Entry.m_Tick = Client()->GameTick(0);
	Entry.m_Pos = Core.m_Pos;
	Entry.m_Vel = Core.m_Vel;
	Entry.m_Input = *pInput;
	Entry.m_Scenario = m_ActiveScenario;
	Entry.m_Danger = m_DangerLevel;
	m_History.push_back(Entry);
	if (m_History.size() > 20) m_History.erase(m_History.begin());
	s_LastPos = Core.m_Pos;
	s_LastVel = Core.m_Vel;
	s_LastScenario = m_ActiveScenario;

	if(IsFrozen)
	{
		m_DangerLevel = DANGER_SAFE;
		return;
	}

	// Stuck Detection & Navigation
	if (g_Config.m_TcBotEnabled)
	{
		float DistMoved = distance(m_LastPos, Core.m_Pos);
		bool IsTryingToMove = pInput->m_Direction != 0;
		
		if (IsTryingToMove && DistMoved < 0.5f)
		{
			m_StuckTicks++;
		}
		else
		{
			m_StuckTicks = 0;
		}

		// --- NEW: Hook Navigation ---
		// If there is a wall in front or a gap, try to hook
		if (g_Config.m_TcBotNavigation)
		{
			vec2 LookDir = vec2(pInput->m_Direction, -0.5f); // Look slightly up and forward
			vec2 HookPos = Core.m_Pos + LookDir * 300.0f;
			
			// Simple check for hookable tiles in front
			int HookTile = Collision()->GetCollisionAt(HookPos.x, HookPos.y);
			if (HookTile == TILE_SOLID && m_StuckTicks > 10) 
			{
				pInput->m_Hook = 1;
			}
			else
			{
				pInput->m_Hook = 0;
			}
		}

		m_LastPos = Core.m_Pos;

		if (m_StuckTicks > g_Config.m_TcBotStuckTicks)
		{
			if (g_Config.m_TcBotLogging)
				log_info("bot", "Stuck detected! AGGRESSIVE recovery. Ticks: %d, Pos: %.1f,%.1f", m_StuckTicks, Core.m_Pos.x, Core.m_Pos.y);
			
			// Aggressive recovery: Randomized blast
			int Rnd = rand() % 4;
			if(Rnd == 0) { pInput->m_Jump = 1; pInput->m_Direction = -pInput->m_Direction; }
			else if(Rnd == 1) { pInput->m_Hook = 1; pInput->m_TargetY = -100; pInput->m_TargetX = (rand()%200)-100; }
			else if(Rnd == 2) { pInput->m_Direction = (rand()%2)*2-1; pInput->m_Jump = (rand()%2); }
			else { pInput->m_Hook = 0; pInput->m_Jump = 1; }
			
			m_StuckTicks = 0;
			return;
		}
	}

	int LookaheadTicks = CalcLookaheadTicks();
	CSimulationResult StopResult = PredictStoppingPath(Core);

	CSimulationResult aResults[NUM_SCENARIOS];
	for(int i = 0; i < NUM_SCENARIOS; i++)
	{
		if(i == SCENARIO_HOOK && !g_Config.m_TcAvoidFreezeSmartHook)
		{
			aResults[i].m_WillFreeze = true;
			aResults[i].m_NearestFreezeDist = 0;
			continue;
		}
		if(i == SCENARIO_JUMP && !g_Config.m_TcAvoidFreezeSmartJump)
		{
			aResults[i].m_WillFreeze = true;
			aResults[i].m_NearestFreezeDist = 0;
			continue;
		}
		SimulateScenario(Core, *pInput, (EScenario)i, LookaheadTicks, aResults[i]);
	}

	const CSimulationResult &CurrentResult = aResults[SCENARIO_CURRENT];
	float FreezeDist = CurrentResult.m_NearestFreezeDist;
	float SafetyThreshold = (float)g_Config.m_TcAvoidFreezeMargin;

	if(g_Config.m_TcAvoidFreeze == MODE_LEGIT)
	{
		float VelLen = length(Core.m_Vel);
		
		// Ping-adaptive Freedom Zone:
		// Base freedom is 4.5 tiles. We add 1 tile per 60ms of ping
		float PingMs = GameClient()->m_Snap.m_pLocalInfo ? (float)GameClient()->m_Snap.m_pLocalInfo->m_Latency : 0.0f;
		float FreedomThreshold = (4.5f + (PingMs / 60.0f)) * 32.0f; 

		if(CurrentResult.m_WillFreeze || CurrentResult.m_WillDie || StopResult.m_WillFreeze)
			m_DangerLevel = DANGER_CRITICAL;
		else if(FreezeDist < FreedomThreshold)
		{
			m_DangerLevel = DANGER_WARNING;
			UpdateLearning(false, true); 
		}
		else
		{
			// FREEDOM ZONE: Danger is too far away, let the user play!
			m_DangerLevel = DANGER_SAFE;
			m_ActiveScenario = SCENARIO_CURRENT;
			m_WasModifying = false;
			return; 
		}
	}
	else if(g_Config.m_TcAvoidFreeze == MODE_RAGE)
	{
		// Rage mode: even a slight hint of future danger counts as critical
		// Increase lookahead for Rage if needed
		bool Trajectory危险 = CurrentResult.m_WillFreeze || CurrentResult.m_WillDie || CurrentResult.m_WillTele;
		bool Stop危险 = StopResult.m_WillFreeze || StopResult.m_WillDie;

		if(Trajectory危险 || Stop危险 || FreezeDist < 64.0f)
		{
			m_DangerLevel = DANGER_CRITICAL;
			UpdateLearning(false, true); // Log predicted freeze
		}
		else
			m_DangerLevel = DANGER_SAFE;
	}

	if(ShouldLog)
	{
		log_info("avoid_freeze", "Pos: %.2f,%.2f Vel: %.2f,%.2f | Margin: %.2f | Ticks: %d | Level: %d",
			Core.m_Pos.x, Core.m_Pos.y, Core.m_Vel.x, Core.m_Vel.y, FreezeDist, LookaheadTicks, m_DangerLevel);
		s_LastLogTime = time_get();
	}

	EScenario BestScenario = ChooseBestScenario(Core, aResults);
	
	// TIGHT PASSAGE OVERRIDE: 
	// Only override if we are NOT in critical danger and the current path is safe enough.
	if(IsInTightPassage(Core.m_Pos) && BestScenario != SCENARIO_CURRENT)
	{
		if(m_DangerLevel != DANGER_CRITICAL && !CurrentResult.m_WillFreeze && !CurrentResult.m_WillDie)
		{
			BestScenario = SCENARIO_CURRENT;
		}
	}

	if(BestScenario != SCENARIO_CURRENT)
	{
		if(g_Config.m_TcAvoidFreeze == MODE_LEGIT)
		{
			if(!m_WasModifying) m_LegitDelayTicks = 0;
			m_LegitDelayTicks++;

			if(m_LegitDelayTicks >= 2)
			{
				const CSimulationResult &BestResult = aResults[BestScenario];
				pInput->m_Direction = BestResult.m_ModifiedInput.m_Direction;

				if(BestScenario == SCENARIO_JUMP && g_Config.m_TcAvoidFreezeSmartJump)
				{
					// Precise Jump: Only jump if falling and danger is very close (e.g. within 3 ticks)
					if(Core.m_Vel.y > 0.1f && BestResult.m_FirstDangerTick <= 3 && BestResult.m_FirstDangerTick != -1)
						pInput->m_Jump = BestResult.m_ModifiedInput.m_Jump;
				}
				if(BestScenario == SCENARIO_HOOK && g_Config.m_TcAvoidFreezeSmartHook)
				{
					pInput->m_Hook = BestResult.m_ModifiedInput.m_Hook;
					pInput->m_TargetX = BestResult.m_ModifiedInput.m_TargetX;
					pInput->m_TargetY = BestResult.m_ModifiedInput.m_TargetY;
				}
				if(BestScenario == SCENARIO_STOP_HOOK)
				{
					pInput->m_Hook = 0;
				}
			}
		}
		else
		{
			const CSimulationResult &BestResult = aResults[BestScenario];
			pInput->m_Direction = BestResult.m_ModifiedInput.m_Direction;

			if(BestScenario == SCENARIO_JUMP) pInput->m_Jump = BestResult.m_ModifiedInput.m_Jump;
			if(BestScenario == SCENARIO_HOOK)
			{
				pInput->m_Hook = BestResult.m_ModifiedInput.m_Hook;
				pInput->m_TargetX = BestResult.m_ModifiedInput.m_TargetX;
				pInput->m_TargetY = BestResult.m_ModifiedInput.m_TargetY;
			}

			if(pInput->m_Jump)
			{
				CNetObj_PlayerInput TestInput = *pInput;
				CSimulationResult JumpCheck;
				SimulateScenario(Core, TestInput, SCENARIO_CURRENT, LookaheadTicks, JumpCheck);
				if(JumpCheck.m_WillFreeze || JumpCheck.m_WillDie) pInput->m_Jump = 0;
			}
		}
		m_WasModifying = true;
		m_ActiveScenario = BestScenario;
	}
	else
	{
		m_WasModifying = false;
		m_LegitDelayTicks = 0;
		m_ActiveScenario = SCENARIO_CURRENT;

		if(g_Config.m_TcAvoidFreeze == MODE_RAGE && pInput->m_Jump)
		{
			CSimulationResult JumpCheck;
			SimulateScenario(Core, *pInput, SCENARIO_CURRENT, LookaheadTicks, JumpCheck);
			if(JumpCheck.m_WillFreeze || JumpCheck.m_WillDie) pInput->m_Jump = 0;
		}
	}

	// === COMPREHENSIVE FILE LOGGING ===
	{
		static int s_LastLoggedScenario = -1;
		static int s_LastDanger = -1;
		static bool s_LastFrozen = false;
		static int s_LogTickCounter = 0;
		s_LogTickCounter++;

		bool StateChanged = (m_ActiveScenario != s_LastLoggedScenario) || 
			((int)m_DangerLevel != s_LastDanger) ||
			(IsFrozen != s_LastFrozen);
		bool PeriodicLog = (s_LogTickCounter % 25 == 0);

		if (StateChanged || PeriodicLog)
		{
			std::ofstream logfile("C:\\Users\\ataka\\ddnet_ai_logs.txt", std::ios_base::app);
			if (logfile.is_open())
			{
				// Header on first write
				static bool s_HeaderWritten = false;
				if (!s_HeaderWritten)
				{
					logfile << "=== SESSION START ===\n";
					logfile << "MAP: " << GameClient()->Map()->BaseName() << "\n";
					logfile << "CONFIG: AvoidFreeze=" << g_Config.m_TcAvoidFreeze 
						<< " Radius=" << g_Config.m_TcAvoidFreezeRadius
						<< " Margin=" << g_Config.m_TcAvoidFreezeMargin
						<< " Ticks=" << g_Config.m_TcAvoidFreezeTicks
						<< " Auto=" << g_Config.m_TcAvoidFreezeAuto
						<< " SmartHook=" << g_Config.m_TcAvoidFreezeSmartHook
						<< " SmartJump=" << g_Config.m_TcAvoidFreezeSmartJump
						<< " BotEnabled=" << g_Config.m_TcBotEnabled
						<< " BotNav=" << g_Config.m_TcBotNavigation
						<< " StuckTicks=" << g_Config.m_TcBotStuckTicks
						<< "\n";
					s_HeaderWritten = true;
				}

				const char *pDangerStr = m_DangerLevel == DANGER_SAFE ? "SAFE" : (m_DangerLevel == DANGER_WARNING ? "WARN" : "CRIT");
				const char *pScenarioStr[] = {"CURRENT", "STOP", "REVERSE", "JUMP", "HOOK", "STOP_HOOK"};

				logfile << "T=" << Client()->GameTick(0)
					<< " Pos=" << Core.m_Pos.x/32.0f << "," << Core.m_Pos.y/32.0f
					<< " Vel=" << Core.m_Vel.x << "," << Core.m_Vel.y
					<< " Dir=" << pInput->m_Direction
					<< " Jump=" << pInput->m_Jump
					<< " Hook=" << pInput->m_Hook
					<< " | Danger=" << pDangerStr
					<< " Scenario=" << pScenarioStr[m_ActiveScenario]
					<< " FreezeDist=" << CurrentResult.m_NearestFreezeDist
					<< " Modified=" << (m_WasModifying ? "YES" : "NO")
					<< " Frozen=" << (IsFrozen ? "YES" : "NO")
					<< " Stuck=" << m_StuckTicks;

				// Log config changes
				static int s_LastMode = -1;
				if (g_Config.m_TcAvoidFreeze != s_LastMode)
				{
					logfile << " | CONFIG_CHANGE: Mode=" << g_Config.m_TcAvoidFreeze;
					s_LastMode = g_Config.m_TcAvoidFreeze;
				}

				if (IsFrozen && !s_LastFrozen)
					logfile << " | *** GOT FROZEN ***";

				logfile << "\n";
			}

			s_LastLoggedScenario = m_ActiveScenario;
			s_LastDanger = (int)m_DangerLevel;
			s_LastFrozen = IsFrozen;
		}
	}
}

void CAvoidFreeze::OnRender()
{
	if(!IsActive()) return;

	ColorRGBA IndicatorColor;
	switch(m_DangerLevel)
	{
	case DANGER_SAFE:     IndicatorColor = ColorRGBA(0.0f, 1.0f, 0.0f, 0.7f); break;
	case DANGER_WARNING:  IndicatorColor = ColorRGBA(1.0f, 0.65f, 0.0f, 0.85f); break;
	case DANGER_CRITICAL: IndicatorColor = ColorRGBA(1.0f, 0.0f, 0.0f, 1.0f); break;
	}

	CUIRect Screen;
	Graphics()->GetScreen(&Screen.x, &Screen.y, &Screen.w, &Screen.h);
	Screen.w -= Screen.x;
	Screen.h -= Screen.y;

	float IndicatorX = Screen.x + Screen.w - 30.0f;
	float IndicatorY = Screen.y + 60.0f;
	float IndicatorRadius = 8.0f;

	Graphics()->TextureClear();
	Graphics()->QuadsBegin();
	Graphics()->SetColor(IndicatorColor);

	IGraphics::CQuadItem QuadItem(IndicatorX - IndicatorRadius, IndicatorY - IndicatorRadius,
		IndicatorRadius * 2.0f, IndicatorRadius * 2.0f);
	Graphics()->QuadsDrawTL(&QuadItem, 1);
	Graphics()->QuadsEnd();

	const char *pModeText = g_Config.m_TcAvoidFreeze == MODE_LEGIT ? "AF:L" : "AF:R";
	TextRender()->Text(IndicatorX - 30.0f, IndicatorY - 6.0f, 10.0f, pModeText);
}

void CAvoidFreeze::UpdateLearning(bool IsDeath, bool IsFreeze)
{
	if (!g_Config.m_TcBotEnabled || !g_Config.m_TcBotLogging)
		return;

	// Anti-spam: only log once per unique position (quantized to tile)
	static int s_LastTileX = -1;
	static int s_LastTileY = -1;
	static int s_LastEventTick = 0;

	const CCharacterCore &Core = GameClient()->m_PredictedChar;
	int TileX = (int)(Core.m_Pos.x / 32.0f);
	int TileY = (int)(Core.m_Pos.y / 32.0f);
	int CurrentTick = Client()->GameTick(0);

	// Only log if position changed by at least 1 tile or 50+ ticks passed
	if (TileX == s_LastTileX && TileY == s_LastTileY && (CurrentTick - s_LastEventTick) < 50)
		return;

	s_LastTileX = TileX;
	s_LastTileY = TileY;
	s_LastEventTick = CurrentTick;

	const char *pType = IsDeath ? "DEATH" : (IsFreeze ? "FREEZE_PREDICT" : "EVENT");
	log_info("bot_learning", "[%s] Tile(%d,%d) Vel: %.1f,%.1f Tick: %d",
		pType, TileX, TileY, Core.m_Vel.x, Core.m_Vel.y, CurrentTick);
}
