#include <engine/shared/config.h>
#include <engine/console.h>
#include <game/client/gameclient.h>
#include <game/client/components/controls.h>
#include <game/client/components/tclient/tas_bot.h>

#include <game/client/components/tclient/tas_bot.h>

#include <windows.h>
#ifdef IMAGE_CURSOR
#undef IMAGE_CURSOR
#endif
// [GHOST_V8_PULSAR_FINAL_VERIFICATION_COMPLETE]
#include <game/client/components/tclient/tas_bot.h>
#include <game/client/components/tclient/astar_pathfinder.h>
#include <game/client/prediction/entities/character.h>
#include <game/mapitems.h>
#include <game/client/animstate.h>
#include <generated/client_data.h>
#include <algorithm>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <iostream>
#include "synapse_weights.h"

CTasBot::CTasBot()
{
	m_LastUpdateTick = -1;
	m_LastAITick = -1;
	m_LastRecalculateTick = -1;
	m_PreviousHorizontalVel = 0.0f;
	m_IsTraining = false;
	m_HasMasterRun = false;
	m_pPathfinder = nullptr;
	m_InceptionActive = false;
	m_SimulatedCore.Reset();
	m_Epsilon = 1.0f; // Start with full exploration
	m_ConsecutiveDeadEnds = 0;
	m_StrategyTicks = 0;
	m_ActionHoldFrames = 0;
	m_CurrentAction = 0;
	m_LastStateHash = 0;
	m_PlaybackTick = 0;
	m_StuckTicks = 0;
	m_LastStuckPos = vec2(0, 0);
	m_LastDistToGoal = 1000000.0f;
	m_LiveLastDist = 0;
	m_DreamLastDist = 0;
	// 5 minutes max run: 50 ticks * 60 seconds * 5
	m_MaxSimulationDepth = 50 * 60 * 5; 
	m_Attempts = 0;
	m_TotalTicksSimulated = 0;
	m_ForceJumpNext = false;
	m_ForceHookNext = false;
	m_PrevAvoidFreeze = -1;
	m_pTasWorld = nullptr;
	m_PosHistoryIdx = 0;
	m_BannedAction = -1;
	m_BannedActionTicks = 0;
	m_ActionHoldElapsed = 0;
	m_LiveLastDist = 0.0f;
	m_DreamLastDist = 0.0f;
	m_LastUpdatePos = vec2(0,0);
	m_StuckCounter = 0;
	for(int i = 0; i < 5; i++) m_PosHistory[i] = vec2(0,0);
	
	m_pDiffusionManifold = nullptr;
	m_LPSDActive = false;

	// Phase 62: Initialize LPSD Shared Memory Bridge (The Oracle Bridge)
	HANDLE hMapFile = CreateFileMappingA(
		INVALID_HANDLE_VALUE,
		NULL,
		PAGE_READWRITE,
		0,
		sizeof(DiffusionBrain),
		"Global\\TrinityBrain");

	if (hMapFile != NULL)
	{
		m_pDiffusionManifold = (DiffusionBrain*)MapViewOfFile(
			hMapFile,
			FILE_MAP_ALL_ACCESS,
			0,
			0,
			sizeof(DiffusionBrain));
		
		if (m_pDiffusionManifold) {
			m_LPSDActive = true;
			dbg_msg("tas_bot", "LPSD SINGULARITY BRIDGE ACTIVE: Global\\TrinityBrain");
			m_pDiffusionManifold->current_tick_index = 0;
		}
	}
	
	// Phase 16: Speedrunner Evolution
	m_BestFinishTicks = 1000000;
	m_TotalFinishes = 0;
	m_EpisodeHistory.clear();
	m_TicksSinceSpawn = 0;
	m_RecentTiles.clear();
}

CTasBot::~CTasBot()
{
	if(m_pTasWorld)
	{
		delete m_pTasWorld;
		m_pTasWorld = nullptr;
	}
	if(m_pPathfinder)
	{
		delete m_pPathfinder;
		m_pPathfinder = nullptr;
	}
}

static void ConStartTraining(IConsole::IResult *pResult, void *pUserData)
{
	((CTasBot *)pUserData)->StartTraining();
}

static void ConStopTraining(IConsole::IResult *pResult, void *pUserData)
{
	((CTasBot *)pUserData)->StopTraining();
}

static void ConPlayTas(IConsole::IResult *pResult, void *pUserData)
{
	((CTasBot *)pUserData)->StartPlayback();
}

static void ConSaveTas(IConsole::IResult *pResult, void *pUserData)
{
	((CTasBot *)pUserData)->SaveTasRun(pResult->GetString(0));
}

static void ConLoadTas(IConsole::IResult *pResult, void *pUserData)
{
	((CTasBot *)pUserData)->LoadTasRun(pResult->GetString(0));
}

void CTasBot::OnInit()
{
	m_pPathfinder = new CAStarPathfinder(GameClient()->Collision());
	m_SimulatedCore.Init(&m_SimulatedWorld, GameClient()->Collision(), &GameClient()->m_Teams);
	m_SimulatedWorld.m_apCharacters[0] = &m_SimulatedCore;
	
	Console()->Register("tas_train_start", "", CFGFLAG_CLIENT, ConStartTraining, this, "Start TAS AI Training");
	Console()->Register("tas_train_stop", "", CFGFLAG_CLIENT, ConStopTraining, this, "Stop TAS AI Training");
	Console()->Register("tas_play", "", CFGFLAG_CLIENT, ConPlayTas, this, "Start TAS AI Playback");
	Console()->Register("tas_save", "s", CFGFLAG_CLIENT, ConSaveTas, this, "Save TAS run to file");
	Console()->Register("tas_load", "s", CFGFLAG_CLIENT, ConLoadTas, this, "Load TAS run from file");
}

void CTasBot::OnMapLoad()
{
	m_IsTraining = false;
	m_HasMasterRun = false;
	m_MasterRun.clear();
	m_StateHistory.clear();
	m_DeathMemory.clear();
	m_CurrentEpisodeTicks = 0;
	m_TicksSinceSpawn = 0;
	m_RecentTiles.clear();
	m_AStarPath.clear();
	m_LastUpdateTick = -1;
	m_LastAITick = -1;
	m_LastRecalculateTick = -1;
	m_InceptionActive = false;
	
	m_QTable.clear();
	LoadMemory();

	// Phase 20: Delayed Init. Don't do heavy work until first active tick.
	m_MapWidth = 0;
	m_MapHeight = 0;
	
	// Reset world on map change
	if(m_pTasWorld)
	{
		delete m_pTasWorld;
		m_pTasWorld = nullptr;
	}

	if(m_pPathfinder)
	{
		delete m_pPathfinder;
		m_pPathfinder = nullptr;
	}

	m_MasterRunPositions.clear();
	m_CurrentPath.clear();
	
	if(GameClient()->Map())
		LoadTasRun(GameClient()->Map()->BaseName());
		
	// Phase 22: Load Trauma Map from previous sessions
	LoadTraumaMap();
}

// Phase 10: Global Flow-Field (Breadth-First Search with Gravity Bias)
void CTasBot::ComputeDistanceMap()
{
	CCollision *pCollision = GameClient()->Collision();
	if(!pCollision) return;

	m_MapWidth = pCollision->GetWidth();
	m_MapHeight = pCollision->GetHeight();
	dbg_msg("tas_bot", "ComputeDistanceMap: Dimensions %dx%d", m_MapWidth, m_MapHeight);
	m_DistanceMap.assign(m_MapWidth * m_MapHeight, 1000000); // Initialize with "Infinity"

	std::queue<int> Queue;

	// 1. Find all Finish Tiles and set them as sinks (distance 0)
	for(int y = 0; y < m_MapHeight; y++)
	{
		for(int x = 0; x < m_MapWidth; x++)
		{
			int Index = y * m_MapWidth + x;
			int Tile = pCollision->GetTileIndex(Index);
			int Front = pCollision->GetFrontTileIndex(Index);
			
			// Hyper-Aggressive Seeding: Finish, Checkpoints, and Time Checkpoints
			bool IsGoal = (Tile == TILE_FINISH || Front == TILE_FINISH || 
			               Tile == TILE_CP || Front == TILE_CP || 
			               (Tile >= 27 && Tile <= 29) || (Front >= 27 && Front <= 29)); // 27-29 are often CP/Finish in Oldschool
			
			if(IsGoal)
			{
				m_DistanceMap[Index] = 0;
				Queue.push(Index);
			}
		}
	}

	if(Queue.empty())
	{
		dbg_msg("tas_bot", "WARNING: No Finish/Checkpoints found. Fallback: Seeding bottom-most walkable layer.");
		for(int x = 0; x < m_MapWidth; x++)
		{
			int y = m_MapHeight - 2;
			int Index = y * m_MapWidth + x;
			if(!pCollision->IsSolid(x * 32 + 16, y * 32 + 16))
			{
				m_DistanceMap[Index] = 0;
				Queue.push(Index);
			}
		}
	}

	// 2. BFS Flood Fill
	while(!Queue.empty())
	{
		int CurrIdx = Queue.front();
		Queue.pop();

		int cx = CurrIdx % m_MapWidth;
		int cy = CurrIdx / m_MapWidth;
		int CurrDist = m_DistanceMap[CurrIdx];

		// Check 4-Neighbors
		int dx[] = { 1, -1,  0,  0 };
		int dy[] = { 0,  0,  1, -1 };
		int costs[] = { 10, 10, 30, 5 }; // Right/Left: 10, UP: 30 (Hard), DOWN: 5 (Easy)

		for(int i = 0; i < 4; i++)
		{
			int nx = cx + dx[i];
			int ny = cy + dy[i];

			if(nx < 0 || nx >= m_MapWidth || ny < 0 || ny >= m_MapHeight) continue;

			int NextIdx = ny * m_MapWidth + nx;
			
			// Collision Check (Skip solids)
			if(pCollision->IsSolid(nx * 32 + 16, ny * 32 + 16)) continue;

			// Phase 10: Freeze Penalty
			int Penalty = 0;
			int Tile = pCollision->GetTileIndex(NextIdx);
			int Front = pCollision->GetFrontTileIndex(NextIdx);
			if(Tile == TILE_FREEZE || Front == TILE_FREEZE) Penalty += 500;
			
			// Phase 22: Inject Trauma Penalty (Dynamic Death Avoidance)
			if (m_TraumaMap.count(NextIdx))
				Penalty += m_TraumaMap[NextIdx] * 100;

			int NewDist = CurrDist + costs[i] + Penalty;
			if(NewDist < m_DistanceMap[NextIdx])
			{
				m_DistanceMap[NextIdx] = NewDist;
				Queue.push(NextIdx);
			}
		}
	}

	// 3. Phase 17: Golden Thread (Wall Repulsion Pass)
	// Artificially increase cost of tiles near walls to force the path to the center.
	for(int y = 1; y < m_MapHeight - 1; y++)
	{
		for(int x = 1; x < m_MapWidth - 1; x++)
		{
			int Index = y * m_MapWidth + x;
			if(m_DistanceMap[Index] >= 1000000) continue;

			bool NearWall = false;
			for(int oy = -1; oy <= 1; oy++)
			{
				for(int ox = -1; ox <= 1; ox++)
				{
					if(pCollision->IsSolid((x + ox) * 32 + 16, (y + oy) * 32 + 16))
					{
						NearWall = true;
						break;
					}
				}
				if(NearWall) break;
			}
			if(NearWall) m_DistanceMap[Index] += 15; // Phase 17: Golden Thread cost
		}
	}

	dbg_msg("tas_bot", "Global Flow-Field (Golden Thread) computed for %dx%d map.", m_MapWidth, m_MapHeight);
}

int CTasBot::GetDistanceMapValue(vec2 Pos)
{
	if(m_DistanceMap.empty()) return 1000000;
	int tx = std::clamp((int)(Pos.x / 32.0f), 0, m_MapWidth - 1);
	int ty = std::clamp((int)(Pos.y / 32.0f), 0, m_MapHeight - 1);
	return m_DistanceMap[ty * m_MapWidth + tx];
}

vec2 CTasBot::GetFlowGradient(vec2 Pos)
{
	if(m_DistanceMap.empty()) return vec2(0, 0);
	int tx = std::clamp((int)(Pos.x / 32.0f), 1, m_MapWidth - 2);
	int ty = std::clamp((int)(Pos.y / 32.0f), 1, m_MapHeight - 2);

	// Find the adjacent tile with the lowest distance
	int MinDist = m_DistanceMap[ty * m_MapWidth + tx];
	vec2 BestDir = vec2(0, 0);

	int dx[] = { 1, -1, 0, 0, 1, 1, -1, -1 };
	int dy[] = { 0, 0, 1, -1, 1, -1, 1, -1 };

	for(int i = 0; i < 8; i++)
	{
		int d = m_DistanceMap[(ty + dy[i]) * m_MapWidth + (tx + dx[i])];
		if(d < MinDist)
		{
			MinDist = d;
			BestDir = vec2((float)dx[i], (float)dy[i]);
		}
	}

	if(length(BestDir) > 0.001f) return normalize(BestDir);
	return vec2(0, 0);
}

uint64_t CTasBot::GetStateHash(const CCharacterCore& Core)
{
	// 1. Map Position (Tile-based 32px resolution)
	int PosX = (int)(Core.m_Pos.x / 32.0f);
	int PosY = (int)(Core.m_Pos.y / 32.0f);
	
	// Phase 17: Robust Velocity Hashing (5 Broad States)
	// Reduced sensitivity to 1-pixel drifts and ping jitters.
	int VelX = (abs(Core.m_Vel.x) < 5.0f) ? 0 : (Core.m_Vel.x > 0 ? 1 : 2);
	int VelY = (abs(Core.m_Vel.y) < 5.0f) ? 0 : (Core.m_Vel.y > 0 ? 3 : 4);
	
	// Phase 10: BFS Distance Map Gradient (Smoothed direction)
	vec2 Grad = GetFlowGradient(Core.m_Pos);
	int gx = (Grad.x > 0.1f) ? 1 : (Grad.x < -0.1f ? 2 : 0);
	int gy = (Grad.y > 0.1f) ? 3 : (Grad.y < -0.1f ? 4 : 0);
	
	int Hook = (Core.m_HookState != 0);
	int Jump = (Core.m_Jumped & 1);
	
	// Phase 14: Future Trajectory Projection (Hazard Awareness)
	vec2 FuturePos = Core.m_Pos + (Core.m_Vel * 15.0f * 0.3f);
	int TileIdx = GameClient()->Collision()->GetPureMapIndex(FuturePos);
	bool Dangerous = GameClient()->Collision()->GetTileIndex(TileIdx) == TILE_FREEZE;
	int Hazard = Dangerous ? 1 : 0;
	
	uint64_t Hash = (uint64_t)(PosX & 0x7FFF); 
	Hash |= ((uint64_t)(PosY & 0x3FFF) << 15);
	Hash |= ((uint64_t)VelX << 29);
	Hash |= ((uint64_t)VelY << 34);
	Hash |= ((uint64_t)gx << 39);
	Hash |= ((uint64_t)gy << 44);
	Hash |= ((uint64_t)Hook << 49);
	Hash |= ((uint64_t)Jump << 50);
	Hash |= ((uint64_t)Hazard << 51);
	
	return Hash;
}

CNetObj_PlayerInput CTasBot::MapActionToInput(int Action, const CCharacterCore& Core, vec2 TargetPos)
{
	CNetObj_PlayerInput Input = {0};
	// Multi-Discrete Action Space (60 combinations)
	// Move: 0=None, 1=Left, 2=Right
	// Jump: 0=Off, 1=On
	// Hook Style: 0=None, 1=Flow-Field, 2-9=8 Directional Aiming
	
	int Move = Action % 3;
	int Jump = (Action / 3) % 2;
	int HookStyle = (Action / 6) % 10;
	
	if (Move == 1) Input.m_Direction = -1;
	else if (Move == 2) Input.m_Direction = 1;
	
	Input.m_Jump = Jump;
	
	if (HookStyle > 0)
	{
		Input.m_Hook = 1;
		vec2 IdealDir;
		if (HookStyle == 1) // Flow-Field Waypoint
			IdealDir = TargetPos - Core.m_Pos;
		else // 8-Way Aiming
		{
			float Angle = (HookStyle - 2) * 45.0f * (3.14159f / 180.0f);
			IdealDir = vec2(sin(Angle), -cos(Angle));
		}
		
		// Phase 14: Smart Aim Assist (Raycast Targeter)
		vec2 BestWall = FindBestHookTarget(Core.m_Pos, IdealDir, 380.0f);
		vec2 AimDir = (BestWall.x != 0 || BestWall.y != 0) ? (BestWall - Core.m_Pos) : IdealDir * 300.0f;
		
		Input.m_TargetX = (int)AimDir.x;
		Input.m_TargetY = (int)AimDir.y;
		
		// Phase 14: Hook Persistence (Muscle Fix)
		// m_Hook = 1 is already set, and we ensure it stays 1 for the duration of hold
	}
	
	return Input;
}

// Phase 64: The Sovereign Centaur (Native Forward-Simulator)
CTasBot::SOracleTrajectory CTasBot::VerifyTrajectoryPrior(const STasActionSequence& PriorSequence)
{
	SOracleTrajectory Result;
	Result.m_IsPhysicallyValid = true;
	Result.m_SurvivalTicks = 0;

	// 1. THE SINGULARITY CLONE
	CCharacter *pLocalChar = GameClient()->m_PredictedWorld.GetCharacterById(GameClient()->m_Snap.m_LocalClientId);
	if (!pLocalChar) return Result;
	CCharacterCore SimCore = pLocalChar->GetCore();
	CCollision *pCollision = GameClient()->Collision();
	
	// 2. THE DETERMINISTIC INNER LOOP
	for(int tick = 0; tick < PriorSequence.m_NumTicks && tick < 100; ++tick)
	{
		// Inject hallucinated input
		SimCore.m_Input = PriorSequence.m_Inputs[tick];

		// Execute DDNet Physics (Cloned)
		SimCore.Tick(true); 
		SimCore.Move();

		// 3. GEOMETRY VERIFICATION
		int TileIndex = pCollision->GetTile(round_to_int(SimCore.m_Pos.x) / 32, round_to_int(SimCore.m_Pos.y) / 32);

		if(TileIndex == TILE_DEATH || TileIndex == TILE_FREEZE) 
		{
			Result.m_IsPhysicallyValid = false;
			break; 
		}
		Result.m_SurvivalTicks++;
	}

	Result.m_EndPos = SimCore.m_Pos;
	Result.m_EndVel = SimCore.m_Vel;
	return Result;
}

void CTasBot::OnStateChange(int NewState, int OldState)
{
	if(NewState == IClient::STATE_LOADING || NewState == IClient::STATE_QUITTING)
	{
		StopTraining();
		if(m_pTasWorld)
		{
			delete m_pTasWorld;
			m_pTasWorld = nullptr;
		}
		
		m_HasMasterRun = false;
		m_MasterRun.clear();
		m_MasterRunPositions.clear();
		m_CurrentPath.clear();
		m_StateHistory.clear();
	}
}

void CTasBot::StartTraining()
{
	if(m_IsTraining) return;

	// Phase 6 Brain Wipe: Anatomical shift requires fresh brain
	dbg_msg("tas_bot", "UNIVERSAL RESET: Wiping legacy brain for Ego-Centric Architecture.");
	m_QTable.clear();
	unlink("tas_bot_brain.dat");

	LoadMemory(); // Try to load (will be empty)
	CCharacter *pLocalChar = GameClient()->m_PredictedWorld.GetCharacterById(GameClient()->m_Snap.m_LocalClientId);
	if(!pLocalChar) {
		dbg_msg("tas_bot", "Training deferred: Local character not found.");
		return;
	}
	if(!GameClient()->Collision()) {
		dbg_msg("tas_bot", "Training deferred: No collision map available.");
		return;
	}

	if(!m_pTasWorld)
	{
		m_pTasWorld = new CTasWorld(GameClient()->Collision());
	}

	m_StartCore = pLocalChar->GetCore();
	m_CurrentPath.clear();
	m_StateHistory.clear();
	m_IsTraining = true;
	m_Attempts = 0;
	m_TotalTicksSimulated = 0;
	m_ForceJumpNext = false;
	
	// Scan map for the closest finish tile
	CCollision *pCollision = GameClient()->Collision();
	if(!pCollision) return;

	int w = pCollision->GetWidth();
	int h = pCollision->GetHeight();
	vec2 BestGoal(0, 0);
	float MinDist = 999999.0f;
	
	for(int y = 0; y < h; y++)
	{
		for(int x = 0; x < w; x++)
		{
			int Index = y * w + x;
			int Tile = pCollision->GetTileIndex(Index);
			int Front = pCollision->GetFrontTileIndex(Index);
			if(Tile == TILE_FINISH || Front == TILE_FINISH)
			{
				vec2 Pos(x * 32.0f + 16.0f, y * 32.0f + 16.0f);
				float D = distance(m_StartCore.m_Pos, Pos);
				if(D < MinDist) { MinDist = D; BestGoal = Pos; }
			}
		}
	}
	
	if(MinDist < 999999.0f)
	{
		CAStarPathfinder AStar(GameClient()->Collision());
		m_AStarPath = AStar.FindPath(m_StartCore.m_Pos, BestGoal);
		
		// If path is empty, try a less strict start pos
		if(m_AStarPath.empty())
		{
			for(int r = 32; r < 128; r += 32) {
				m_AStarPath = AStar.FindPath(m_StartCore.m_Pos + vec2(r, 0), BestGoal);
				if(!m_AStarPath.empty()) break;
				m_AStarPath = AStar.FindPath(m_StartCore.m_Pos + vec2(-r, 0), BestGoal);
				if(!m_AStarPath.empty()) break;
			}
		}

		dbg_msg("tas_bot", "A* generated path with %d nodes to %.0f,%.0f", (int)m_AStarPath.size(), BestGoal.x, BestGoal.y);
	}
	else
	{
		dbg_msg("tas_bot", "No finish tile found for A*!");
		m_AStarPath.clear();
	}
	
	m_pTasWorld->LoadFromCurrentCore(m_StartCore);
	dbg_msg("tas_bot", "Training Started! Max depth: %d ticks", m_MaxSimulationDepth);
}

void CTasBot::StopTraining()
{
	if (m_IsTraining)
	{
		SaveMemory(); // Save brain on stop
		dbg_msg("tas_bot", "Training Aborted. Memory saved. Sim Ticks: %d, Attempts: %d", m_TotalTicksSimulated, m_Attempts);
	}
	m_IsTraining = false;
}

CNetObj_PlayerInput CTasBot::GetNextExplorationInput(bool ForceJump, vec2 CurrentPos)
{
	CNetObj_PlayerInput Input = {0};
	// Heuristic: Tend to move right
	Input.m_Direction = 1;
	
	if(m_AStarPath.size() > 0)
	{
		// Find closest node on path
		int BestIndex = -1;
		float MinDist = 999999.0f;
		for(size_t i = 0; i < m_AStarPath.size(); i++)
		{
			float D = distance(CurrentPos, m_AStarPath[i]);
			if(D < MinDist) { MinDist = D; BestIndex = (int)i; }
		}
		
		// Aim for a node slightly ahead (approx 2-4 tiles)
		int ForwardBias = 2 + (rand() % 3);
		int TargetIndex = std::min(BestIndex + ForwardBias, (int)m_AStarPath.size() - 1);
		if(TargetIndex >= 0)
		{
			vec2 TargetPos = m_AStarPath[TargetIndex];
			if(TargetPos.x > CurrentPos.x + 4.0f) Input.m_Direction = 1;
			else if(TargetPos.x < CurrentPos.x - 4.0f) Input.m_Direction = -1;
			else Input.m_Direction = 0;
			
			// If target is significantly higher, jump
			if(TargetPos.y < CurrentPos.y - 24.0f)
				Input.m_Jump = 1;

			// Look at target
			Input.m_TargetX = (int)(TargetPos.x - CurrentPos.x);
			Input.m_TargetY = (int)(TargetPos.y - CurrentPos.y);
		}
	}
	else
	{
		// Stuck-Suicide Logic (Trackmania restart mechanic)
		if(m_CurrentPath.size() % 60 == 0 && m_CurrentPath.size() > 0) // Every 1 second
		{
			if(distance(m_StateHistory.back().m_Pos, m_LastStuckPos) < 16.0f)
			{
				m_StuckTicks += 60;
			}
			else
			{
				m_StuckTicks = 0;
				m_LastStuckPos = m_StateHistory.back().m_Pos;
			}
			
			if(m_StuckTicks > 180) // Stuck for 3 seconds -> KILL
			{
				if (g_Config.m_TcBotLogging)
					dbg_msg("tas_bot", "AI is STUCK. Implementing SUICIDE (say /kill) to restart training.");
				m_CurrentPath.clear();
				m_StateHistory.clear();
				m_pTasWorld->LoadFromCurrentCore(m_StartCore);
				m_StuckTicks = 0;
				m_Attempts++;
				return Input; // Return early to restart the training step
			}
		}

		// 1. Exploration if we aren't following a strict path
		Input.m_Direction = (rand() % 100) < 80 ? 1 : -1;
	}
	
	if(ForceJump)
	{
		Input.m_Jump = 1;
	}
	else if(m_ForceHookNext)
	{
		Input.m_Hook = 1;
		m_ForceHookNext = false;
		
		// Kanca için rastgele yukarı bakış (Recover için)
		Input.m_TargetX = (rand() % 100) - 50;
		Input.m_TargetY = -100;
	}
	else
	{
		// 5% chance to randomly jump
		if((rand() % 100) < 5) Input.m_Jump = 1;
		
		// 2% chance to randomly hook
		if((rand() % 100) < 2) Input.m_Hook = 1;
	}
	
	return Input;
}

// Pro Pro-Level Reward Function (Trackmania Style)
vec2 CTasBot::GetDynamicWaypoint(vec2 BotPos)
{
	if(m_AStarPath.empty()) return BotPos;

	// Look for the first node that is at least ~3 tiles away
	for(const auto& Node : m_AStarPath)
	{
		if(distance(BotPos, Node) > 96.0f)
			return Node;
	}

	return m_AStarPath.back();
}

void CTasBot::SaveMemory()
{
	IOHANDLE File = io_open("tas_bot_brain.dat", IOFLAG_WRITE);
	if(!File) return;

	// Write Q-Table Size
	uint32_t Size = (uint32_t)m_QTable.size();
	io_write(File, &Size, sizeof(Size));

	for(auto const& [Hash, Values] : m_QTable)
	{
		io_write(File, &Hash, sizeof(Hash));
		uint32_t VecSize = 60; // Fixed size for std::array<float, 60>
		io_write(File, &VecSize, sizeof(VecSize));
		io_write(File, Values.data(), VecSize * sizeof(float));
	}
	io_close(File);
	dbg_msg("tas_bot", "Memory saved: %u states (Phase 11).", Size);
}

void CTasBot::LoadMemory()
{
	IOHANDLE File = io_open("tas_bot_brain.dat", IOFLAG_READ);
	if(!File) return;

	uint32_t Size = 0;
	io_read(File, &Size, sizeof(Size));

	for(uint32_t i = 0; i < Size; i++)
	{
		uint64_t Hash = 0;
		io_read(File, &Hash, sizeof(Hash));
		uint32_t VecSize = 0;
		io_read(File, &VecSize, sizeof(VecSize));
		
		std::array<float, 60> Values;
		Values.fill(0.0f);
		
		if(VecSize == 60)
		{
			io_read(File, Values.data(), 60 * sizeof(float));
		}
		else
		{
			// Handle legacy data or mixed sizes
			std::vector<float> Legacy(VecSize);
			io_read(File, Legacy.data(), VecSize * sizeof(float));
			for(uint32_t j = 0; j < std::min(VecSize, 60u); j++)
				Values[j] = Legacy[j];
		}
		m_QTable[Hash] = Values;
	}
	io_close(File);
	dbg_msg("tas_bot", "Memory loaded: %u states (Phase 11).", Size);
}

void CTasBot::UpdateQValue(uint64_t State, int Action, float Reward, uint64_t NextState)
{
	if(Action < 0 || Action >= 60) return;
	
	float MaxNextQ = -1e18f;
	for(int i = 0; i < 60; ++i) if(m_QTable[NextState][i] > MaxNextQ) MaxNextQ = m_QTable[NextState][i];

	// Phase 18: Optimized Hyperparameters for long-horizon platforming
	float Alpha = 0.2f; 
	float Gamma = 0.99f; 
	m_QTable[State][Action] += Alpha * (Reward + Gamma * MaxNextQ - m_QTable[State][Action]);
}

float CTasBot::EvaluateState(const CCandidatePath& Cand, const CCharacterCore& StartCore)
{
	if(Cand.m_IsDead) return -100000.0f; 

	float Score = 0.0f;
	vec2 FinalPos = Cand.m_FinalCore.m_Pos;
	
	// Trackmania RL: Extract core metrics
	float VelLen = length(Cand.m_FinalCore.m_Vel);
	vec2 VelDir = vec2(0,0);
	if(VelLen > 0.001f) VelDir = normalize(Cand.m_FinalCore.m_Vel);

	// 1. PATH PROGRESS REWARD
	if(!m_AStarPath.empty() && m_AStarPath.size() < 1000000) // Sanity check on path size
	{
		int StartIdx = -1;
		int EndIdx = -1;
		float MinStart = 1e9;
		float MinEnd = 1e9;
		
		for(int i = 0; i < (int)m_AStarPath.size(); i++) {
			float dS = distance(StartCore.m_Pos, m_AStarPath[i]);
			float dE = distance(FinalPos, m_AStarPath[i]);
			if(dS < MinStart) { MinStart = dS; StartIdx = i; }
			if(dE < MinEnd) { MinEnd = dE; EndIdx = i; }
		}
		
		if(EndIdx > StartIdx)
			Score += (EndIdx - StartIdx) * 50.0f; // Normalized reward
		else
			Score -= (StartIdx - EndIdx) * 20.0f;
			
		Score -= MinEnd * 2.0f; // Stay close to path

		// Momentum Reward: Bonus if velocity is aligned with the A* path direction
		if(EndIdx != -1 && EndIdx < (int)m_AStarPath.size() - 1)
		{
			vec2 PathDir = vec2(0,0);
			float PathLen = distance(m_AStarPath[EndIdx+1], m_AStarPath[EndIdx]);
			if(PathLen > 0.001f) PathDir = normalize(m_AStarPath[EndIdx+1] - m_AStarPath[EndIdx]);
			
			float Alignment = dot(VelDir, PathDir);
			if(Alignment > 0.5f) Score += Alignment * VelLen * 5.0f; 
		}
	}
	
	// 2. SPEED BONUS (Trackmania Style)
	Score += VelLen * 3.0f;
	
	// Momentum Reward: (Moved inside path block)

	// IDLING PENALTY: Force the bot to move!
	if(VelLen < 0.2f)
	{
		Score -= 1000.0f; // Massive penalty for standing still or being stuck
	}

	// 3. ENTITY REWARDS (Modular)
	if(g_Config.m_TcBotDDRaceAware && m_pTasWorld)
	{
		if(m_pTasWorld->ReachedFinish())
			Score += 10000.0f; // VICTORY!
			
		if(m_pTasWorld->IsFrozen())
			Score -= 5000.0f; // FREEZE IS ALMOST DEATH
	}
	
	// 4. WALL & FREEZE REPULSION (Proactive Steering)
	CCollision *pCollision = GameClient()->Collision();
	if(pCollision)
	{
		bool NearWall = pCollision->IsSolid(FinalPos.x + 18, FinalPos.y) || 
		                pCollision->IsSolid(FinalPos.x - 18, FinalPos.y) ||
		                pCollision->IsSolid(FinalPos.x, FinalPos.y + 18) ||
		                pCollision->IsSolid(FinalPos.x, FinalPos.y - 18);
		
		int Restrictions = pCollision->GetMoveRestrictions(FinalPos);
		if(Restrictions & (CANTMOVE_LEFT | CANTMOVE_RIGHT)) Score -= 1500.0f;
		else if(NearWall) Score -= 500.0f;
		
		// Proactive Freeze Avoidance: Check tiles in a small cross pattern around the bot
		bool NearFreeze = false;
		for(int ox = -40; ox <= 40; ox += 20)
		{
			for(int oy = -40; oy <= 40; oy += 20)
			{
				int Tile = pCollision->GetCollisionAt(FinalPos.x + ox, FinalPos.y + oy);
				if(Tile == TILE_FREEZE || Tile == TILE_DFREEZE || Tile == TILE_LFREEZE)
				{
					NearFreeze = true;
					break;
				}
			}
			if(NearFreeze) break;
		}
		
		if(NearFreeze) Score -= 800.0f; // Strong repulsion to stay away from freeze
	}

	// 5. JITTER PREVENTION
	if(Cand.m_Inputs[0].m_Direction == m_LastBestInput.m_Direction && m_LastBestInput.m_Direction != 0)
		Score += 10.0f;
	if(Cand.m_Inputs[0].m_Hook == m_LastBestInput.m_Hook && m_LastBestInput.m_Hook != 0)
		Score += 15.0f;

	// 6. DEATH/STUCK PENALTY
	if(Cand.m_IsDead) Score -= 5000.0f;
	if(distance(FinalPos, StartCore.m_Pos) < 1.0f) Score -= 100.0f;

	// 7. LEARNING MEMORY
	for(const auto& Death : m_DeathMemory)
	{
		float D = distance(FinalPos, Death.m_Pos);
		if(D < 64.0f) Score -= 2000.0f * (1.0f - (D / 64.0f)); 
	}

	return Score;
}

CNetObj_PlayerInput CTasBot::GetBestLiveInput(const CCharacterCore& CurrentCore)
{
	// Phase 19: Allow execution during Playback mode
	if(!m_IsTraining && !g_Config.m_TcTasBotPlayback) return {0};

	// 1. Gemini Pro 50Hz Master Tick-Lock
	int CurrentTick = GameClient()->Client()->GameTick(0);
	if(CurrentTick == m_LastAITick)
	{
		return m_LastInput;
	}
	m_LastAITick = CurrentTick;

	// === BELOW THIS LINE, EVERYTHING RUNS AT EXACTLY 50Hz ===
	
	// 2. COUNTERS & HISTORY UPDATE
	if(m_BannedActionTicks > 0) m_BannedActionTicks--;
	if(m_ActionHoldFrames > 0) 
	{
		m_ActionHoldFrames--;
		m_ActionHoldElapsed++;
	}

	// Update Position History (Sliding Window)
	int OldIdx = (m_PosHistoryIdx + 1) % 5;
	m_PosHistory[m_PosHistoryIdx] = CurrentCore.m_Pos;
	m_PosHistoryIdx = (m_PosHistoryIdx + 1) % 5;


	// 3. FRAME-HOLD: Persistence & Interrupt
	if(m_ActionHoldFrames > 0)
	{
		// COLLISION INTERRUPT: Component-wise Check + Engine Parity (MoveRestrictions)
		if(m_ActionHoldElapsed >= 5)
		{
			float dx = abs(CurrentCore.m_Pos.x - m_PosHistory[OldIdx].x);
			bool HorizontalStuck = (m_CurrentAction == 0 || m_CurrentAction == 1) && dx < 2.0f;
			bool WallLeft = GameClient()->Collision()->CheckPoint(CurrentCore.m_Pos + vec2(-15, 0));
			bool WallRight = GameClient()->Collision()->CheckPoint(CurrentCore.m_Pos + vec2(15, 0));
			bool IsBlocked = (m_CurrentAction == 0 && WallLeft) || (m_CurrentAction == 1 && WallRight);

			if(HorizontalStuck || IsBlocked)
			{
				dbg_msg("tas_bot", "INTERRUPT: Wall detected at 50Hz (dx:%.2f, Act:%d).", dx, m_CurrentAction);
				
				// Phase 19: No learning during Playback
				if (!g_Config.m_TcTasBotPlayback)
					UpdateQValue(m_LastStateHash, m_CurrentAction, -2000.0f, GetStateHash(CurrentCore));

				m_BannedAction = m_CurrentAction;
				m_BannedActionTicks = 25; 
				m_ActionHoldFrames = 3; // Phase 12: Enforce 3-frame hold to reduce jitter
				m_ActionHoldElapsed = 0;
			}
		}

		if(m_ActionHoldFrames > 0)
		{
			m_LastInput = MapActionToInput(m_CurrentAction, CurrentCore, CurrentCore.m_Pos + GetFlowGradient(CurrentCore.m_Pos) * 100.0f);
			if(m_CurrentAction == 2 && m_ActionHoldElapsed > 1) m_LastInput.m_Jump = 0;
			return m_LastInput;
		}
	}

	// 4. NEW DECISION ENGINE (50Hz)
	m_ActionHoldElapsed = 0;
	uint64_t State = GetStateHash(CurrentCore);

	// Phase 19: Zero-Epsilon Reality Lock
	// In the real world / Playback mode, we NEVER explore. Strict exploitation only.
	float SampleEpsilon = m_Epsilon;
	if(!m_InceptionActive || g_Config.m_TcTasBotPlayback) SampleEpsilon = 0.0f;

	int Action = -1;
	bool IsRandom = false;
	if((float)(rand() % 1000) / 1000.0f < SampleEpsilon)
	{
		IsRandom = true;
		do {
			Action = rand() % 60;
		} while(Action == m_BannedAction && m_BannedActionTicks > 0 && (rand() % 10) < 9); 
	}
	else
	{
		float MaxQ = -1e18f;
		bool HasKnowledge = false;
		for(int i = 0; i < 60; i++)
		{
			if(i == m_BannedAction && m_BannedActionTicks > 0) continue;
			if(m_QTable[State][i] != 0.0f) HasKnowledge = true;
			if(m_QTable[State][i] > MaxQ)
			{
				MaxQ = m_QTable[State][i];
				Action = i;
			}
		}
		
		// Phase 19 Fallback: If state is unknown and playing in reality or playback, stay neutral.
		if(!HasKnowledge && (!m_InceptionActive || g_Config.m_TcTasBotPlayback)) Action = 0;
		if(Action == -1) Action = 0;
	}

	m_CurrentAction = Action;
	
	// Forensic Trace
	dbg_msg("tas_bot", "RECAP 50Hz: State:%llu Act:%d (Rand:%d) Eps:%.3f PosX:%.1f", 
		State, Action, IsRandom, (float)m_Epsilon, CurrentCore.m_Pos.x);
	
	// Adaptive Hold Duration
	m_ActionHoldFrames = 5 + (int)(15.0f * (1.0f - m_Epsilon));
	m_ActionHoldElapsed = 0;
	m_LastStateHash = State;
	
	m_LastInput = MapActionToInput(m_CurrentAction, CurrentCore, CurrentCore.m_Pos + GetFlowGradient(CurrentCore.m_Pos) * 100.0f);
	return m_LastInput;
}

void CTasBot::StepTrainingBatch()
{
	if(!m_pTasWorld && GameClient()->Collision()) 
		m_pTasWorld = new CTasWorld(GameClient()->Collision());

	if(!m_pTasWorld) return;

	// Process up to 1000 simulated ticks per update with a 2ms time budget
	// This ensures the main thread doesn't starve, fixing network and UI freezes
	int64_t StartTime = time_get();
	int64_t Budget = time_freq() / 500; // 2ms budget
	
	for(int i = 0; i < 1000; i++)
	{
		// Check budget every 10 iterations for efficiency
		if(i % 10 == 0 && (time_get() - StartTime) > Budget)
			break;
		if(m_CurrentPath.size() >= (size_t)m_MaxSimulationDepth)
		{
			// Prune branch because it's taking too long
			m_CurrentPath.clear();
			m_StateHistory.clear();
			m_pTasWorld->LoadFromCurrentCore(m_StartCore);
			m_Attempts++;
			m_ForceJumpNext = false;
			continue;
		}

		// Save the state BEFORE we apply input
		m_StateHistory.push_back(m_pTasWorld->GetCore());

		CNetObj_PlayerInput Input = GetNextExplorationInput(m_ForceJumpNext, m_pTasWorld->GetCore().m_Pos);
		m_ForceJumpNext = false; // Reset force flag
		m_CurrentPath.push_back(Input);
		
		m_pTasWorld->SimulateTick(Input);
		m_TotalTicksSimulated++;

		// 1. Success check
		if(m_pTasWorld->ReachedFinish())
		{
			bool BetterRun = !m_HasMasterRun || (int)m_CurrentPath.size() < (int)m_MasterRun.size();
			
			if(BetterRun)
			{
				m_MasterRun = m_CurrentPath;
				
				// Save the expected physical coordinates for each tick of the successful run
				m_MasterRunPositions.clear();
				for(size_t j = 0; j < m_StateHistory.size(); j++)
				{
					m_MasterRunPositions.push_back(m_StateHistory[j].m_Pos);
				}
				m_MasterRunPositions.push_back(m_pTasWorld->GetCore().m_Pos); // Final tick pos
				
				m_HasMasterRun = true;
				dbg_msg("tas_bot", "TAS RUN FOUND (Length: %d ticks)! Attempts: %d | Total Sim Ticks: %d", 
					(int)m_MasterRun.size(), m_Attempts, m_TotalTicksSimulated);

				// Auto-save the run
				SaveTasRun(GameClient()->Map()->BaseName());
			}

			// If not in speedrun mode, stop training immediately
			if(!g_Config.m_ClTasSpeedrun)
			{
				m_IsTraining = false;
				m_PlaybackTick = 0;
				break;
			}
			else
			{
				// Speedrun mode: Reset and try to find a faster path
				m_Attempts++;
				m_CurrentPath.clear();
				m_StateHistory.clear();
				m_pTasWorld->LoadFromCurrentCore(m_StartCore);
				continue;
			}
		}
		
		// 2. Danger check (Trigger Backtracking)
		if(m_pTasWorld->IsDeadly() || m_pTasWorld->IsFrozen())
		{
			// --- HIGH-POWER TRAJECTORY OPTIMIZATION ---
			m_Attempts++;
			std::vector<CCandidatePath> Candidates;
			CCharacterCore StartBatchCore = m_StateHistory.back();
			
			for(int c = 0; c < m_NumCandidates; c++)
			{
				CCandidatePath Cand;
				m_pTasWorld->LoadFromCurrentCore(StartBatchCore);
				Cand.m_IsDead = false;
				
				// Generate a specialized strategy for this candidate (Upgraded to 18 states)
				int Strategy = c % 18; 
				for(int t = 0; t < m_CandidateTicks; t++)
				{
				// 3. Map Action to Input
				CNetObj_PlayerInput CandInput;
				CandInput = MapActionToInput(Strategy, StartBatchCore, StartBatchCore.m_Pos + GetFlowGradient(StartBatchCore.m_Pos) * 100.0f);
					if(Strategy == 17) { CandInput.m_Direction = (rand()%3)-1; CandInput.m_Jump = rand()%2; CandInput.m_Hook = rand()%2; }
					
					m_pTasWorld->SimulateTick(CandInput);
					Cand.m_Inputs.push_back(CandInput);
					if(m_pTasWorld->IsDeadly() || m_pTasWorld->IsFrozen()) { Cand.m_IsDead = true; break; }
				}
				
				Cand.m_FinalCore = m_pTasWorld->GetCore();
				
				// Evaluate State using Trackmania Tactics Reward Function
				Cand.m_Score = EvaluateState(Cand, StartBatchCore);
				Candidates.push_back(Cand);
			}
			
			// Select Best Candidate
			int BestIdx = -1;
			float BestScore = -1e9;
			for(int k = 0; k < (int)Candidates.size(); k++)
			{
				if(Candidates[k].m_Score > BestScore) { BestScore = Candidates[k].m_Score; BestIdx = k; }
			}
			
			if(BestIdx != -1 && !Candidates[BestIdx].m_IsDead)
			{
				for(const auto& Inp : Candidates[BestIdx].m_Inputs)
				{
					m_CurrentPath.push_back(Inp);
					// Note: We'd need to simulate again to get exact state history or store it
					// For now, let's just append the best found sequence
				}
				// Re-sim to update world and history properly
				m_pTasWorld->LoadFromCurrentCore(StartBatchCore);
				for(const auto& Inp : Candidates[BestIdx].m_Inputs)
				{
					m_pTasWorld->SimulateTick(Inp);
					m_StateHistory.push_back(m_pTasWorld->GetCore());
				}
			}
			else
			{
				// Backtrack if no safe path found
				int BacktrackTicks = 20 + (rand() % 40);
				if((int)m_CurrentPath.size() > BacktrackTicks)
				{
					m_CurrentPath.resize(m_CurrentPath.size() - BacktrackTicks);
					m_StateHistory.resize(m_StateHistory.size() - BacktrackTicks);
				}
				else
				{
					m_CurrentPath.clear();
					m_StateHistory.clear();
				}
				m_pTasWorld->LoadFromCurrentCore(m_StateHistory.empty() ? m_StartCore : m_StateHistory.back());
			}
		}
	}
}

void CTasBot::StartPlayback()
{
	if(!m_HasMasterRun)
	{
		dbg_msg("tas_bot", "No master run found! Train first.");
		return;
	}
	m_IsTraining = false;
	m_PlaybackTick = 0;
	g_Config.m_ClTasPlayback = 1;
	dbg_msg("tas_bot", "Playback started.");
}
void CTasBot::OnUpdate()
{
	static int s_TraceCounter = 0;
	if (s_TraceCounter++ % 100 == 0) dbg_msg("tas_bot_trace", "OnUpdate Entry (Tick %d)", GameClient()->Client()->GameTick(0));

	// === SAFETY GATE ===
	CCharacter *pLocalChar = GameClient()->m_PredictedWorld.GetCharacterById(GameClient()->m_Snap.m_LocalClientId);
	if (!pLocalChar) return;
	CCharacterCore CurrentCore = pLocalChar->GetCore();

	// Phase 62: LPSD (Latent Phase-Space Diffusion) State Sync
	if (m_LPSDActive && m_pDiffusionManifold) {
		m_pDiffusionManifold->current_pos_x = CurrentCore.m_Pos.x;
		m_pDiffusionManifold->current_pos_y = CurrentCore.m_Pos.y;
		m_pDiffusionManifold->current_vel_x = CurrentCore.m_Vel.x;
		m_pDiffusionManifold->current_vel_y = CurrentCore.m_Vel.y;
		m_pDiffusionManifold->current_tick_index++; // Heatbeat for Python Denoising

		// If LPSD playback is active, intercept control
		if (g_Config.m_ClTasPlayback == 2) { // Mode 2 = LPSD Direct Manifold
			int local_idx = (m_pDiffusionManifold->current_tick_index % 100);
			int action = m_pDiffusionManifold->predicted_actions[local_idx];
			
			// Inject action directly into the engine
			CNetObj_PlayerInput Inp = {0};
			Inp.m_Direction = (action == 1) ? -1 : (action == 2 ? 1 : 0);
			Inp.m_Jump = (action == 3) ? 1 : 0;
			Inp.m_Hook = (action == 4) ? 1 : 0;
			
			// Force apply (Oracle Override)
			GameClient()->m_Controls.m_aInputData[GameClient()->m_Snap.m_LocalClientId] = Inp;
			return; // Bypass traditional decision logic
		}
	}

	// Phase 57: Protocol Injector - Aggressive Initialization
	if (m_MapWidth <= 0 || m_DistanceMap.empty() || m_DistanceMap[0] == 1000000) {
		CCollision *pColl = GameClient()->Collision();
		if(pColl && pColl->GetWidth() > 0)
		{
			dbg_msg("tas_bot", "Map Content Detected (%dx%d): Building Navigation Engine...", pColl->GetWidth(), pColl->GetHeight());
			ComputeDistanceMap();
			if (!m_pPathfinder) m_pPathfinder = new CAStarPathfinder(pColl);
			dbg_msg("tas_bot", "Navigation Ready.");
		}
	}

	if(!g_Config.m_ClTasTraining && !g_Config.m_TcTasBotPlayback)
	{
		StopTraining();
		return;
	}

	if(!m_IsTraining && g_Config.m_ClTasTraining) StartTraining();

	int CurrentTick = GameClient()->Client()->GameTick(0);
	if(CurrentTick <= m_LastUpdateTick) return;
	m_LastUpdateTick = CurrentTick;

	// Phase 8: Inception (Local TAS World Hyper-Simulation)
	if (g_Config.m_ClTasBotInception && g_Config.m_ClTasTraining)
	{
		if (!m_InceptionActive)
		{
			ResetInception();
			m_InceptionActive = true;
			dbg_msg("tas_bot", "INCEPTION MODE ENABLED: Dreaming at light speed...");
		}

		// Phase 16: Decoupled Simulation Speed (Unflickered UI)
		int SimTicks = g_Config.m_TcTasInceptionSpeed / 50;
		if(SimTicks < 1) SimTicks = 1;

		for(int i = 0; i < SimTicks; i++)
			RunInceptionTick();
	}
	else if (m_InceptionActive)
	{
		m_InceptionActive = false;
		dbg_msg("tas_bot", "INCEPTION MODE DISABLED: Waking up.");
	}

	// Phase 19: GPS FIX - Continuous Dynamic Recalculation (Shared by Training & Playback)
	if(m_LastRecalculateTick == -1 || CurrentTick > m_LastRecalculateTick + 25)
	{
		m_LastRecalculateTick = CurrentTick;
		CCharacter *pLocalChar = GameClient()->m_PredictedWorld.GetCharacterById(GameClient()->m_Snap.m_LocalClientId);
		if(pLocalChar && !m_AStarPath.empty())
		{
			vec2 Goal = m_AStarPath.back();
			m_AStarPath = m_pPathfinder->FindPath(pLocalChar->GetCore().m_Pos, Goal);
			dbg_msg("tas_bot", "GPS FIXED: Recalculated path at tick %d", CurrentTick);
		}
	}

	// Phase 7: Hyper-Simulation (Speed Hack)
	if(g_Config.m_ClTasTraining && g_Config.m_ClTasHyperTrain)
	{
		if(!m_HyperSimulationActive)
		{
			m_HyperSimulationActive = true;
			GameClient()->Client()->Rcon("sv_test_cmds 1");
			GameClient()->Client()->Rcon("speed 20");
			dbg_msg("tas_bot", "HYPER-SIMULATION ENABLED: TimeScale 20x");
		}
	}
	else if(m_HyperSimulationActive)
	{
		m_HyperSimulationActive = false;
		GameClient()->Client()->Rcon("speed 1");
		dbg_msg("tas_bot", "HYPER-SIMULATION DISABLED: TimeScale 1x");
	}

	// === Q-LEARNING UPDATE ===
	pLocalChar = GameClient()->m_PredictedWorld.GetCharacterById(GameClient()->m_Snap.m_LocalClientId);
	if(pLocalChar)
	{
		CurrentCore = pLocalChar->GetCore();
		uint64_t CurrentState = GetStateHash(CurrentCore);
		
		// Phase 10: Dynamic Reward Evaluation (Real-time)
		CCollision *pCollision = GameClient()->Collision();
		// Phase 62: Hierarchical Objective Fusion (H-OF)
		float CurrentPotential = (float)GetDistanceMapValue(CurrentCore.m_Pos);
		m_LastDistToGoal = CurrentPotential;
		
		// 1. Macro: Path Progress (A* Flow-Field)
		m_CurrentReward.macro_path_progress = (float)m_LiveLastDist - CurrentPotential;
		m_LiveLastDist = (int)CurrentPotential;

		// 2. Meso: Trauma Avoidance (Phase-Space Density)
		float trauma_level = GetPhaseSpaceTrauma(CurrentCore.m_Pos, CurrentCore.m_Vel);
		m_CurrentReward.meso_trauma_avoidance = 1.0f - trauma_level;

		// 3. Micro: Kinetic Energy (KOG Momentum Preservation)
		float current_speed_sq = (CurrentCore.m_Vel.x * CurrentCore.m_Vel.x) + (CurrentCore.m_Vel.y * CurrentCore.m_Vel.y);
		m_CurrentReward.micro_kinetic_energy = 0.5f * current_speed_sq;

		// Total Unified Reward for Neural Swarm (Backward Compatibility)
		float total_reward = (m_CurrentReward.macro_path_progress * 100.0f) + 
		                    (m_CurrentReward.meso_trauma_avoidance * 50.0f) + 
		                    (m_CurrentReward.micro_kinetic_energy * 5.0f);

		// Telemetry Heartbeat Update
		char aBuf[256];
		str_format(aBuf, sizeof(aBuf), "[HEARTBEAT] Dist: %.2f M:%.2f m:%.2f K:%.2f", 
			CurrentPotential, m_CurrentReward.macro_path_progress, 
			m_CurrentReward.meso_trauma_avoidance, m_CurrentReward.micro_kinetic_energy);
		
		DumpTelemetry("HEARTBEAT", CurrentCore.m_Pos);
		if (g_Config.m_TcBotLogging) dbg_msg("tas_bot", "%s", aBuf);

		// 3. Static Stalling Check
		float current_speed = sqrt(current_speed_sq);
		bool on_ice = pCollision->GetTile(round_to_int(CurrentCore.m_Pos.x) / 32, round_to_int(CurrentCore.m_Pos.y) / 32) == TILE_FREEZE;
		if (current_speed < 1.0f && !on_ice) total_reward -= (200.0f * m_ExplorationBoost);

		// 4. Goal Proximity Multiplier
		float finish_multiplier = 1.0f + (8000.0f / (CurrentPotential + 1.0f));
		total_reward *= finish_multiplier;

		float Reward = total_reward;
		// [END_REWARD]
		ExportNeuralState(Reward);

		// 1. Update REAL knowledge
		// Phase 19: Disable learning during Playback
		if (!g_Config.m_TcTasBotPlayback)
			UpdateQValue(m_LastStateHash, m_CurrentAction, Reward, CurrentState);

		// 2. Dyna-Q: Store Experience
		CTransition Trans = { m_LastStateHash, m_CurrentAction, Reward, CurrentState };
		m_ExperienceBuffer.push_back(Trans);
		if((int)m_ExperienceBuffer.size() > MAX_EXPERIENCE) m_ExperienceBuffer.erase(m_ExperienceBuffer.begin());

		// 3. Dyna-Q: Background Replay (10x faster learning)
		if(!m_ExperienceBuffer.empty())
		{
			for(int r = 0; r < 10; r++)
			{
				int idx = rand() % m_ExperienceBuffer.size();
				const auto& t = m_ExperienceBuffer[idx];
				UpdateQValue(t.m_State, t.m_Action, t.m_Reward, t.m_NextState);
			}
		}

		m_LastStateHash = CurrentState;
		
		// 3. Auto-Kill / Stuck Detection (Consolidated V2)
		bool IsFrozenLive = false;
		{
			int tx = (int)(CurrentCore.m_Pos.x / 32.0f);
			int ty = (int)(CurrentCore.m_Pos.y / 32.0f);
			if(tx >= 0 && tx < m_MapWidth && ty >= 0 && ty < m_MapHeight)
			{
				int Tile = pCollision->GetTileIndex(ty * m_MapWidth + tx);
				int Front = pCollision->GetFrontTileIndex(ty * m_MapWidth + tx);
				if(Tile == TILE_FREEZE || Front == TILE_FREEZE) IsFrozenLive = true;
			}
		}


		if(IsFrozenLive || distance(pLocalChar->GetCore().m_Pos, m_LastUpdatePos) < 5.0f)
			m_StuckCounter++;
		else
		{
			m_StuckCounter = 0;
			m_LastUpdatePos = pLocalChar->GetCore().m_Pos;
		}

		// Phase 20: Start-Area Awareness & Grace Period
		bool StandingOnStart = false;
		int TileIdx = pCollision->GetPureMapIndex(pLocalChar->GetCore().m_Pos);
		if(TileIdx >= 0 && (pCollision->GetTileIndex(TileIdx) == TILE_START || pCollision->GetFrontTileIndex(TileIdx) == TILE_START))
			StandingOnStart = true;

		if(m_StuckCounter > 50 * 3) // 3 Seconds (50Hz)
		{
			if (m_TicksSinceSpawn < 150 || StandingOnStart) {
				m_StuckCounter = 0; // Reset timer, we are either too new or at a safe zone
			} else {
				dbg_msg("tas_bot", "STUCK/FREEZE (3s): Auto-Resetting.");
				GameClient()->Console()->ExecuteLine("say /kill", IConsole::CLIENT_ID_GAME);
				DumpTelemetry("[STUCK_KILL]", CurrentCore.m_Pos); // Telemetry for stuck kill
			}
			m_LiveLastDist = 0.0f;
		m_DreamLastDist = 0.0f;
			m_LastUpdatePos = vec2(0,0);
			m_ActionHoldFrames = 0; 
			m_Epsilon *= 0.999f; 
			
			static int s_ResetCounter = 0;
			if(++s_ResetCounter % 10 == 0) SaveMemory();
		}
	}
	else
	{
		// Auto-Respawn: If dead, try to kill/jump to come back
		if(g_Config.m_TcTasBotAutoKill && CurrentTick % 150 == 0) // Every 3 seconds
		{
			// Phase 20: Only pulse if we have been missing for a while (Grace)
			// (m_TicksSinceSpawn is usually 0 when character is missing)
			dbg_msg("tas_bot", "Character missing: Sending respawn pulse (say /kill).");
			GameClient()->Console()->ExecuteLine("say /kill", IConsole::CLIENT_ID_GAME);
		}
	}
	{
		// Only step training if we have a valid character to sample from
		if(pLocalChar)
			StepTrainingBatch();
	}
}

void CTasBot::OnRender()
{
	// === SAFETY GATE ===
	if(!GameClient()->Collision() || GameClient()->m_Snap.m_LocalClientId < 0)
		return;
		
	static int s_TraceCounterRender = 0;
	if (s_TraceCounterRender++ % 100 == 0) dbg_msg("tas_bot_trace", "OnRender Entry");

	if(m_InceptionActive)
		RenderInceptionGhost();

	// Draw A* Path for legacy reference (if exists)
	if(!m_AStarPath.empty())
	{
		Graphics()->TextureClear();
		Graphics()->LinesBegin();
		for(size_t i = 1; i < m_AStarPath.size(); i++)
		{
			vec2 p0 = m_AStarPath[i-1];
			vec2 p1 = m_AStarPath[i];
			Graphics()->SetColor(0, 1, 0, 0.8f);
			IGraphics::CLineItem Line(p0.x, p0.y, p1.x, p1.y);
			Graphics()->LinesDraw(&Line, 1);
		}
		Graphics()->LinesEnd();
	}

	// AI Debug HUD (User Request: "Canli Log/SS")
	if(m_IsTraining)
	{
		CCharacter *pLocalChar = GameClient()->m_PredictedWorld.GetCharacterById(GameClient()->m_Snap.m_LocalClientId);
		if(pLocalChar)
		{
			float Vel = length(pLocalChar->GetCore().m_Vel);
			bool Hooked = pLocalChar->GetCore().m_HookState == HOOK_GRABBED;
			
			Graphics()->TextureClear();
			Graphics()->LinesBegin();
			// Velocity Bar (Yellow)
			Graphics()->SetColor(1, 1, 0, 1);
			IGraphics::CLineItem VelLine(pLocalChar->GetCore().m_Pos.x - 32, pLocalChar->GetCore().m_Pos.y - 60, pLocalChar->GetCore().m_Pos.x - 32 + Vel*2, pLocalChar->GetCore().m_Pos.y - 60);
			Graphics()->LinesDraw(&VelLine, 1);
			
			// Hook Status (Cyan if grabbed, Red if not)
			if(Hooked) Graphics()->SetColor(0, 1, 1, 1);
			else Graphics()->SetColor(1, 0, 0, 1);
			IGraphics::CLineItem HookLine(pLocalChar->GetCore().m_Pos.x - 32, pLocalChar->GetCore().m_Pos.y - 55, pLocalChar->GetCore().m_Pos.x + 32, pLocalChar->GetCore().m_Pos.y - 55);
			Graphics()->LinesDraw(&HookLine, 1);
			
			// Phase 14: Flow-Field Direction (Cyan)
			vec2 Grad = GetFlowGradient(pLocalChar->GetCore().m_Pos);
			Graphics()->SetColor(0.0f, 1.0f, 1.0f, 0.8f);
			IGraphics::CLineItem GradLine(pLocalChar->GetCore().m_Pos.x, pLocalChar->GetCore().m_Pos.y, 
				pLocalChar->GetCore().m_Pos.x + Grad.x * 64.0f, pLocalChar->GetCore().m_Pos.y + Grad.y * 64.0f);
			Graphics()->LinesDraw(&GradLine, 1);
			
			// Phase 14: Hazard Projection (Red Line if dangerous)
			vec2 FuturePos = pLocalChar->GetCore().m_Pos + (pLocalChar->GetCore().m_Vel * 15.0f * 0.3f);
			if(GameClient()->Collision()->CheckPoint(FuturePos))
			{
				Graphics()->SetColor(1.0f, 0.0f, 0.0f, 1.0f);
				IGraphics::CLineItem HazLine(pLocalChar->GetCore().m_Pos.x, pLocalChar->GetCore().m_Pos.y, FuturePos.x, FuturePos.y);
				Graphics()->LinesDraw(&HazLine, 1);
			}
			
			Graphics()->LinesEnd();
		}
	}
}

vec2 CTasBot::FindBestHookTarget(vec2 BotPos, vec2 IdealDir, float Radius)
{
	CCollision *pCollision = GameClient()->Collision();
	if(!pCollision) return vec2(0, 0);

	vec2 BestTarget(0, 0);
	float BestValue = -1e9;
	
	// Phase 14: Smart Grapple Sweep (-45 to +45 deg around IdealDir)
	float BaseAngle = atan2(IdealDir.y, IdealDir.x);
	for(float AngleOffset = -0.785f; AngleOffset <= 0.785f; AngleOffset += 0.1f) // ~5 deg increments
	{
		float ScanAngle = BaseAngle + AngleOffset;
		vec2 ScanDir(cos(ScanAngle), sin(ScanAngle));
		vec2 To = BotPos + ScanDir * Radius;
		
		vec2 OutPos;
		if(pCollision->IntersectLine(BotPos, To, nullptr, &OutPos))
		{
			// Check if the tile we hit is actually hookable (Solid and not No-Hook)
			int tx = (int)(OutPos.x / 32.0f);
			int ty = (int)(OutPos.y / 32.0f);
			
			// Pillar 3: Absolute Bounds Check
			if(tx < 0 || ty < 0 || tx >= pCollision->GetWidth() || ty >= pCollision->GetHeight())
				continue;

			int Tile = pCollision->GetTileIndex(ty * pCollision->GetWidth() + tx);
			int Front = pCollision->GetFrontTileIndex(ty * pCollision->GetWidth() + tx);
			
			if(Tile != TILE_NOHOOK && Front != TILE_NOHOOK)
			{
				float Dist = distance(BotPos, OutPos);
				if(Dist > 64.0f)
				{
					// Prefer higher walls and direction alignment
					float Val = (BotPos.y - OutPos.y) + dot(ScanDir, IdealDir) * 100.0f;
					
					// Pillar 3: Hazard Awareness (Freeze adjacency)
					bool NearFreeze = false;
					for(int ox = -1; ox <= 1; ox++) {
						for(int oy = -1; oy <= 1; oy++) {
							int nx = tx + ox, ny = ty + oy;
							if(nx < 0 || ny < 0 || nx >= pCollision->GetWidth() || ny >= pCollision->GetHeight()) continue;
							int t = pCollision->GetTileIndex(ny * pCollision->GetWidth() + nx);
							int f = pCollision->GetFrontTileIndex(ny * pCollision->GetWidth() + nx);
							if(t == TILE_FREEZE || f == TILE_FREEZE) { NearFreeze = true; break; }
						}
						if(NearFreeze) break;
					}
					if(NearFreeze) Val -= 1000.0f; // Strongly discourage

					if(Val > BestValue)
					{
						BestValue = Val;
						BestTarget = OutPos;
					}
				}
			}
		}
	}
	return BestTarget;
}

void CTasBot::OnBeforeInput(CNetObj_PlayerInput *pInput)
{
	static int s_TraceCounterInput = 0;
	if (s_TraceCounterInput++ % 100 == 0) dbg_msg("tas_bot_trace", "OnBeforeInput Entry (Tick %d)", GameClient()->Client()->GameTick(0));

	// === MASTER KILLSWITCH (Phase 20) ===
	if (!g_Config.m_TcTasBotTraining && !g_Config.m_TcTasBotPlayback && !g_Config.m_ClTasTraining && !g_Config.m_ClTasPlayback)
		return;

	CCollision *pCollision = GameClient()->Collision();
	if(!pCollision || GameClient()->m_Snap.m_LocalClientId < 0)
		return;
		
	// Phase 20: Brain Warm-Up. Give reality a head start.
	if (m_TicksSinceSpawn < 50) {
		m_TicksSinceSpawn++; // Increment even while waiting
		return;
	}

	CCharacter *pLocalChar = GameClient()->m_PredictedWorld.GetCharacterById(GameClient()->m_Snap.m_LocalClientId);
	if(pLocalChar)
	{
		m_TicksSinceSpawn++; // Grace period tracking (Phase 20)
		if(m_TicksSinceSpawn < 150) {
			if(m_TicksSinceSpawn % 50 == 0) dbg_msg("tas_bot", "GRACE PERIOD: Skipping brain for %d ticks...", 150 - m_TicksSinceSpawn);
			return;
		}
	}

	// === SAFETY GATE: Resolve Logic Gate Paradox (V6) ===
	// Allow processing if either Playback OR Training is active
	if(!g_Config.m_ClTasPlayback && !m_IsTraining && !g_Config.m_TcTasBotPlayback)
	{
		m_PlaybackTick = 0;
		return;
	}

	// Phase 48: CENTAUR HYBRID NAVIGATOR V4 (A* + Neural + PE-G)
	if(g_Config.m_TcBotEnabled)
	{
		CCharacter *pLocalChar = GameClient()->m_PredictedWorld.GetCharacterById(GameClient()->m_Snap.m_LocalClientId);
		if(pLocalChar)
		{
			// A. MACRO-NAVIGATOR (A* / Pathfinding)
			vec2 Target = GetDynamicWaypoint(pLocalChar->m_Pos);
			float Dist = distance(pLocalChar->m_Pos, Target);
			m_LastDistToGoal = Dist; // Sync for Telemetry
			
			// B. STABILITY NODE (User Request: Stand Still)
			if (Dist < 24.0f && length(pLocalChar->m_Core.m_Vel) < 1.0f)
			{
				pInput->m_Direction = 0;
				pInput->m_Jump = 0;
				pInput->m_Hook = 0;
			}
			else
			{
				// C. NEURAL MICRO-REFLEX
				RunNeuralForwardPass();
				
				// D. HYBRID MERGE
				pInput->m_Direction = (Target.x > pLocalChar->m_Pos.x) ? 1 : -1;
				pInput->m_Jump = (m_NeuralOutput[0] > 0.5f) ? 1 : 0;
				pInput->m_Hook = (m_NeuralOutput[1] > 0.5f) ? 1 : 0;
			}
			
			// E. SSAS EXPORT (PE-G Optimized telemetry)
			ExportNeuralState(1.0f / (1.0f + Dist));
		}
		return;
	}
}

void CTasBot::RenderInceptionGhost()
{
	if (!m_InceptionActive) return;

	CCharacter *pLocalChar = GameClient()->m_PredictedWorld.GetCharacterById(GameClient()->m_Snap.m_LocalClientId);
	if (!pLocalChar) return;

	CTeeRenderInfo RenderInfo = GameClient()->m_aClients[GameClient()->m_Snap.m_LocalClientId].m_RenderInfo;
	for(int i = 0; i < 6; i++) RenderInfo.m_aSixup[0].m_aColors[i].a *= 0.5f; // 50% Alpha Ghost

	// Create a stable animation state for the ghost (Base pose) to avoid nullptr crash
	CAnimState Anim;
	Anim.Set(&g_pData->m_aAnimations[ANIM_BASE], 0.0f);

	// Explicitly render the ghost tee
	RenderTools()->RenderTee(&Anim, &RenderInfo, 0, vec2(1, 0), m_SimulatedCore.m_Pos);

	// Phase 10/11: Render Flow-Field Gradient Line (Cyan)
	vec2 Grad = GetFlowGradient(m_SimulatedCore.m_Pos);
	Graphics()->TextureClear();
	Graphics()->LinesBegin();
	Graphics()->SetColor(0.0f, 1.0f, 1.0f, 0.6f); 
	IGraphics::CLineItem Line(m_SimulatedCore.m_Pos.x, m_SimulatedCore.m_Pos.y, 
		m_SimulatedCore.m_Pos.x + Grad.x * 64.0f, m_SimulatedCore.m_Pos.y + Grad.y * 64.0f);
	Graphics()->LinesDraw(&Line, 1);
	Graphics()->LinesEnd();
}

void CTasBot::DumpTelemetry(const char *Tag, vec2 Pos)
{
	if (!g_Config.m_TcBotLogging) return;
	
	char aBuf[256];
	if (str_comp(Tag, "[HEARTBEAT]") == 0)
	{
		str_format(aBuf, sizeof(aBuf), "%s Dist: %.2f Pos: %.1f, %.1f", Tag, m_LastDistToGoal, Pos.x, Pos.y);
	}
	else
	{
		str_format(aBuf, sizeof(aBuf), "%s Pos: %.1f, %.1f", Tag, Pos.x, Pos.y);
	}
	
	// Direct append to telemetry log
	IOHANDLE File = io_open("tas_telemetry_matrix.log", IOFLAG_APPEND);
	if(File)
	{
		io_write(File, aBuf, str_length(aBuf));
		io_write(File, "\n", 1);
		io_close(File);
	}
}

void CTasBot::LoadTraumaMap()
{
	m_TraumaMap.clear();
	
	std::ifstream f("trauma_map.json");
	if (!f.is_open()) return;
	
	std::string line;
	while (std::getline(f, line))
	{
		// Simple pattern matching for {"x": X, "y": Y, "deaths": D}
		// Since we generate this via Python, it will be standard.
		int x, y, deaths;
		if (sscanf(line.c_str(), " {\"x\": %d, \"y\": %d, \"deaths\": %d}", &x, &y, &deaths) == 3 ||
		    sscanf(line.c_str(), "{\"x\": %d, \"y\": %d, \"deaths\": %d}", &x, &y, &deaths) == 3)
		{
			int tileIdx = y * m_MapWidth + x;
			m_TraumaMap[tileIdx] = deaths;
		}
	}
	f.close();
	
	if (!m_TraumaMap.empty())
		dbg_msg("tas_bot", "Phase 22: Trauma Map loaded with %d hotspots.", (int)m_TraumaMap.size());
}


void CTasBot::UpdateTapMatrix(vec2 Pos, vec2 Vel, float Trauma) {
	m_TapMatrix.push_back({Pos, Vel, Trauma});
	if (m_TapMatrix.size() > 5000) m_TapMatrix.erase(m_TapMatrix.begin());
}

float CTasBot::GetPhaseSpaceTrauma(vec2 Pos, vec2 Vel) {
	float total = 0.0f;
	for (auto& e : m_TapMatrix) {
		float d = distance(Pos, e.m_Pos);
		if (d < 128.0f) {
			float align = 0.0f;
			if (length(Vel) > 0.1f && length(e.m_Vel) > 0.1f)
				align = std::max(0.0f, dot(normalize(Vel), normalize(e.m_Vel)));
			float spatial = exp(-(d*d) / (2.0f * 64.0f * 64.0f));
			total += e.m_Factor * spatial * (0.5f + 0.5f * align);
		}
	}
	return total;
}

void CTasBot::RunNeuralForwardPass() {
	CCharacter *pLocalChar = GameClient()->m_PredictedWorld.GetCharacterById(GameClient()->m_Snap.m_LocalClientId);
	if (!pLocalChar) return;

	// Phase 44: Curiosity Entropy Injection (Anti-Loop)
	static vec2 LastPosForEntropy = {0,0};
	static int LoopCounter = 0;
	if (distance(pLocalChar->m_Pos, LastPosForEntropy) < 0.1f) LoopCounter++;
	else LoopCounter = 0;
	LastPosForEntropy = pLocalChar->m_Pos;

	// Layer 1: Forward Pass (ReLU)
	float h1[8] = {0};
	for (int i = 0; i < 8; i++) {
		for (int j = 0; j < 5; j++) h1[i] += m_NeuralInput[j] * Synapse::W1[i][j];
		// Entropy Injection if stuck
		if (LoopCounter > 500) h1[i] += (rand() % 100) / 50.0f - 1.0f; 
		h1[i] = std::max(0.0f, h1[i] + Synapse::B1[i]);
	}
	
	// Layer 2: Output Pass (Tanh-ish approx)
	for (int i = 0; i < 2; i++) {
		m_NeuralOutput[i] = 0.0f;
		for (int j = 0; j < 8; j++) m_NeuralOutput[i] += h1[j] * Synapse::W2[i][j];
		m_NeuralOutput[i] += Synapse::B2[i];
	}
}

void CTasBot::ExportNeuralState(float Reward) {
	if (!g_Config.m_TcBotLogging) return;
	
	CCharacter *pLocalChar = GameClient()->m_PredictedWorld.GetCharacterById(GameClient()->m_Snap.m_LocalClientId);
	if (!pLocalChar) return;

	char aBuf[512];
	str_format(aBuf, sizeof(aBuf), 
		"{\"s\": [%.1f, %.1f, %.2f, %.2f, %.2f], \"a\": [%d, %d], \"r\": %.3f}", 
		pLocalChar->m_Pos.x, pLocalChar->m_Pos.y, 
		pLocalChar->m_Core.m_Vel.x, pLocalChar->m_Core.m_Vel.y,
		m_LastDistToGoal, 
		m_LastInput.m_Direction, m_LastInput.m_Jump,
		Reward);

	IOHANDLE File = io_open("swarm_experience.jsonl", IOFLAG_APPEND);
	if(File) {
		io_write(File, aBuf, str_length(aBuf));
		io_write(File, "\n", 1);
		io_close(File);
	}
}

void CTasBot::SaveTasRun(const char *pFileName)
{
	if(!m_HasMasterRun) return;

	char aBuf[IO_MAX_PATH_LENGTH];
	str_format(aBuf, sizeof(aBuf), "tas_runs/%s.tas", pFileName);
	IOHANDLE File = Storage()->OpenFile(aBuf, IOFLAG_WRITE, IStorage::TYPE_SAVE);
	if(!File) return;

	int Version = 1;
	io_write(File, &Version, sizeof(int));
	int Count = (int)m_MasterRun.size();
	io_write(File, &Count, sizeof(int));

	io_write(File, m_MasterRun.data(), sizeof(CNetObj_PlayerInput) * Count);
	io_write(File, m_MasterRunPositions.data(), sizeof(vec2) * m_MasterRunPositions.size());

	io_close(File);
	dbg_msg("tas_bot", "Saved TAS run to %s", aBuf);
}

void CTasBot::LoadTasRun(const char *pFileName)
{
	char aBuf[IO_MAX_PATH_LENGTH];
	str_format(aBuf, sizeof(aBuf), "tas_runs/%s.tas", pFileName);
	IOHANDLE File = Storage()->OpenFile(aBuf, IOFLAG_READ, IStorage::TYPE_ALL);
	if(!File) return;

	int Version;
	if (io_read(File, &Version, sizeof(int)) != sizeof(int) || Version != 1)
	{
		io_close(File);
		return;
	}

	int Count;
	io_read(File, &Count, sizeof(int));

	m_MasterRun.resize(Count);
	m_MasterRunPositions.resize(Count);
	io_read(File, m_MasterRun.data(), sizeof(CNetObj_PlayerInput) * Count);
	io_read(File, m_MasterRunPositions.data(), sizeof(vec2) * Count);

	if(m_MasterRun.size() != m_MasterRunPositions.size())
	{
		m_MasterRun.clear();
		m_MasterRunPositions.clear();
		m_HasMasterRun = false;
		io_close(File);
		return;
	}

	io_close(File);
	m_HasMasterRun = true;
	dbg_msg("tas_bot", "Loaded TAS run from %s", aBuf);
}

void CTasBot::ResetInception()
{
	CCharacter *pLocalChar = GameClient()->m_PredictedWorld.GetCharacterById(GameClient()->m_Snap.m_LocalClientId);
	if (pLocalChar)
	{
		m_SimulatedCore = pLocalChar->m_Core;
		m_InceptionResetPos = pLocalChar->m_Pos;
	}
	else
	{
		m_SimulatedCore.Reset();
		m_InceptionResetPos = vec2(0, 0);
	}
	
	m_SimulatedCore.m_Id = 0; 
	m_SimulatedCore.SetCoreWorld(&m_SimulatedWorld, GameClient()->Collision(), &GameClient()->m_Teams);
	m_SimulatedWorld.m_apCharacters[0] = &m_SimulatedCore;

	m_StrategyTicks = 0;
	m_ActionHoldFrames = 0;
	m_TotalTicksSimulated = 0;
	m_LastAITick = -1;
	m_PreviousHorizontalVel = 0.0f;
	
	m_CurrentEpisodeTicks = 0;
	m_TicksSinceSpawn = 0;
	m_RecentTiles.clear();
	m_EpisodeHistory.clear();
}

void CTasBot::RunInceptionTick()
{
	if (!g_Config.m_ClTasTraining) return;
	CCollision *pCollision = GameClient()->Collision();
	if(!pCollision) return;

	if (!m_SimulatedCore.Collision()) {
		ResetInception();
		if (!m_SimulatedCore.Collision()) return;
	}

	uint64_t State = GetStateHash(m_SimulatedCore);
	m_CurrentEpisodeTicks++;
	m_TicksSinceSpawn++; 

	int CurrentTileIdx = pCollision->GetPureMapIndex(m_SimulatedCore.m_Pos);
	if (m_RecentTiles.empty() || m_RecentTiles.back() != CurrentTileIdx) {
		m_RecentTiles.push_back(CurrentTileIdx);
		if (m_RecentTiles.size() > 10) m_RecentTiles.pop_front();
	}

	int Action = 0; // Simplified for restoration
	vec2 TargetPos = m_SimulatedCore.m_Pos + GetFlowGradient(m_SimulatedCore.m_Pos) * 100.0f;
	CNetObj_PlayerInput Input = MapActionToInput(Action, m_SimulatedCore, TargetPos);
	
	m_SimulatedCore.m_Input = Input;
	m_SimulatedCore.Tick(true, true);
	m_SimulatedCore.Move();
	m_SimulatedCore.Quantize();

	uint64_t NextState = GetStateHash(m_SimulatedCore);
	m_TotalTicksSimulated++;
}

void CTasBot::ResetInception(vec2 Pos) { ResetInception(); m_SimulatedCore.m_Pos = Pos; }
