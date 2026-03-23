#include <engine/shared/config.h>
#include <engine/console.h>
#include <game/client/gameclient.h>
#include <game/client/components/tclient/tas_bot.h>
#include <game/client/components/tclient/astar_pathfinder.h>
#include <game/client/prediction/entities/character.h>
#include <game/mapitems.h>
#include <algorithm>

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
	m_LastDist = 0.0f;
	m_LastUpdatePos = vec2(0,0);
	m_StuckCounter = 0;
	for(int i = 0; i < 5; i++) m_PosHistory[i] = vec2(0,0);
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
	StopTraining();
	m_HasMasterRun = false;
	m_MasterRun.clear();
	m_MasterRunPositions.clear();
	
	// Reset world on map change
	if(m_pTasWorld)
	{
		delete m_pTasWorld;
		m_pTasWorld = nullptr;
	}

	m_CurrentPath.clear();
	m_StateHistory.clear();
	
	// Try to auto-load run for this map (check Map() validity)
	if(GameClient()->Map())
		LoadTasRun(GameClient()->Map()->BaseName());
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
					dbg_msg("tas_bot", "AI is STUCK. Implementing SUICIDE (kill) to restart training.");
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

uint64_t CTasBot::GetStateHash(const CCharacterCore& Core, vec2 DynamicWaypoint)
{
	// 1. Target Angle (3 bits / 8 octants)
	vec2 TargetDir = DynamicWaypoint - Core.m_Pos;
	float Angle = atan2(TargetDir.y, TargetDir.x);
	if(Angle < 0) Angle += 2.0f * pi;
	uint64_t TargetOctant = (uint64_t)(floor((Angle + pi/8.0f) / (pi/4.0f))) % 8;

	// 2. Velocity Vector (4 bits / 9 states)
	uint64_t VelState = 0;
	float VelLen = length(Core.m_Vel);
	if(VelLen > 1.0f)
	{
		float VelAngle = atan2(Core.m_Vel.y, Core.m_Vel.x);
		if(VelAngle < 0) VelAngle += 2.0f * pi;
		VelState = 1 + ((uint64_t)(floor((VelAngle + pi/8.0f) / (pi/4.0f))) % 8);
	}

	// 3. Raycast Sensors (Whiskers) (8 bits / 4 directions)
	uint64_t WhiskerMask = 0;
	vec2 Dirs[] = { vec2(0,-1), vec2(0,1), vec2(-1,0), vec2(1,0) }; // UP, DOWN, LEFT, RIGHT
	for(int i = 0; i < 4; i++)
	{
		uint64_t Type = 0; // Empty
		for(int d = 1; d <= 4; d++)
		{
			vec2 CheckPos = Core.m_Pos + Dirs[i] * (float)(d * 32);
			int Index = GameClient()->Collision()->GetPureMapIndex(CheckPos);
			int Tile = GameClient()->Collision()->GetTileIndex(Index);
			int Front = GameClient()->Collision()->GetFrontTileIndex(Index);
			
			bool Danger = (Tile == TILE_DEATH || Front == TILE_DEATH || Tile == TILE_FREEZE || Front == TILE_FREEZE);
			bool Solid = GameClient()->Collision()->CheckPoint(CheckPos);
			bool Hookable = (Tile == TILE_NOHOOK || Front == TILE_NOHOOK) ? false : Solid;

			if(Danger) { Type = 3; break; }
			if(Solid) { Type = Hookable ? 1 : 2; break; }
		}
		WhiskerMask |= (Type << (i * 2));
	}

	// 4. Hook Status (1 bit)
	uint64_t HookBit = (Core.m_HookState == 5) ? 1 : 0; // 5 is HOOK_GRABBED in many DDNet versions

	return (TargetOctant) | (VelState << 3) | (WhiskerMask << 7) | (HookBit << 15);
}

CNetObj_PlayerInput CTasBot::MapActionToInput(int Action, const CCharacterCore& Core, vec2 DynamicWaypoint)
{
	CNetObj_PlayerInput Inp = {0};
	
	// Default target: Waypoint
	vec2 TargetDir = DynamicWaypoint - Core.m_Pos;
	Inp.m_TargetX = (int)TargetDir.x;
	Inp.m_TargetY = (int)TargetDir.y;

	switch(Action)
	{
		case 0: Inp.m_Direction = -1; break; // Move Left
		case 1: Inp.m_Direction = 1; break;  // Move Right
		case 2: Inp.m_Jump = 1; break;       // Jump
		case 3: // Direct Hook (Waypoint)
		{
			Inp.m_Hook = 1;
			if(Inp.m_TargetX == 0 && Inp.m_TargetY == 0)
				Inp.m_TargetY = -1; // Force non-zero
			break;
		}
		case 4: // Action 4: Ceiling/Swing Hook
		{
			// Scan for nearest solid tile above or upper-diagonal
			vec2 BestCeiling(0, -96);
			bool Found = false;
			for(int ay = -1; ay >= -5; ay--) // Scan UP
			{
				for(int ax = -2; ax <= 2; ax++) // Lateral spread
				{
					vec2 CheckPos = Core.m_Pos + vec2(ax * 32.0f, ay * 32.0f);
					if(GameClient()->Collision()->CheckPoint(CheckPos))
					{
						BestCeiling = vec2(ax * 32.0f, ay * 32.0f);
						Found = true;
						goto found_ceiling;
					}
				}
			}
			found_ceiling:
			Inp.m_TargetX = (int)BestCeiling.x;
			Inp.m_TargetY = (int)BestCeiling.y;
			if(Inp.m_TargetX == 0 && Inp.m_TargetY == 0)
				Inp.m_TargetY = -1; // Safety
			Inp.m_Hook = 1;
			break;
		}
		case 5: Inp.m_Hook = 0; break; // Release Hook
		case 6: Inp.m_Direction = 1; Inp.m_Jump = 1; break; // Right + Jump
		case 7: Inp.m_Direction = -1; Inp.m_Jump = 1; break; // Left + Jump
	}
	return Inp;
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
		uint32_t VecSize = 8; // Fixed size for std::array<float, 8>
		io_write(File, &VecSize, sizeof(VecSize));
		io_write(File, Values.data(), VecSize * sizeof(float));
	}
	io_close(File);
	dbg_msg("tas_bot", "Memory saved: %u states.", Size);
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
		
		std::array<float, 8> Values;
		Values.fill(0.0f);
		
		if(VecSize == 8)
		{
			io_read(File, Values.data(), 8 * sizeof(float));
		}
		else
		{
			// Handle legacy data or mixed sizes if necessary
			std::vector<float> Legacy(VecSize);
			io_read(File, Legacy.data(), VecSize * sizeof(float));
			for(uint32_t j = 0; j < std::min(VecSize, 8u); j++)
				Values[j] = Legacy[j];
		}
		m_QTable[Hash] = Values;
	}
	io_close(File);
	dbg_msg("tas_bot", "Memory loaded: %u states.", Size);
}

void CTasBot::UpdateQValue(uint64_t State, int Action, float Reward, uint64_t NextState)
{
	if(Action < 0 || Action >= 8) return;
	
	// std::array is fixed size, no resize needed. operator[] value-initializes the array (zeros it).

	float MaxNextQ = -1e18f;
	for(int i = 0; i < 8; ++i) if(m_QTable[NextState][i] > MaxNextQ) MaxNextQ = m_QTable[NextState][i];

	// Q-Learning Bellman Equation
	float Alpha = 0.1f;
	float Gamma = 0.9f;
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
	if(!m_IsTraining) return {0};

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

	vec2 DynamicWaypoint = GetDynamicWaypoint(CurrentCore.m_Pos);

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
				UpdateQValue(m_LastStateHash, m_CurrentAction, -2000.0f, GetStateHash(CurrentCore, DynamicWaypoint));
				m_BannedAction = m_CurrentAction;
				m_BannedActionTicks = 25; 
				m_ActionHoldFrames = 0;
			}
		}

		if(m_ActionHoldFrames > 0)
		{
			m_LastInput = MapActionToInput(m_CurrentAction, CurrentCore, DynamicWaypoint);
			if(m_CurrentAction == 2 && m_ActionHoldElapsed > 1) m_LastInput.m_Jump = 0;
			return m_LastInput;
		}
	}

	// 4. NEW DECISION ENGINE (50Hz)
	m_ActionHoldElapsed = 0;
	uint64_t State = GetStateHash(CurrentCore, DynamicWaypoint);

	int Action = -1;
	bool IsRandom = false;
	if((float)(rand() % 1000) / 1000.0f < m_Epsilon)
	{
		IsRandom = true;
		do {
			Action = rand() % 8;
		} while(Action == m_BannedAction && m_BannedActionTicks > 0 && (rand() % 10) < 9); 
	}
	else
	{
		float MaxQ = -1e18f;
		for(int i = 0; i < 8; i++)
		{
			if(i == m_BannedAction && m_BannedActionTicks > 0) continue;
			if(m_QTable[State][i] > MaxQ)
			{
				MaxQ = m_QTable[State][i];
				Action = i;
			}
		}
		if(Action == -1) Action = rand() % 8;
	}

	m_CurrentAction = Action;
	
	// Forensic Trace
	int Restrictions = GameClient()->Collision()->GetMoveRestrictions(CurrentCore.m_Pos);
	dbg_msg("tas_bot", "RECAP 50Hz: State:%llu Act:%d (Rand:%d) Eps:%.3f PosX:%.1f", 
		State, Action, IsRandom, (float)m_Epsilon, CurrentCore.m_Pos.x);
	
	// Adaptive Hold Duration
	m_ActionHoldFrames = 5 + (int)(15.0f * (1.0f - m_Epsilon));
	m_ActionHoldElapsed = 0;
	m_LastStateHash = State;
	
	m_LastInput = MapActionToInput(m_CurrentAction, CurrentCore, DynamicWaypoint);
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
				
				// Generate a specialized strategy for this candidate
				int Strategy = c % 8; 
				for(int t = 0; t < m_CandidateTicks; t++)
				{
					CNetObj_PlayerInput CandInput = {0};
					// 0: Right, 1: Left, 2: Right+Hook, 3: Left+Hook, 4: Right+Jump, 5: Left+Jump, 6: Hook-Only, 7: Random
					if(Strategy == 0) CandInput.m_Direction = 1;
					else if(Strategy == 1) CandInput.m_Direction = -1;
					else if(Strategy == 2) { CandInput.m_Direction = 1; CandInput.m_Hook = 1; CandInput.m_TargetY = -100; }
					else if(Strategy == 3) { CandInput.m_Direction = -1; CandInput.m_Hook = 1; CandInput.m_TargetY = -100; }
					else if(Strategy == 4) { CandInput.m_Direction = 1; CandInput.m_Jump = 1; }
					else if(Strategy == 5) { CandInput.m_Direction = -1; CandInput.m_Jump = 1; }
					else if(Strategy == 6) { CandInput.m_Hook = 1; CandInput.m_TargetY = -100; }
					else { CandInput.m_Direction = (rand()%3)-1; CandInput.m_Jump = rand()%2; CandInput.m_Hook = rand()%2; }
					
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
	// === SAFETY GATE ===
	if(!GameClient()->Collision() || GameClient()->m_Snap.m_LocalClientId < 0)
		return;

	// === MUTUAL EXCLUSION ===
	if(g_Config.m_ClTasTraining && g_Config.m_ClTasPlayback)
	{
		g_Config.m_ClTasPlayback = 0; 
		dbg_msg("tas_bot", "CONFLICT: Disabling Playback Mode because Training Mode is active.");
	}

	if(!g_Config.m_ClTasTraining)
	{
		StopTraining();
		return;
	}

	if(!m_IsTraining) StartTraining();

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

		// Run multiple simulation ticks per render frame with a 2ms budget
		int64_t StartTime = time_get();
		int64_t Budget = time_freq() / 500; // 2ms budget
		for (int i = 0; i < g_Config.m_ClTasBotInceptionSpeed; ++i)
		{
			if(i % 5 == 0 && (time_get() - StartTime) > Budget)
				break;
			RunInceptionTick();
		}
	}
	else if (m_InceptionActive)
	{
		m_InceptionActive = false;
		dbg_msg("tas_bot", "INCEPTION MODE DISABLED: Waking up.");
	}

	// Phase 7: GPS FIX - Continuous Dynamic Recalculation
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
	CCharacter *pLocalChar = GameClient()->m_PredictedWorld.GetCharacterById(GameClient()->m_Snap.m_LocalClientId);
	if(pLocalChar)
	{
		vec2 DynamicWaypoint = GetDynamicWaypoint(pLocalChar->GetCore().m_Pos);
		uint64_t CurrentState = GetStateHash(pLocalChar->GetCore(), DynamicWaypoint);
		
		// Compute Reward for the frame
		float Reward = 0.0f;
		
		// 1. Progress Reward
		if(!m_AStarPath.empty())
		{
			float DistToGoal = distance(pLocalChar->GetCore().m_Pos, m_AStarPath.back());
			if(m_LastDist == 0.0f) m_LastDist = DistToGoal;
			Reward += (m_LastDist - DistToGoal) * 1.0f; // Weight increased
			m_LastDist = DistToGoal;
			
			if(distance(pLocalChar->GetCore().m_Pos, DynamicWaypoint) < 96.0f)
				Reward += 5.0f; // Milestone bonus
		}
		
		Reward += length(pLocalChar->GetCore().m_Vel) * 0.05f;
		
		// 3. Tarzan Reward (Momentum Shaping)
		float CurrentHVel = abs(pLocalChar->GetCore().m_Vel.x);
		if(m_CurrentAction == 4 || m_CurrentAction == 5)
		{
			// If we are swinging or releasing, reward building horizontal momentum towards waypoint side
			vec2 ToWaypoint = DynamicWaypoint - pLocalChar->GetCore().m_Pos;
			bool CorrectDir = (ToWaypoint.x > 0 && pLocalChar->GetCore().m_Vel.x > 5.0f) || 
			                  (ToWaypoint.x < 0 && pLocalChar->GetCore().m_Vel.x < -5.0f);
			
			if(CorrectDir && CurrentHVel > m_PreviousHorizontalVel + 1.0f)
			{
				Reward += 50.0f; // MASSIVE TARZAN BONUS
				dbg_msg("tas_bot", "TARZAN! Momentum gain: %.2f -> %.2f", m_PreviousHorizontalVel, CurrentHVel);
			}
		}
		m_PreviousHorizontalVel = CurrentHVel;

		// 4. Floor Penalty
		if(GameClient()->Collision()->CheckPoint(pLocalChar->GetCore().m_Pos + vec2(0, 16)))
		{
			Reward -= 1.0f;
		}

		// 5. Perform the actual Q-Learning update
		UpdateQValue(m_LastStateHash, m_CurrentAction, Reward, CurrentState);
		m_LastStateHash = CurrentState;
		
		// 3. Auto-Kill / Stuck Detection
		bool ForceReset = false;
		if(distance(pLocalChar->GetCore().m_Pos, m_LastUpdatePos) < 5.0f)
			m_StuckCounter++;
		else
		{
			m_StuckCounter = 0;
			m_LastUpdatePos = pLocalChar->GetCore().m_Pos;
		}

		if(m_pTasWorld && (m_pTasWorld->IsDeadly() || m_pTasWorld->IsFrozen()))
		{
			dbg_msg("tas_bot", "FREEZE/DEATH: Auto-Resetting.");
			Reward -= 1000.0f;
			ForceReset = true;
		}

		if(ForceReset || (g_Config.m_TcTasBotAutoKill && m_StuckCounter > 50 * 5))
		{
			if(!ForceReset) dbg_msg("tas_bot", "STUCK (5s): Auto-Resetting.");
			GameClient()->Console()->ExecuteLine("kill", IConsole::CLIENT_ID_GAME);
			m_StuckCounter = 0;
			m_LastDist = 0.0f;
			m_LastUpdatePos = vec2(0,0);
			m_ActionHoldFrames = 0; 
			m_Epsilon *= 0.999f; 
			
			// Save memory every reset for safety
			static int s_ResetCounter = 0;
			if(++s_ResetCounter % 10 == 0) SaveMemory();
		}
	}
	else
	{
		// Auto-Respawn: If dead, try to kill/jump to come back
		static int s_RespawnPulse = 0;
		if(CurrentTick % 100 == 0) // Every 2 seconds
		{
			dbg_msg("tas_bot", "Character missing: Sending respawn pulse (kill/jump).");
			GameClient()->Console()->ExecuteLine("kill", IConsole::CLIENT_ID_GAME);
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

	// Phase 8: Visualize the Inception Ghost
	if (g_Config.m_ClTasBotInception)
	{
		RenderInceptionGhost();
	}

	// Draw A* Path
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
			Graphics()->LinesEnd();
		}
	}
}

vec2 CTasBot::FindBestHookTarget(vec2 BotPos, vec2 TargetDir, float Radius)
{
	CCollision *pCollision = GameClient()->Collision();
	if(!pCollision) return vec2(0, 0);

	int w = pCollision->GetWidth();
	int h = pCollision->GetHeight();
	int x0 = std::max(0, (int)(BotPos.x - Radius) / 32);
	int x1 = std::min(w - 1, (int)(BotPos.x + Radius) / 32);
	int y0 = std::max(0, (int)(BotPos.y - Radius) / 32);
	int y1 = std::min(h - 1, (int)(BotPos.y + Radius) / 32);

	vec2 BestTarget(0, 0);
	float BestValue = -1e9;

	for(int y = y0; y <= y1; y++)
	{
		for(int x = x0; x <= x1; x++)
		{
			if(pCollision->IsSolid(x * 32, y * 32))
			{
				vec2 TilePos(x * 32 + 16, y * 32 + 16);
				vec2 ToTile = normalize(TilePos - BotPos);
				float Dist = distance(BotPos, TilePos);
				
				// DIRECTIONAL FILTER: Only hook towards the target waypoint
				if(Dist < Radius && Dist > 64.0f && dot(ToTile, TargetDir) > 0.3f)
				{
					// Value: Highest point in the target direction
					float Val = (BotPos.y - TilePos.y) + dot(ToTile, TargetDir) * 100.0f;
					if(Val > BestValue)
					{
						BestValue = Val;
						BestTarget = TilePos;
					}
				}
			}
		}
	}
	return BestTarget;
}

void CTasBot::OnBeforeInput(CNetObj_PlayerInput *pInput)
{
	// === SAFETY GATE: Don't run ANY input logic until game state is fully ready ===
	if(!GameClient()->Collision() || GameClient()->m_Snap.m_LocalClientId < 0)
		return;

	// === SAFETY GATE: Resolve Logic Gate Paradox (V6) ===
	// Allow processing if either Playback OR Training is active
	if(!g_Config.m_ClTasPlayback && !m_IsTraining)
	{
		m_PlaybackTick = 0;
		return;
	}

	// Phase 8: Inception Bypass
	if (g_Config.m_ClTasBotInception && m_InceptionActive)
	{
		// Real character stays still on the server while we dream locally
		mem_zero(pInput, sizeof(CNetObj_PlayerInput));
		pInput->m_TargetY = -1;
		return;
	}

	// Execution logic for true live-server replay with Ping Compensation
	if(m_IsTraining)
	{
		// Avoid Freeze Synergy: Disable normal avoid freeze while AI is training (Modular)
		if(g_Config.m_TcBotSynergy && m_PrevAvoidFreeze == -1)
		{
			m_PrevAvoidFreeze = g_Config.m_TcAvoidFreeze;
			g_Config.m_TcAvoidFreeze = 0;
			dbg_msg("tas_bot", "SYNERGY: Disabled global Avoid-Freeze for training.");
		}

		// Use the REAL-TIME BRAIN to move the character
		CCharacter *pLocalChar = GameClient()->m_PredictedWorld.GetCharacterById(GameClient()->m_Snap.m_LocalClientId);
		
		// Death detection: Character existed but now is gone
		static bool s_HadChar = false;
		if(s_HadChar && !pLocalChar)
		{
			// JUST DIED! Save last known position
			SDeathInfo Death;
			Death.m_Pos = m_LastStuckPos;
			Death.m_Tick = Client()->GameTick(g_Config.m_ClDummy);
			m_DeathMemory.push_back(Death);
			if(m_DeathMemory.size() > 50) m_DeathMemory.erase(m_DeathMemory.begin());
			dbg_msg("tas_bot", "LEARNED: Death detected. Added to penalty memory.");
			
			// Auto-Kill logic: If stuck for too long, reset to try again
			if(g_Config.m_TcBotAutoReset) // m_Alive is not a member of CTasBot, using pLocalChar != nullptr instead
			{
				m_PlaybackTick = 0; // Restart internal state
			}
		}
		s_HadChar = (pLocalChar != nullptr);

		if(pLocalChar)
		{
			m_LastStuckPos = pLocalChar->GetCore().m_Pos;
			
			// Auto-Kill logic: If stuck for too long, reset to try again
			if(g_Config.m_TcBotAutoReset && pLocalChar)
			{
				if(length(pLocalChar->GetCore().m_Vel) < 0.1f)
					m_StuckTicks++;
				else
					m_StuckTicks = 0;

				if(m_StuckTicks > 300) // 6 seconds
				{
					dbg_msg("tas_bot", "STUCK detected! Sending KILL to reset.");
					GameClient()->SendKill();
					m_StuckTicks = 0;
				}
			}

			*pInput = GetBestLiveInput(pLocalChar->GetCore());
		}
		return;
	}
	else if(m_PrevAvoidFreeze != -1)
	{
		// Restore normal avoid freeze when training stops (Modular)
		if(g_Config.m_TcBotSynergy)
		{
			g_Config.m_TcAvoidFreeze = m_PrevAvoidFreeze;
			m_PrevAvoidFreeze = -1;
			dbg_msg("tas_bot", "SYNERGY: Restored global Avoid-Freeze.");
		}
	}

	if(m_HasMasterRun && !m_IsTraining) 
	{
		if(m_PlaybackTick >= 0 && m_PlaybackTick < (int)m_MasterRun.size() && m_PlaybackTick < (int)m_MasterRunPositions.size())
		{
			CCharacter *pLocalChar = GameClient()->m_PredictedWorld.GetCharacterById(GameClient()->m_Snap.m_LocalClientId);
			if(!pLocalChar) {
				// No debug message here to avoid spamming the console 50 times/sec
				return;
			}
			
			vec2 CurrentPos = pLocalChar->GetCore().m_Pos;
			vec2 ExpectedPos = m_MasterRunPositions[m_PlaybackTick];
			
			// Calculate distance between where we are and where the TAS says we should be
			float Dist = distance(CurrentPos, ExpectedPos);
			
			// Tolerance check (30.0f is roughly 1 tile radius)
			if(Dist < 30.0f)
			{
				// In sync! Execute the planned input for this tick
				*pInput = m_MasterRun[m_PlaybackTick];
				m_PlaybackTick++;
			}
			else
			{
				// Out of sync! (Lag spike or misprediction from server)
				// Do NOT increment the playback tick. Wait for the character to physically arrive.
				// Keep holding the previous input to maintain momentum.
				*pInput = m_MasterRun[std::max(0, m_PlaybackTick - 1)];
				
				// Optional: We could trigger CAvoidFreeze here for extreme safety
			}
		}
		else
		{
			g_Config.m_ClTasPlayback = 0;
			dbg_msg("tas_bot", "Playback finished.");
		}
	}
}

void CTasBot::SaveTasRun(const char *pFileName)
{
	if(!m_HasMasterRun) return;

	char aBuf[IO_MAX_PATH_LENGTH];
	str_format(aBuf, sizeof(aBuf), "tas_runs/%s.tas", pFileName);
	IOHANDLE File = Storage()->OpenFile(aBuf, IOFLAG_WRITE, IStorage::TYPE_SAVE);
	if(!File) return;

	// Write header (version, count)
	int Version = 1;
	io_write(File, &Version, sizeof(int));
	int Count = (int)m_MasterRun.size();
	io_write(File, &Count, sizeof(int));

	// Write inputs and positions
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

	// Final verification: ensure both contain the same amount of data
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
	
	m_SimulatedCore.m_Id = 0; // Local ID for sim
	m_SimulatedCore.SetCoreWorld(&m_SimulatedWorld, GameClient()->Collision(), &GameClient()->m_Teams);
	m_SimulatedWorld.m_apCharacters[0] = &m_SimulatedCore;

	m_StrategyTicks = 0;
	m_ActionHoldFrames = 0;
	m_TotalTicksSimulated = 0;
	m_LastAITick = -1;
	m_PreviousHorizontalVel = 0.0f;
}

void CTasBot::RunInceptionTick()
{
	if (!g_Config.m_ClTasTraining) return;

	// 1. Get current state
	vec2 DynamicWaypoint = GetDynamicWaypoint(m_SimulatedCore.m_Pos);
	uint64_t State = GetStateHash(m_SimulatedCore, DynamicWaypoint);

	// 2. Choose Action (Epsilon-Greedy)
	int Action;
	if ((rand() % 100) < (int)(m_Epsilon * 100))
		Action = rand() % 8;
	else
	{
		float MaxQ = -1e9;
		Action = 0;
		for (int a = 0; a < 8; ++a) {
			if (m_QTable[State][a] > MaxQ) {
				MaxQ = m_QTable[State][a];
				Action = a;
			}
		}
	}

	// 3. Map Action to Input
	CNetObj_PlayerInput Input = MapActionToInput(Action, m_SimulatedCore, DynamicWaypoint);
	
	// FIX: Ensure hook has target
	if (Input.m_Hook)
	{
		if (Input.m_TargetX == 0 && Input.m_TargetY == 0)
			Input.m_TargetY = -1;
	}

	// 4. Step Physics (Simulated)
	m_SimulatedCore.m_Input = Input;
	m_SimulatedCore.Tick(true, true);
	m_SimulatedCore.Move();
	m_SimulatedCore.Quantize();

	// 5. Evaluate Reward
	float Reward = -1.0f; // Time penalty
	
	// Distance reward
	float Dist = distance(m_SimulatedCore.m_Pos, DynamicWaypoint);
	if (Dist < m_LastDist) Reward += 10.0f; // Increased for faster convergence
	m_LastDist = Dist;

	// Tarzan Reward (Momentum bonus)
	if (absolute(m_SimulatedCore.m_Vel.x) > absolute(m_PreviousHorizontalVel))
		Reward += 5.0f;
	m_PreviousHorizontalVel = m_SimulatedCore.m_Vel.x;

	// Failure Check (Inception Reset)
	bool Failed = false;
	int Tile = GameClient()->Collision()->GetTile(round_to_int(m_SimulatedCore.m_Pos.x), round_to_int(m_SimulatedCore.m_Pos.y));
	int Front = GameClient()->Collision()->GetFrontTile(round_to_int(m_SimulatedCore.m_Pos.x), round_to_int(m_SimulatedCore.m_Pos.y));

	if (Tile == TILE_DEATH || Front == TILE_DEATH)
		Failed = true;
	if (m_SimulatedCore.m_IsInFreeze)
		Failed = true;
	
	if (Failed)
	{
		Reward = -5000.0f; // Heavier penalty
		UpdateQValue(State, Action, Reward, State);
		ResetInception();
		return;
	}

	// Success Check
	if (Dist < 48.0f)
	{
		Reward = 2000.0f;
		UpdateQValue(State, Action, Reward, State);
		// Don't reset, just keep going to next waypoint
	}

	// 6. Update Q-Table
	uint64_t NextState = GetStateHash(m_SimulatedCore, DynamicWaypoint);
	UpdateQValue(State, Action, Reward, NextState);

	m_TotalTicksSimulated++;
	
	// Epsilon Decay (Episode-based)
	if (m_TotalTicksSimulated % 10000 == 0 && m_Epsilon > 0.05f)
		m_Epsilon -= 0.01f;
}

void CTasBot::RenderInceptionGhost()
{
	if (!m_InceptionActive) return;

	// Use the local player's render info but modify for ghosting
	CCharacter *pLocalChar = GameClient()->m_PredictedWorld.GetCharacterById(GameClient()->m_Snap.m_LocalClientId);
	if (!pLocalChar) return;

	CTeeRenderInfo RenderInfo = GameClient()->m_aClients[GameClient()->m_Snap.m_LocalClientId].m_RenderInfo;
	
	// Create a dummy character object for the renderer
	CNetObj_Character GhostChar;
	mem_zero(&GhostChar, sizeof(GhostChar));
	
	CNetObj_CharacterCore GhostCore;
	m_SimulatedCore.Write(&GhostCore);
	
	// Manually sync core fields (Safe bypass for flat protocol structs)
	GhostChar.m_X = GhostCore.m_X;
	GhostChar.m_Y = GhostCore.m_Y;
	GhostChar.m_VelX = GhostCore.m_VelX;
	GhostChar.m_VelY = GhostCore.m_VelY;
	GhostChar.m_Angle = GhostCore.m_Angle;
	GhostChar.m_Direction = GhostCore.m_Direction;
	GhostChar.m_Jumped = GhostCore.m_Jumped;
	GhostChar.m_HookState = GhostCore.m_HookState;
	GhostChar.m_HookTick = GhostCore.m_HookTick;
	GhostChar.m_HookX = GhostCore.m_HookX;
	GhostChar.m_HookY = GhostCore.m_HookY;
	
	// Visual adjustments to distinguish the "Dream Tee"
	// We use ClientId = -2 to trigger the Ghost Alpha in players.cpp
	GameClient()->m_Players.RenderPlayer(
		&GhostChar, &GhostChar, 
		&RenderInfo, 
		-2, // Ghost ID
		0.0f
	);

	// Optional: Render a line to the current target
	vec2 DynamicWaypoint = GetDynamicWaypoint(m_SimulatedCore.m_Pos);
	Graphics()->TextureClear();
	Graphics()->LinesBegin();
	Graphics()->SetColor(0.0f, 1.0f, 0.0f, 0.3f);
	IGraphics::CLineItem Line(m_SimulatedCore.m_Pos.x, m_SimulatedCore.m_Pos.y, DynamicWaypoint.x, DynamicWaypoint.y);
	Graphics()->LinesDraw(&Line, 1);
	Graphics()->LinesEnd();
}
