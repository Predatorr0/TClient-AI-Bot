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
	m_IsTraining = false;
	m_HasMasterRun = false;
	m_pTasWorld = nullptr;
	m_Epsilon = 0.5f; // Initial high exploration
	m_ConsecutiveDeadEnds = 0;
	m_StrategyTicks = 0;
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
}

CTasBot::~CTasBot()
{
	if(m_pTasWorld)
	{
		delete m_pTasWorld;
		m_pTasWorld = nullptr;
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
		dbg_msg("tas_bot", "Training Aborted. Total simulated ticks: %d, Attempts: %d", m_TotalTicksSimulated, m_Attempts);
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
		
		if(NearWall) Score -= 100.0f;
		
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
	if(!m_pTasWorld && GameClient()->Collision()) 
		m_pTasWorld = new CTasWorld(GameClient()->Collision());
	
	if(!m_pTasWorld) return {0};
	
	std::vector<CCandidatePath> Candidates;
	
	// SENSORS: Check surroundings for hookable walls
	vec2 HookTarget = vec2(0,0);
	if(g_Config.m_TcBotDDRaceAware)
		HookTarget = FindBestHookTarget(CurrentCore.m_Pos, 400.0f);

	// 1. EXPLORATION (Epsilon-Greedy): Occasionally replace most candidates with random mutations
	bool UseElite = (((rand() % 1000) / 1000.0f) > m_Epsilon);

	for(int i = 0; i < m_NumCandidates; i++)
	{
		CCandidatePath Cand;
		
		// STRATEGY ASSIGNMENT
		// i=0: Random Explorer (Pure Noise)
		// i=1: Elite Follower (if exists)
		// i=2-10: Strategic Specialists
		int Strategy = i % 12;

		for(int t = 0; t < m_CandidateTicks; t++)
		{
			CNetObj_PlayerInput Inp = {0};
			Inp.m_TargetY = -100;
			Inp.m_TargetX = (rand() % 400) - 200;

			// Random Exploration Strategy
			if(!UseElite || Strategy == 0)
			{
				Inp.m_Direction = (rand() % 3) - 1;
				Inp.m_Jump = (rand() % 2);
				Inp.m_Hook = (rand() % 2);
			}
			else if(Strategy == 6) { Inp.m_Hook = 1; }
			else if(Strategy == 7) { Inp.m_Jump = 1; }
			else if(Strategy == 8) { Inp.m_Direction = (rand()%3)-1; Inp.m_Hook = (rand()%2); }
			else if(Strategy == 9) { // Smart Hook toward A* Path
				if(!m_AStarPath.empty()) {
					vec2 Target = m_AStarPath[std::min((int)m_AStarPath.size()-1, 5)];
					Inp.m_TargetX = (int)(Target.x - CurrentCore.m_Pos.x);
					Inp.m_TargetY = (int)(Target.y - CurrentCore.m_Pos.y);
					Inp.m_Hook = 1;
				}
			}
			else if(Strategy == 10) { Inp.m_Direction = 1; Inp.m_Hook = 1; Inp.m_Jump = 1; }
			else { Inp.m_Direction = -1; Inp.m_Hook = 1; Inp.m_Jump = 1; }

			m_pTasWorld->SimulateTick(Inp);
			Cand.m_Inputs.push_back(Inp);
			if(m_pTasWorld->IsDeadly() || m_pTasWorld->IsFrozen()) { Cand.m_IsDead = true; break; }
		}
		
		Cand.m_FinalCore = m_pTasWorld->GetCore();
		Cand.m_Score = EvaluateState(Cand, CurrentCore);
		Candidates.push_back(Cand);
	}
	
	// PERSISTENCE: If we are in the middle of a strategy, don't switch unless another is MUCH better
	if(m_StrategyTicks > 0 && m_LastStrategy != -1)
	{
		m_StrategyTicks--;
	}
	
	int BestIdx = -1;
	float BestScore = -1e9;
	for(int i = 0; i < (int)Candidates.size(); i++)
	{
		float Score = Candidates[i].m_Score;
		if(m_StrategyTicks > 0 && (i % 12) == m_LastStrategy) Score += 500.0f; // Stay with current strategy
		
		if(Score > BestScore) { BestScore = Score; BestIdx = i; }
	}

	// Save best for next tick (Genetic Evolution)
	if(BestIdx != -1)
	{
		m_LastStrategy = BestIdx % 12;
		m_StrategyTicks = 5; // Stick for 5 ticks (0.1s approx)
		m_BestCandidate = Candidates[BestIdx];
		m_LastBestInput = m_BestCandidate.m_Inputs[0];
		
		// Logging
		if (g_Config.m_TcBotLogging && (Client()->GameTick(g_Config.m_ClDummy) % 60 == 0))
		{
			dbg_msg("tas_bot", "LIVE: Score:%.1f Strategy:%d Vel:%.1f,%.1f", 
				BestScore, m_LastStrategy, CurrentCore.m_Vel.x, CurrentCore.m_Vel.y);
		}

		return m_LastBestInput;
	}

	m_LastBestInput = {0};
	return m_LastBestInput;
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
	// === SAFETY GATE: Don't run ANY AI logic until game state is fully ready ===
	if(!GameClient()->Collision() || GameClient()->m_Snap.m_LocalClientId < 0)
		return;

	// === MUTUAL EXCLUSION: Training and Playback cannot run at the same time ===
	if(g_Config.m_ClTasTraining && g_Config.m_ClTasPlayback)
	{
		g_Config.m_ClTasPlayback = 0; // Training takes priority
		m_PlaybackTick = 0;
		dbg_msg("tas_bot", "CONFLICT: Disabling Playback Mode because Training Mode is active.");
	}

	// === EPISODE RESET: Automatically kill if stuck or no path progress ===
	if(m_IsTraining)
	{
		static int s_NoProgressTicks = 0;
		CCharacter *pLocalChar = GameClient()->m_PredictedWorld.GetCharacterById(GameClient()->m_Snap.m_LocalClientId);
		
		if(pLocalChar)
		{
			// Check if we are stuck or moving too slowly for too long
			if(length(pLocalChar->GetCore().m_Vel) < 0.2f)
				s_NoProgressTicks++;
			else
				s_NoProgressTicks = 0;
				
			if(s_NoProgressTicks > 50 * 5) // 5 seconds of no movement
			{
				dbg_msg("tas_bot", "STUCK: Resetting episode (Kill).");
				GameClient()->Console()->ExecuteLine("kill", IConsole::CLIENT_ID_GAME);
				s_NoProgressTicks = 0;
				m_ConsecutiveDeadEnds++;
			}
		}
	}

	if(g_Config.m_ClTasTraining)
		StartTraining();
	else if(!g_Config.m_ClTasTraining && m_IsTraining)
		StopTraining();

	if(m_IsTraining)
	{
		// Only step training if we have a valid character to sample from
		CCharacter *pLocalChar = GameClient()->m_PredictedWorld.GetCharacterById(GameClient()->m_Snap.m_LocalClientId);
		if(pLocalChar)
			StepTrainingBatch();
	}
}

void CTasBot::OnRender()
{
	// === SAFETY GATE: Don't run ANY render/analysis logic until game state is fully ready ===
	if(!GameClient()->Collision() || GameClient()->m_Snap.m_LocalClientId < 0)
		return;

	// Auto-Analyze Map if stuck or starting
	static int s_StuckCounter = 0;
	if(m_IsTraining)
	{
		CCharacter *pLocalChar = GameClient()->m_PredictedWorld.GetCharacterById(GameClient()->m_Snap.m_LocalClientId);
		if(pLocalChar && length(pLocalChar->GetCore().m_Vel) < 0.5f) s_StuckCounter++;
		else s_StuckCounter = 0;
		
		if(s_StuckCounter > 120 || (m_AStarPath.empty() && m_IsTraining))
		{
			dbg_msg("tas_bot", "STUCK or NO PATH! Reseting training and A*...");
			m_AStarPath.clear();
			m_CurrentPath.clear();
			m_StateHistory.clear();
			StartTraining(); // This calls FindPath again
			s_StuckCounter = 0;
		}
	}
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
			Graphics()->SetColor(0, 1, 0, 1.0f);
			IGraphics::CLineItem Marker(p1.x, p1.y, p1.x, p1.y-5.0f);
			Graphics()->LinesDraw(&Marker, 1);
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

vec2 CTasBot::FindBestHookTarget(vec2 Pos, float Radius)
{
	CCollision *pCollision = GameClient()->Collision();
	if(!pCollision) return vec2(0, 0);

	int w = pCollision->GetWidth();
	int h = pCollision->GetHeight();
	int x0 = std::max(0, (int)(Pos.x - Radius) / 32);
	int x1 = std::min(w - 1, (int)(Pos.x + Radius) / 32);
	int y0 = std::max(0, (int)(Pos.y - Radius) / 32);
	int y1 = std::min(h - 1, (int)(Pos.y + Radius) / 32);

	vec2 BestTarget(0, 0);
	float BestValue = -1e9;

	for(int y = y0; y <= y1; y++)
	{
		for(int x = x0; x <= x1; x++)
		{
			if(pCollision->IsSolid(x * 32, y * 32))
			{
				vec2 TilePos(x * 32 + 16, y * 32 + 16);
				float Dist = distance(Pos, TilePos);
				if(Dist < Radius && Dist > 64.0f)
				{
					// Value: Highest point that is hookable
					float VerticalVal = Pos.y - TilePos.y; 
					if(VerticalVal > BestValue)
					{
						BestValue = VerticalVal;
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

	// Check config toggle
	if(!g_Config.m_ClTasPlayback)
	{
		m_PlaybackTick = 0;
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
