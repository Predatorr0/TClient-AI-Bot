#include "synapse_weights.h"
#ifndef GAME_CLIENT_COMPONENTS_TCLIENT_TAS_BOT_H
#define GAME_CLIENT_COMPONENTS_TCLIENT_TAS_BOT_H

#include <deque>
#include <map>
#include <vector>
#include <string>
#include <fstream>
#include <cstdint>
#include <array>
#include <queue>

#include <base/vmath.h>
#include <base/system.h>
#include <game/client/component.h>
#include <generated/protocol.h>
#include <game/gamecore.h>

#include "tas_world.h"

class CGameClient;
class CTasWorld;
class CAStarPathfinder;

class CTasBot : public CComponent
{
	friend class CTasWorld;

	CTasWorld *m_pTasWorld;
	CAStarPathfinder *m_pPathfinder;
	
	// Phase 10: Flow-Field (Global Navigation)
	int m_MapWidth;
	int m_MapHeight;
	std::vector<int> m_DistanceMap; 
	void ComputeDistanceMap();
	int GetDistanceMapValue(vec2 Pos);
	vec2 GetFlowGradient(vec2 Pos);
	
	// Path Tracking
	std::vector<vec2> m_AStarPath;
	std::vector<CNetObj_PlayerInput> m_CurrentPath;
	std::vector<CCharacterCore> m_StateHistory;

	// Inception Simulation
	CCharacterCore m_SimulatedCore;
	CWorldCore m_SimulatedWorld; 
	bool m_InceptionActive;
	vec2 m_InceptionResetPos;
	void RunInceptionTick();
	void ResetInception(vec2 Pos);
	void ResetInception(); 
	
	// Phase 20: Anti-Loop & Stability
	std::deque<int> m_RecentTiles;
	int m_TicksSinceSpawn;
	int m_StuckTicks;
	int m_LiveLastDist;
	vec2 m_LastStuckPos;
	int m_PrevAvoidFreeze;
	int m_StuckCounter;
	vec2 m_LastUpdatePos;
	
	// RL Metrics & Core
	int m_CurrentEpisodeTicks;
	float m_LastDistToGoal; // RESTORED
	int m_DreamLastDist;
	int m_StrategyTicks;
	int m_ActionHoldFrames;
	int m_ActionHoldElapsed;
	float m_PreviousHorizontalVel;
	int m_MaxSimulationDepth;
	int m_Attempts;
	int m_ConsecutiveDeadEnds;
	bool m_ForceJumpNext;
	bool m_ForceHookNext;
	int m_NumCandidates = 18;
	int m_CandidateTicks = 30;
	int m_BannedAction;
	int m_BannedActionTicks;
	int m_CurrentAction;
	uint64_t m_LastStateHash;
	CNetObj_PlayerInput m_LastInput;
	CNetObj_PlayerInput m_LastBestInput;
	vec2 m_PosHistory[5];
	int m_PosHistoryIdx;
	CCharacterCore m_StartCore;
	bool m_HyperSimulationActive;
	
	struct SDeathInfo {
		vec2 m_Pos;
		int m_Tick;
	};
	std::vector<SDeathInfo> m_DeathMemory;

	struct CTransition {
		uint64_t m_State;
		int m_Action;
		float m_Reward;
		uint64_t m_NextState;
	};
	std::vector<CTransition> m_ExperienceBuffer;
	std::vector<CTransition> m_EpisodeHistory; // RESTORED
	const int MAX_EXPERIENCE = 10000;
	std::map<uint64_t, int> m_StateVisitCount;

	void ResetEpisode();
	void StepTrainingBatch();
	void BackpropagateRewards(float FinalReward);
	void UpdateQValue(uint64_t State, int Action, float Reward, uint64_t NextState);
	uint64_t GetStateHash(const CCharacterCore& Core);
	CNetObj_PlayerInput MapActionToInput(int Action, const CCharacterCore& Core, vec2 TargetPos);
	CNetObj_PlayerInput GetBestLiveInput(const CCharacterCore& Core);
	CNetObj_PlayerInput GetNextExplorationInput(bool ForceJump, vec2 CurrentPos);
	vec2 GetDynamicWaypoint(vec2 BotPos);
	
	struct CCandidatePath {
		std::vector<CNetObj_PlayerInput> m_Inputs;
		CCharacterCore m_FinalCore;
		float m_Score;
		bool m_IsDead;
	};
	float EvaluateState(const CCandidatePath& Cand, const CCharacterCore& StartCore);
	vec2 FindBestHookTarget(vec2 BotPos, vec2 IdealDir, float Radius);

	// Phase 22: Jarvis Protocol & Trauma Map
	void DumpTelemetry(const char *Tag, vec2 Pos = vec2(0,0));
	void LoadTraumaMap();
	std::map<int, int> m_TraumaMap; 
	
	// RL Parameters
	int m_BestFinishTicks;
	int m_TotalFinishes;
	float m_Epsilon;
	int m_TotalTicksSimulated;
	
	// RL Table: State -> Action-Array[60]
	std::map<uint64_t, std::array<float, 60>> m_QTable;

	// Ghost Rendering
	void RenderInceptionGhost();

	bool m_HasMasterRun;
	std::vector<CNetObj_PlayerInput> m_MasterRun;
	std::vector<vec2> m_MasterRunPositions;


	// Phase 32: Advanced Trinity V4 (TAP & SHRF)
	
	float m_ExplorationBoost = 1.0f;
	void UpdateTapMatrix(vec2 Pos, vec2 Vel, float Trauma);
	void ExportNeuralState(float Reward);
	float GetTraumaIntensity(vec2 Pos);


	// Phase 35: Phase-Space Trauma (V5)
	struct STraumaEvent {
		vec2 m_Pos;
		vec2 m_Vel;
		float m_Factor;
	};
	std::vector<STraumaEvent> m_TapMatrix;
	float GetPhaseSpaceTrauma(vec2 Pos, vec2 Vel);


	// Phase 64: The Sovereign Centaur (Native Oracle) 🏛️
	struct STasActionSequence {
		int m_NumTicks;
		CNetObj_PlayerInput m_Inputs[100]; 
	};

	struct SOracleTrajectory {
		vec2 m_EndPos;
		vec2 m_EndVel;
		bool m_IsPhysicallyValid;
		int m_SurvivalTicks;
	};

	SOracleTrajectory VerifyTrajectoryPrior(const STasActionSequence& PriorSequence);

	// Phase 62: Latent Phase-Space Diffusion (LPSD) 🌌
	struct DiffusionBrain {
		// Python writes the hallucinated trajectory (100 ticks ahead)
		int predicted_actions[100]; 
		float predicted_target_x[100];
		float predicted_target_y[100];
		
		// C++ reports actual state for next denoising pass
		float current_pos_x, current_pos_y;
		float current_vel_x, current_vel_y;
		int current_tick_index;
		
		int read_buffer_index; 
	};

	DiffusionBrain* m_pDiffusionManifold;
	bool m_LPSDActive;

	// Phase 61.1: Hierarchical Objective Fusion (H-OF) [LEGACY-READY]
	struct RewardVector {
		float macro_path_progress;    // A* node proximity
		float meso_trauma_avoidance;  // Distance from known death-vectors
		float micro_kinetic_energy;   // Momentum preservation
	};

	struct BotExperience {
		float pos_x, pos_y;
		float vel_x, vel_y;
		int action_taken;
		RewardVector rewards;
	};

	RewardVector m_CurrentReward;
	float m_ContinuousMomentumReward;

	// Phase 37: Neural Forward Pass (V6)
	float m_NeuralInput[5]; 
	float m_NeuralOutput[2];
	void RunNeuralForwardPass();

public:
	CTasBot();
	virtual ~CTasBot();
	virtual int Sizeof() const override { return sizeof(*this); }

	virtual void OnInit() override;
	virtual void OnRender() override;
	virtual void OnUpdate() override;
	virtual void OnMapLoad() override;
	virtual void OnStateChange(int NewState, int OldState) override;
	
	void OnBeforeInput(CNetObj_PlayerInput *pInput);
	
	void StartTraining();
	void StopTraining();
	void StartPlayback();
	void SaveMemory();
	void LoadMemory();
	void SaveTasRun(const char *pFileName);
	void LoadTasRun(const char *pFileName);
	
	bool HasMasterRun() const { return m_HasMasterRun; }

	bool m_IsTraining;
	int m_PlaybackTick;
	int m_LastUpdateTick;
	int m_LastAITick;
	int m_LastRecalculateTick;
};

#endif
