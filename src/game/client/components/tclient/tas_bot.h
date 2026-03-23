#ifndef GAME_CLIENT_COMPONENTS_TCLIENT_TAS_BOT_H
#define GAME_CLIENT_COMPONENTS_TCLIENT_TAS_BOT_H

#include <game/client/component.h>
#include <vector>
#include <map>
#include <cstdint>
#include <array>
#include <generated/protocol.h>
#include <game/client/prediction/entities/character.h>
#include <base/system.h>
#include <base/vmath.h>
#include "tas_world.h"

class CTasWorld;
class CAStarPathfinder;

class CTasBot : public CComponent
{
public:
	CTasBot();
	virtual ~CTasBot();

	virtual int Sizeof() const override { return sizeof(*this); }
	virtual void OnInit() override;
	virtual void OnUpdate() override;
	virtual void OnRender() override;
	virtual void OnMapLoad() override;
	virtual void OnStateChange(int NewState, int OldState) override;

	void StartTraining();
	void StopTraining();
	void StartPlayback();
	void SaveTasRun(const char *pFileName);
	void LoadTasRun(const char *pFileName);

	bool IsTraining() const { return m_IsTraining; }
	bool HasMasterRun() const { return m_HasMasterRun; }

	// Override input to execute the master run
	void OnBeforeInput(CNetObj_PlayerInput *pInput);

private:
	struct CCandidatePath
	{
		std::vector<CNetObj_PlayerInput> m_Inputs;
		CCharacterCore m_FinalCore;
		float m_Score;
		bool m_IsDead;
	};

	float EvaluateState(const CCandidatePath& Cand, const CCharacterCore& StartCore);
	void StepTrainingBatch();
	vec2 FindBestHookTarget(vec2 Pos, float Radius);
	CNetObj_PlayerInput GetBestLiveInput(const CCharacterCore& CurrentCore);
	CNetObj_PlayerInput GetNextExplorationInput(bool ForceJump, vec2 CurrentPos);

	bool m_IsTraining;
	bool m_HasMasterRun;
	
	CTasWorld *m_pTasWorld;
	CAStarPathfinder *m_pPathfinder;
	CCharacterCore m_StartCore;

	// The sequence of inputs we are currently exploring
	std::vector<CNetObj_PlayerInput> m_CurrentPath;
	
	// Snapshot history for every tick so we can backtrack safely
	std::vector<CCharacterCore> m_StateHistory;
	
	std::vector<CNetObj_PlayerInput> m_MasterRun;
	std::vector<vec2> m_MasterRunPositions; // Expected physical coordinates per tick
	
	int m_PlaybackTick; // For live execution
	
	int m_MaxSimulationDepth;
	int m_Attempts;
	int m_TotalTicksSimulated;
	
	bool m_ForceJumpNext;
	bool m_ForceHookNext;
	
	std::vector<vec2> m_AStarPath;

	// Optimization Settings: Pro-Level Brain
	int m_NumCandidates = 64;   // Higher sampling for better hook discovery
	int m_CandidateTicks = 15; // 0.25s lookahead
	// Reinforcement Learning (Trackmania Style)
	float m_Epsilon; // Exploration rate
	int m_ConsecutiveDeadEnds;
	CNetObj_PlayerInput m_LastBestInput;
	int m_LastStrategy;
	int m_StrategyTicks;
	
	// Stuck Tracking
	int m_StuckTicks;
	vec2 m_LastStuckPos;

	// Genetic Memory: Learning from Failure
	struct SDeathInfo { vec2 m_Pos; int m_Tick; };
	std::vector<SDeathInfo> m_DeathMemory;
	CCandidatePath m_BestCandidate;

	int m_PrevAvoidFreeze;

	// Frame-Hold & Q-Learning (Trackmania Refactor)
	int m_ActionHoldFrames;     // How many frames to stick with current action
	int m_CurrentAction;        // Active discrete action index
	uint64_t m_LastStateHash;   // Hash of the previous state
	
	// Simple Q-Table: StateHash -> Fixed Array of Action Values
	std::map<uint64_t, std::array<float, 8>> m_QTable;
	
	// Sliding Window Collision Detection & Redirect State
	vec2 m_PosHistory[5];
	int m_PosHistoryIdx;
	int m_BannedAction;
	int m_BannedActionTicks;
	int m_ActionHoldElapsed; // To ensure we only check collision after ~5 frames of holding
	float m_LastDist;
	vec2 m_LastUpdatePos;
	uint64_t m_StuckCounter;
	
	uint64_t GetStateHash(const CCharacterCore& Core, vec2 DynamicWaypoint);
	vec2 GetDynamicWaypoint(vec2 BotPos);
	vec2 FindBestHookTarget(vec2 BotPos, vec2 TargetDir, float Radius);
	CNetObj_PlayerInput MapActionToInput(int Action, const CCharacterCore& Core, vec2 DynamicWaypoint);
	void UpdateQValue(uint64_t State, int Action, float Reward, uint64_t NextState);
	void SaveMemory();
	void LoadMemory();
	
	int m_LastUpdateTick; // Temporal Guard: For OnUpdate reward/stuck
	int m_LastAITick;     // Gemini Pro 50Hz Master Lock
	int m_LastRecalculateTick; // GPS Fix timer
	bool m_HyperSimulationActive; // RCON Speed status
	CNetObj_PlayerInput m_LastInput; // Cached decision
	float m_PreviousHorizontalVel;  // For Tarzan Reward
	
	// Phase 8: Inception (Local TAS World)
	CCharacterCore m_SimulatedCore;
	CWorldCore m_SimulatedWorld;
	bool m_InceptionActive;
	vec2 m_InceptionResetPos;

	void RunInceptionTick();
	void RenderInceptionGhost();
	void ResetInception();
};

#endif
