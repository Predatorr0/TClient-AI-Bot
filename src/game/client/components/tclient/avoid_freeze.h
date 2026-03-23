/* Avoiding Freeze - DDNet TAS Component
 * Predicts future tee positions and automatically modifies input
 * to prevent entering freeze/death/teleport tiles.
 * Supports legit (natural) and rage (aggressive) modes.
 * Ping-adaptive lookahead with KoG tight-passage navigation.
 */

#ifndef GAME_CLIENT_COMPONENTS_TCLIENT_AVOID_FREEZE_H
#define GAME_CLIENT_COMPONENTS_TCLIENT_AVOID_FREEZE_H

#include <game/client/component.h>
#include <game/gamecore.h>
#include <generated/protocol.h>

class CAvoidFreeze : public CComponent
{
public:
	virtual ~CAvoidFreeze() {}
	// Danger level for visual indicator (red/orange/green)
	enum EDangerLevel
	{
		DANGER_SAFE = 0,   // Green - no freeze nearby
		DANGER_WARNING,    // Orange - freeze is close, monitoring
		DANGER_CRITICAL,   // Red - about to enter freeze, taking action
	};

	// Mode of operation
	enum EMode
	{
		MODE_OFF = 0,
		MODE_LEGIT = 1,   // Natural-looking avoidance
		MODE_RAGE = 2,    // Aggressive, never enters freeze
	};

	// Scenario types we simulate
	enum EScenario
	{
		SCENARIO_CURRENT = 0,     // Keep current input
		SCENARIO_STOP,            // Release direction key
		SCENARIO_REVERSE,         // Press opposite direction
		SCENARIO_JUMP,            // Current input + jump
		SCENARIO_HOOK,            // Current input + hook to nearest safe wall
		SCENARIO_STOP_HOOK,       // Release hook
		NUM_SCENARIOS
	};

	// Result of simulating a scenario N ticks ahead
	struct CSimulationResult
	{
		bool m_IsSafe = true;           // Final position is safe
		bool m_WillFreeze = false;
		bool m_WillDie = false;
		bool m_WillTele = false;
		bool m_WillLaser = false;
		int m_FirstDangerTick = -1;
		float m_NearestFreezeDist = 999.0f;
		vec2 m_FinalPos;                // Predicted position at the end of lookahead
		CNetObj_PlayerInput m_ModifiedInput;
	};

	// History entry for death analysis
	struct CHistoryEntry
	{
		int m_Tick;
		vec2 m_Pos;
		vec2 m_Vel;
		CNetObj_PlayerInput m_Input;
		EScenario m_Scenario;
		EDangerLevel m_Danger;
	};

	int GetTilesAt(vec2 Pos, float Proximity) const;

	virtual int Sizeof() const override { return sizeof(*this); }
	virtual void OnInit() override;
	virtual void OnRender() override;
	void UpdateLearning(bool IsDeath, bool IsFreeze);

	// Called before input is sent to modify it if needed
	void OnBeforeInput(CNetObj_PlayerInput *pInput);

	// Safety check for external components (like CBot)
	bool TestInputSafety(const CCharacterCore &Core, const CNetObj_PlayerInput &Input, int LookaheadTicks) const;

	// Get current danger level for HUD display
	EDangerLevel GetDangerLevel() const { return m_DangerLevel; }
	bool IsActive() const;

private:
	// Main logic
	int CalcLookaheadTicks() const;
	void SimulateScenario(const CCharacterCore &CurrentCore, const CNetObj_PlayerInput &BaseInput,
		EScenario Scenario, int NumTicks, CSimulationResult &Result) const;
	EScenario ChooseBestScenario(const CCharacterCore &Core, const CSimulationResult *pResults) const;

	// Tile danger checks
	bool IsDangerousTileAt(vec2 Pos) const;
	bool IsFreezeTileAt(vec2 Pos) const;
	bool IsDeathTileAt(vec2 Pos) const;
	bool IsTeleTileAt(vec2 Pos) const;
	float DistanceToNearestFreeze(vec2 Pos) const;

	// KoG tight passage helpers
	bool IsInTightPassage(vec2 Pos) const;
	CSimulationResult PredictStoppingPath(const CCharacterCore &Core) const;
	bool CanPassSafely(const CCharacterCore &Core, const CNetObj_PlayerInput &Input, int Ticks) const;

	// Hook target finder
	vec2 FindSafeHookTarget(vec2 Pos, vec2 Vel) const;

	// State
	EDangerLevel m_DangerLevel = DANGER_SAFE;
	EScenario m_ActiveScenario = SCENARIO_CURRENT;
	int m_LegitDelayTicks = 0;    // For legit mode: gradual reaction
	bool m_WasModifying = false;  // Tracking if we modified input last tick

	// Stuck detection
	vec2 m_LastPos = vec2(0, 0);
	int m_StuckTicks = 0;
	
	// NEW: Logging & Analysis
	char m_aLastMapName[128];
	std::vector<CHistoryEntry> m_History;
	int64_t m_LastLogTime;
};

#endif // GAME_CLIENT_COMPONENTS_TCLIENT_AVOID_FREEZE_H
