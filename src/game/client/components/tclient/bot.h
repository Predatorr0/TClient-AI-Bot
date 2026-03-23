/* AI Speedrun Bot for TClient
 * Scans map for Finish/Checkpoint tiles and attempts to navigate
 * towards them using a pathfinding layer.
 */

#ifndef GAME_CLIENT_COMPONENTS_TCLIENT_BOT_H
#define GAME_CLIENT_COMPONENTS_TCLIENT_BOT_H

#include <game/client/component.h>
#include <base/vmath.h>
#include <vector>

struct CNetObj_PlayerInput;
class CCharacterCore;

class CBot : public CComponent
{
public:
	virtual int Sizeof() const override { return sizeof(*this); }
	virtual void OnInit() override;
	virtual void OnMapLoad() override;
	virtual void OnRender() override;

	// Main logic to decide inputs
	void OnBeforeInput(CNetObj_PlayerInput *pInput);

private:
	bool m_BotEnabled = false;
	std::vector<vec2> m_vTargets; // Finish and Checkpoint positions
	vec2 m_CurrentTarget;
	bool m_HasTarget = false;

	struct SFailPoint
	{
		vec2 m_Pos;
		int m_TicksRemaining;
	};
	std::vector<SFailPoint> m_vFailPoints;

	void ScanMap();
	void FindNextTarget();
	void AddFailPoint(vec2 Pos);
	bool IsNearFailPoint(vec2 Pos) const;
};

#endif // GAME_CLIENT_COMPONENTS_TCLIENT_BOT_H
