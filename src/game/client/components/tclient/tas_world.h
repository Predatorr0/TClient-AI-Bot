#ifndef GAME_CLIENT_COMPONENTS_TCLIENT_TAS_WORLD_H
#define GAME_CLIENT_COMPONENTS_TCLIENT_TAS_WORLD_H

#include <game/client/prediction/entities/character.h>
#include <game/mapitems.h>
#include <generated/protocol.h>

class CCollision;

class CTasWorld
{
public:
	CTasWorld(CCollision *pCollision);
	~CTasWorld();

	// Load the initial state from the live game
	void LoadFromCurrentCore(const CCharacterCore &Core);

	// Taking snapshots of internal state for backtracking
	void SaveSnapshot();
	void LoadSnapshot();

	// Advance simulation by 1 tick using a specific input
	void SimulateTick(const CNetObj_PlayerInput &Input);

	// Status checks via collision map
	bool IsDeadly() const;
	bool IsFrozen() const;
	bool ReachedFinish() const;

	const CCharacterCore &GetCore() const { return m_Core; }
	int GetTickCount() const { return m_TickCount; }

private:
	CCollision *m_pCollision;
	CCharacterCore m_Core;
	CCharacterCore m_SnapshotCore; // Single-level backup for now
	int m_TickCount;
	int m_SnapshotTick;
};

#endif
