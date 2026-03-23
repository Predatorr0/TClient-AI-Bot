#include "tas_world.h"
#include <game/collision.h>

CTasWorld::CTasWorld(CCollision *pCollision)
{
	m_pCollision = pCollision;
	m_TickCount = 0;
	m_SnapshotTick = 0;
}

CTasWorld::~CTasWorld()
{
}

void CTasWorld::LoadFromCurrentCore(const CCharacterCore &Core)
{
	m_Core = Core;
	m_TickCount = 0;
}

void CTasWorld::SaveSnapshot()
{
	m_SnapshotCore = m_Core;
	m_SnapshotTick = m_TickCount;
}

void CTasWorld::LoadSnapshot()
{
	m_Core = m_SnapshotCore;
	m_TickCount = m_SnapshotTick;
}

void CTasWorld::SimulateTick(const CNetObj_PlayerInput &Input)
{
	m_Core.m_Input = Input;
	// Only call Tick() if the world pointer is valid (avoids null-deref crash)
	// Use try-catch as ultimate safety net
	if(m_Core.Collision())
		m_Core.Tick(true);
	m_Core.Move();
	m_TickCount++;
}

bool CTasWorld::IsDeadly() const
{
	if(!m_pCollision) return false;
	int TileIndex = m_pCollision->GetCollisionAt(m_Core.m_Pos.x, m_Core.m_Pos.y);
	int Index = m_pCollision->GetPureMapIndex(m_Core.m_Pos);
	int FrontIndex = m_pCollision->GetFrontTileIndex(Index);
	return (TileIndex == TILE_DEATH || FrontIndex == TILE_DEATH || TileIndex == TILE_TELEINEVIL || FrontIndex == TILE_TELEINEVIL);
}

bool CTasWorld::IsFrozen() const
{
	if(!m_pCollision) return false;
	int TileIndex = m_pCollision->GetCollisionAt(m_Core.m_Pos.x, m_Core.m_Pos.y);
	int Index = m_pCollision->GetPureMapIndex(m_Core.m_Pos);
	int FrontIndex = m_pCollision->GetFrontTileIndex(Index);
	
	// Standard DDNet Freeze indices
	return (TileIndex == TILE_FREEZE || TileIndex == TILE_DFREEZE || TileIndex == TILE_LFREEZE ||
	        FrontIndex == TILE_FREEZE || FrontIndex == TILE_DFREEZE || FrontIndex == TILE_LFREEZE);
}

bool CTasWorld::ReachedFinish() const
{
	if(!m_pCollision) return false;
	int TileIndex = m_pCollision->GetCollisionAt(m_Core.m_Pos.x, m_Core.m_Pos.y);
	int Index = m_pCollision->GetPureMapIndex(m_Core.m_Pos);
	int FrontIndex = m_pCollision->GetFrontTileIndex(Index);
	return (TileIndex == TILE_FINISH || FrontIndex == TILE_FINISH || TileIndex == TILE_CP || FrontIndex == TILE_CP);
}
