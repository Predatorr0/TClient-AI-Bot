#include "bot.h"
#include <engine/shared/config.h>
#include <engine/console.h>
#include <game/client/gameclient.h>
#include <game/client/components/controls.h>
#include <game/client/components/tclient/avoid_freeze.h>
#include <game/collision.h>
#include <game/mapitems.h>
#include <generated/protocol.h>
#include <game/mapbugs.h>
#include <game/client/prediction/entities/character.h>

void CBot::OnInit()
{
}

void CBot::OnMapLoad()
{
	ScanMap();
}

void CBot::ScanMap()
{
	m_vTargets.clear();
	CCollision *pCollision = GameClient()->Collision();
	if(!pCollision) return;

	int Width = pCollision->GetWidth();
	int Height = pCollision->GetHeight();

	for(int y = 0; y < Height; y++)
	{
		for(int x = 0; x < Width; x++)
		{
			int Index = y * Width + x;
			int TileIndex = pCollision->GetTileIndex(Index);
			int FrontIndex = pCollision->GetFrontTileIndex(Index);

			// Check for Finish or Checkpoints
			if(TileIndex == TILE_FINISH || FrontIndex == TILE_FINISH ||
			   TileIndex == TILE_CP || FrontIndex == TILE_CP ||
			   (TileIndex >= TILE_TIME_CHECKPOINT_FIRST && TileIndex <= TILE_TIME_CHECKPOINT_LAST))
			{
				m_vTargets.push_back(vec2(x * 32.0f + 16.0f, y * 32.0f + 16.0f));
			}
		}
	}

	if(!m_vTargets.empty())
	{
		dbg_msg("bot", "Map scanned: Found %d path targets (Finish/Checkpoints)", (int)m_vTargets.size());
	}
}

void CBot::FindNextTarget()
{
	if(m_vTargets.empty()) return;

	CCharacter *pLocalChar = GameClient()->m_PredictedWorld.GetCharacterById(GameClient()->m_Snap.m_LocalClientId);
	if(!pLocalChar) return;

	vec2 Pos = pLocalChar->GetCore().m_Pos;
	float MinDist = 1000000.0f;
	
	for(const vec2 &Target : m_vTargets)
	{
		float D = distance(Pos, Target);
		if(D < MinDist)
		{
			MinDist = D;
			m_CurrentTarget = Target;
			m_HasTarget = true;
		}
	}
}

void CBot::OnBeforeInput(CNetObj_PlayerInput *pInput)
{
	if(g_Config.m_TcBotEnabled == 0)
		return;

	// === SAFETY GATE: Don't run bot logic until game state is fully ready ===
	if(!GameClient()->Collision() || GameClient()->m_Snap.m_LocalClientId < 0)
		return;

	FindNextTarget();

	// Update fail points (cooldown)
	for(auto it = m_vFailPoints.begin(); it != m_vFailPoints.end();)
	{
		it->m_TicksRemaining--;
		if(it->m_TicksRemaining <= 0)
			it = m_vFailPoints.erase(it);
		else
			++it;
	}

	// Detect "failure" (Death or deep freeze)
	int LocalId = GameClient()->m_Snap.m_LocalClientId;
	if(LocalId >= 0 && LocalId < MAX_CLIENTS)
	{
		auto *pClient = &GameClient()->m_aClients[LocalId];
		if(pClient->m_Active && (pClient->m_DeepFrozen || pClient->m_FreezeEnd > Client()->GameTick(g_Config.m_ClDummy)))
		{
			// Use local character pos safely
			CCharacter *pLocalChar = GameClient()->m_PredictedWorld.GetCharacterById(LocalId);
			if(pLocalChar)
			{
				vec2 CurrentPos = pLocalChar->GetCore().m_Pos;
				if(!IsNearFailPoint(CurrentPos))
				{
					AddFailPoint(CurrentPos);
					// Log
					dbg_msg("tc_bot", "Logged failure point at %.2f, %.2f", CurrentPos.x, CurrentPos.y);
				}
			}
		}
	}

	if(!m_HasTarget)
		return;

	CCharacter *pLocalChar = GameClient()->m_PredictedWorld.GetCharacterById(LocalId);
	if(!pLocalChar)
		return;

	vec2 Pos = pLocalChar->GetCore().m_Pos;
	vec2 Diff = m_CurrentTarget - Pos;

	int TargetDir = 0;
	if(Diff.x > 32.0f)
		TargetDir = 1;
	else if(Diff.x < -32.0f)
		TargetDir = -1;

	// COLLABORATION: Ask Avoid Freeze if our intended move is safe
	CNetObj_PlayerInput TestInput = *pInput;
	TestInput.m_Direction = TargetDir;

	bool Safe = GameClient()->m_AvoidFreeze.TestInputSafety(pLocalChar->GetCore(), TestInput, 15);

	if(!Safe)
	{
		// If moving is unsafe, try jumping as well
		TestInput.m_Jump = 1;
		if(GameClient()->m_AvoidFreeze.TestInputSafety(pLocalChar->GetCore(), TestInput, 15))
		{
			pInput->m_Direction = TargetDir;
			pInput->m_Jump = 1;
		}
		else
		{
			// Still unsafe? Stay still
			pInput->m_Direction = 0;
		}
	}
	else
	{
		pInput->m_Direction = TargetDir;
	}

	// Obstacle avoidance (Fail points memory)
	if(IsNearFailPoint(Pos + vec2(pInput->m_Direction * 64.0f, 0.0f)))
	{
		pInput->m_Jump = 1;
	}

	// Jump if there is a wall in front
	if(GameClient()->Collision() && GameClient()->Collision()->CheckPoint(Pos.x + pInput->m_Direction * 32.0f, Pos.y))
		pInput->m_Jump = 1;

	// Aim
	vec2 AimDir = m_CurrentTarget - Pos;
	pInput->m_TargetX = (int)AimDir.x;
	pInput->m_TargetY = (int)AimDir.y;
}

void CBot::AddFailPoint(vec2 Pos)
{
	SFailPoint fp;
	fp.m_Pos = Pos;
	fp.m_TicksRemaining = 1000; // ~20 seconds
	m_vFailPoints.push_back(fp);
}

bool CBot::IsNearFailPoint(vec2 Pos) const
{
	for(const auto &fp : m_vFailPoints)
	{
		if(distance(fp.m_Pos, Pos) < 128.0f)
			return true;
	}
	return false;
}

void CBot::OnRender()
{
	// HUD or Visual debugging can go here
}
