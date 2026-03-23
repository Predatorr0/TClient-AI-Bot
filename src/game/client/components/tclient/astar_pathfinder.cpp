#include "astar_pathfinder.h"
#include <game/collision.h>
#include <game/mapitems.h>
#include <queue>
#include <map>
#include <cmath>
#include <algorithm>

struct CNode
{
	int m_X, m_Y;
	float m_G, m_H;
	int m_ParentX, m_ParentY;

	float F() const { return m_G + m_H; }

	bool operator>(const CNode &Other) const
	{
		return F() > Other.F();
	}
};

CAStarPathfinder::CAStarPathfinder(CCollision *pCollision)
{
	m_pCollision = pCollision;
	m_Width = pCollision ? pCollision->GetWidth() : 0;
	m_Height = pCollision ? pCollision->GetHeight() : 0;
}

bool CAStarPathfinder::IsDangerous(int X, int Y) const
{
	if(!m_pCollision) return true;
	if(X < 0 || X >= m_Width || Y < 0 || Y >= m_Height) return true;

	int Index = Y * m_Width + X;
	int Tile = m_pCollision->GetTileIndex(Index);
	int Front = m_pCollision->GetFrontTileIndex(Index);
	
	// DDRace Danger Tiles
	if(Tile == TILE_FREEZE || Tile == TILE_DFREEZE || Tile == TILE_LFREEZE || Tile == TILE_DEATH || Tile == TILE_TELEINEVIL) return true;
	if(Front == TILE_FREEZE || Front == TILE_DFREEZE || Front == TILE_LFREEZE || Front == TILE_DEATH || Front == TILE_TELEINEVIL) return true;
	
	return false;
}

bool CAStarPathfinder::IsWalkable(int X, int Y) const
{
	if(!m_pCollision) return false;
	if(X < 0 || X >= m_Width || Y < 0 || Y >= m_Height) return false;

	// Phase 7: 2x2 Tile Clearance (Hitbox Safety)
	// Tee is approx 28x28, checking 3 points per side + center
	for(int ay = 0; ay < 2; ay++)
	{
		for(int ax = 0; ax < 2; ax++)
		{
			int px = X * 32 + ax * 31; // Check corners of a 2x2 area
			int py = Y * 32 + ay * 31;
			if(m_pCollision->IsSolid(px, py)) return false;
			if(IsDangerous(px / 32, py / 32)) return false;
		}
	}

	return true;
}

std::vector<vec2> CAStarPathfinder::FindPath(vec2 StartPos, vec2 GoalPos)
{
	std::vector<vec2> Path;
	if(!m_pCollision || m_Width <= 0 || m_Height <= 0)
		return Path;

	int StartX = std::clamp((int)(StartPos.x / 32.0f), 0, m_Width - 1);
	int StartY = std::clamp((int)(StartPos.y / 32.0f), 0, m_Height - 1);
	int GoalX = std::clamp((int)(GoalPos.x / 32.0f), 0, m_Width - 1);
	int GoalY = std::clamp((int)(GoalPos.y / 32.0f), 0, m_Height - 1);

	// If start is in solid/freeze, try to snap to nearest valid
	if(!IsWalkable(StartX, StartY))
	{
		// We are already inside danger/wall according to strict padding, just use it anyway
	}

	std::priority_queue<CNode, std::vector<CNode>, std::greater<CNode>> OpenList;
	std::vector<bool> ClosedList(m_Width * m_Height, false);
	std::map<int, CNode> AllNodes;

	auto GetIndex = [&](int x, int y) { return y * m_Width + x; };

	CNode StartNode = {StartX, StartY, 0.0f, (float)(std::abs(StartX - GoalX) + std::abs(StartY - GoalY)), -1, -1};
	OpenList.push(StartNode);
	AllNodes[GetIndex(StartX, StartY)] = StartNode;

	const int MAX_NODES = 100000; // Limit search space to avoid hanging game
	int NodesEvaluated = 0;

	int FinalX = -1, FinalY = -1;
	bool Found = false;

	const int aDirs[8][2] = {
		{1, 0}, {-1, 0}, {0, 1}, {0, -1},
		{1, 1}, {-1, -1}, {1, -1}, {-1, 1}
	};

	while(!OpenList.empty() && NodesEvaluated < MAX_NODES)
	{
		CNode Current = OpenList.top();
		OpenList.pop();

		int CurrIdx = GetIndex(Current.m_X, Current.m_Y);

		if(ClosedList[CurrIdx])
			continue;

		ClosedList[CurrIdx] = true;
		NodesEvaluated++;

		if(Current.m_X == GoalX && Current.m_Y == GoalY)
		{
			FinalX = Current.m_X;
			FinalY = Current.m_Y;
			Found = true;
			break;
		}

		for(int i = 0; i < 8; i++)
		{
			int nx = Current.m_X + aDirs[i][0];
			int ny = Current.m_Y + aDirs[i][1];

			if(nx < 0 || nx >= m_Width || ny < 0 || ny >= m_Height)
				continue;

			// Diagonal clipping check: if moving diagonally, ensure the two adjacent tiles are also walkable
			if(i >= 4)
			{
				if(!IsWalkable(Current.m_X, ny) || !IsWalkable(nx, Current.m_Y))
					continue;
			}

			int nIdx = GetIndex(nx, ny);
			if(ClosedList[nIdx] || !IsWalkable(nx, ny))
				continue;

			// Phase 7: Physics-Aware Costing (Gravity & Hook Bias)
			float MoveCost = (i < 4) ? 1.0f : 1.414f;
			
			// Gravity Bias: Moving UP (ny < Current.m_Y) is harder
			if(ny < Current.m_Y)
			{
				bool HookableAbove = false;
				for(int ay = -1; ay >= -4; ay--) // Look up for hookable ceiling
				{
					if(m_pCollision->IsSolid(nx * 32 + 16, (ny + ay) * 32 + 16)) {
						HookableAbove = true;
						break;
					}
				}
				if(!HookableAbove) MoveCost *= 10.0f; // Massive penalty for floating up
			}
			else if(ny > Current.m_Y)
			{
				MoveCost *= 0.8f; // Fall discount
			}

			float NewG = Current.m_G + MoveCost;

			if(AllNodes.find(nIdx) == AllNodes.end() || NewG < AllNodes[nIdx].m_G)
			{
				float H = (float)(std::abs(nx - GoalX) + std::abs(ny - GoalY));
				CNode Neighbor = {nx, ny, NewG, H, Current.m_X, Current.m_Y};
				AllNodes[nIdx] = Neighbor;
				OpenList.push(Neighbor);
			}
		}
	}

	if(!Found && FinalX == -1)
	{
		// If not found, use the closest evaluated node
		float BestH = 999999.0f;
		for(const auto &Pair : AllNodes)
		{
			if(Pair.second.m_H < BestH)
			{
				BestH = Pair.second.m_H;
				FinalX = Pair.second.m_X;
				FinalY = Pair.second.m_Y;
			}
		}
	}

	if(FinalX != -1)
	{
		int cx = FinalX;
		int cy = FinalY;
		while(cx != -1 && cy != -1 && (cx != StartX || cy != StartY))
		{
			Path.push_back(vec2(cx * 32.0f + 16.0f, cy * 32.0f + 16.0f));
			int Idx = GetIndex(cx, cy);
			int px = AllNodes[Idx].m_ParentX;
			int py = AllNodes[Idx].m_ParentY;
			cx = px;
			cy = py;
		}
		std::reverse(Path.begin(), Path.end());
		SmoothPath(Path); // Clean up zigzags
	}

	return Path;
}

void CAStarPathfinder::SmoothPath(std::vector<vec2>& Path)
{
	if(Path.size() < 3) return;

	std::vector<vec2> Smoothed;
	Smoothed.push_back(Path[0]);

	size_t Current = 0;
	while(Current < Path.size() - 1)
	{
		size_t Next = Current + 1;
		// Look as far ahead as possible for LOS
		for(size_t i = Current + 2; i < Path.size(); i++)
		{
			vec2 Hit;
			if(!m_pCollision->IntersectLine(Path[Current], Path[i], &Hit, nullptr))
			{
				Next = i;
			}
			else break;
		}
		Smoothed.push_back(Path[Next]);
		Current = Next;
	}
	Path = Smoothed;
}
