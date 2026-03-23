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

	// Check if the center is solid
	if(m_pCollision->IsSolid(X * 32 + 16, Y * 32 + 16))
		return false;

	// Check for danger (Freeze, Death, etc)
	if(IsDangerous(X, Y))
		return false;

	// Character radius safety: Check 4 corners of the character (approx 14 units radius)
	// We check if any of these corners hit a solid wall
	int px = X * 32 + 16;
	int py = Y * 32 + 16;
	const int r = 14;
	if(m_pCollision->IsSolid(px - r, py - r) || m_pCollision->IsSolid(px + r, py - r) ||
	   m_pCollision->IsSolid(px - r, py + r) || m_pCollision->IsSolid(px + r, py + r))
		return false;

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

			// Diagonals cost more (sqrt(2) approx 1.414)
			float MoveCost = (i < 4) ? 1.0f : 1.414f;
			
			// Add penalty if falling freely without ground below (encourage jumping/hooking properly? 
			// A* doesn't know physics, so we just stick to generic shortest-path).
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
	}
	else
	{
		// Log why it failed (this will be visible in the console if called from CTasBot)
		// Note: CAStarPathfinder doesn't have its own console access, so CTasBot will log Path.empty()
	}

	return Path;
}
