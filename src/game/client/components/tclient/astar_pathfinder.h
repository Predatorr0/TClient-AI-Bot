#ifndef GAME_CLIENT_COMPONENTS_TCLIENT_ASTAR_H
#define GAME_CLIENT_COMPONENTS_TCLIENT_ASTAR_H

#include <vector>
#include <base/vmath.h>

class CCollision;

class CAStarPathfinder
{
public:
	CAStarPathfinder(CCollision *pCollision);

	// Returns a list of REAL WORLD coordinates forming the path from Start to Goal
	std::vector<vec2> FindPath(vec2 StartPos, vec2 GoalPos);

private:
	bool IsWalkable(int X, int Y) const;
	bool IsDangerous(int X, int Y) const;

	CCollision *m_pCollision;
	int m_Width;
	int m_Height;
};

#endif // GAME_CLIENT_COMPONENTS_TCLIENT_ASTAR_H
