#pragma once
#include "Play.h"

namespace Collision
{
	bool DistanceSqrTest(Vector2f A, Vector2f B, float distance);
	bool CircleCircleCollision(Vector2f p1, float r1, Vector2f p2, float r2);

	bool ScreenEdgeCircleCollision(float height, float width, Vector2f cPos, float cRad, Vector2f& collisionPos);
}