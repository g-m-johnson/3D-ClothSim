#include "Collision.h"

bool Collision::DistanceSqrTest(Vector2f A, Vector2f B, float distance)
{
	Vector2f AB = B - A;
	if (lengthSqr(AB) < distance * distance)
	{
		return true;
	}

	return false;
}

bool Collision::CircleCircleCollision(Vector2f p1, float r1, Vector2f p2, float r2)
{
	if (DistanceSqrTest(p1, p2, r1 + r2))
	{
		return true;
	}

	return false;
}

bool ScreenEdgeCircleCollision(float height, float width, Vector2f cPos, float cRad, Vector2f& collisionPos)
{
	Vector2f camPos = Play::GetCameraPosition();
	
	// Left edge
	if (cPos.x - cRad < camPos.x)
	{
		collisionPos = Vector2f(camPos.x, cPos.y);
		return true;
	}
	// Right edge
	else if (cPos.x + cRad > camPos.x + width)
	{
		collisionPos = Vector2f(camPos.x + width, cPos.y);
		return true;
	}
	// Top edge
	else if (cPos.y - cRad < camPos.y)
	{
		collisionPos = Vector2f(cPos.x, camPos.y);
		return true;
	}
	// Bottom edge
	else if (cPos.y + cRad > camPos.y + height)
	{
		collisionPos = Vector2f(cPos.y, camPos.y + height);
		return true;
	}

	return false;
}


