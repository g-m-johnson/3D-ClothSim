#pragma once
#include "Geometry.h"

class Mouse
{
public:

	static Mouse& Instance()
	{
		static Mouse *Instance = new Mouse();
		return *Instance;
	}
	
	void Update();

	Vector3f GetRayDirection(const Matrix4x4f& viewMatrix, const Matrix4x4f& projectMatrix, const Vector3f& cameraPosWorldSpace);
	void Raycast();
	void DebugDrawMouseRay();

private:
	Mouse(){}
	
	Geometry::Ray m_ray;

	Vector3f m_mouseWorld;

	Vector2f m_position;
	Vector2f m_prevPos;
	
	float m_cursorSize = 5;

	bool m_isMouseReleased = true;
};

