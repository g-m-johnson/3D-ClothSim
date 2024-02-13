#pragma once
#include "Geometry.h"
#include "PhysSimMain.h"

class Mouse
{
public:

	static Mouse& Instance()
	{
		static Mouse *Instance = new Mouse();
		return *Instance;
	}
	
	void Update();

	const Vector3f GetRayDirection(const Matrix4x4f& viewMatrix, const Matrix4x4f& projectMatrix, const Vector3f& cameraPosWorldSpace);
	void DebugDrawMouseRay();

private:
	Mouse(){}
	
	Geometry::Ray m_ray;
	Vector3f m_mouseWorld;
	
	const float m_cursorSize = CURSOR_RADIUS;
};

