#pragma once
#include "../Play3d/Play3d.h"
using namespace Play3d;

class Mouse
{
public:

	static Mouse& Instance()
	{
		static Mouse *Instance = new Mouse();
		return *Instance;
	}
	
	Vector3f GetRayDirection(const Matrix4x4f& viewMatrix, const Matrix4x4f& projectMatrix, const Vector3f& cameraPosWorldSpace);
	void Raycast();

private:
	Mouse(){}
	
	Vector2f m_position;
	Vector2f m_prevPos;
	
	float m_cursorSize = 30;
};

