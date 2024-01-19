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
	void Initialise();
	void Update();
	Vector2f GetMousePos() { return m_position; }
	Vector2f GetMousePrevPos() { return m_prevPos; }
	float GetMouseCursorSize() { return m_cursorSize; }

	Vector3f GetMouseRaycast();
private:
	Mouse(){}
	
	Vector2f m_position;
	Vector2f m_prevPos;
	
	float m_cursorSize = 30;
};

