#pragma once
#include "Play.h"

class Cloth;
class ClothStick;

class ClothPoint
{
public:
	ClothPoint(Cloth* cloth, Vector2f pos);
	~ClothPoint();

	void Update(float dT);

	void AddStick(ClothStick* stick, int index);

	Vector2f GetPosition() { return m_position; }
	void SetPosition(Vector2f pos) { m_position = pos; }

	bool GetIsPinned() { return m_isPinned; }
	void SetIsPinned(bool isStatic) { m_isPinned = isStatic; }

private:
	Vector2f m_position;
	Vector2f m_prevPos;
	Vector2f m_initPos;

	Cloth* m_cloth;
	ClothStick* m_sticks[2] = {nullptr};

	bool m_isPinned = false;
};

