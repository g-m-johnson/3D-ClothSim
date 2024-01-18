#pragma once
#include "../Play3d/Play3d.h"
using namespace Play3d;

class Cloth;
class ClothStick;

class ClothPoint
{
public:
	ClothPoint(Cloth* cloth, Vector3f pos);
	~ClothPoint();

	void Update();

	void AddStick(ClothStick* stick, int index);

	Vector3f GetPosition() { return m_position; }
	void SetPosition(Vector3f pos) { m_position = pos; }

	bool GetIsPinned() { return m_isPinned; }
	void SetIsPinned(bool isStatic) { m_isPinned = isStatic; }

private:
	Vector3f m_position;
	Vector3f m_prevPos;
	Vector3f m_initPos;

	Cloth* m_cloth;
	ClothStick* m_sticks[2] = {nullptr};

	bool m_isPinned = false;
};

