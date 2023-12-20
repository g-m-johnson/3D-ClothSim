#pragma once
#include "Play.h"

class ClothPoint;
class ClothStick;

class Cloth
{
public:
	Cloth(int width, int height, int spacing, Vector2f start);
	~Cloth();

	void Update();
	void Render();

private:
	Vector2f m_gravity{ 0.0f, 981.0f };
	float m_drag{ 0.01f };
	float m_elasticity{ 10.f };

	std::vector<ClothPoint*> m_vPoints;
	std::vector<ClothStick*> m_vSticks;
};

