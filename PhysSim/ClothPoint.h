#pragma once
#include "Play.h"

class ClothPoint
{
public:
	ClothPoint(Vector2f pos);
	~ClothPoint();

	void Update();
	void Render();

	Vector2f GetPosition() { return m_position; }
	void SetPosition(Vector2f pos) { m_position = pos; }
private:
	Vector2f m_position;

};

