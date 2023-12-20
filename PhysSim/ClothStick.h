#pragma once

class ClothPoint;

class ClothStick
{
public:
	ClothStick(ClothPoint* leftPoint, ClothPoint* rightPoint);
	~ClothStick();

	void Update();
	void Render();

private:
	ClothPoint* m_leftPoint;
	ClothPoint* m_rightPoint;
};

