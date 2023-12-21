#pragma once

class ClothPoint;

class ClothStick
{
public:
	ClothStick(ClothPoint* leftPoint, ClothPoint* rightPoint);
	~ClothStick();

	void Update();
	void Render();

	void SetIsSelected(bool selected) { m_isSelected = selected; }
	void SetIsActive(bool active) { m_isActive = active; }

private:
	ClothPoint* m_leftPoint;
	ClothPoint* m_rightPoint;
	float m_length;
	bool m_isSelected = false;
	bool m_isActive = true;
};

