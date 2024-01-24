#pragma once

class ClothPoint;

class ClothStick
{
public:
	ClothStick(ClothPoint* p1, ClothPoint* p2);
	~ClothStick();

	void Update();
	void Render();

	void CalculateSpringForces();

	void SetIsSelected(bool selected) { m_isSelected = selected; }
	
	bool GetIsActive() { return m_isActive; }
	void SetIsActive(bool active) { m_isActive = active; }


private:
	ClothPoint* m_point1;
	ClothPoint* m_point2;
	float m_restLength;
	float m_k = 500.f;	// spring constant
	float m_c = 0.1f;	// damping constant
	float m_shearConstant = 60.f;
	bool m_isSelected = false;
	bool m_isActive = true;
};

