#pragma once

class ClothParticle;

class ClothConstraint
{
public:
	ClothConstraint(ClothParticle* p1, ClothParticle* p2);
	~ClothConstraint();

	void Update();
	void Render();

	void CalculateSpringForces();

	void SetIsSelected(bool selected) { m_isSelected = selected; }
	
	bool GetIsActive() { return m_isActive; }
	void SetIsActive(bool active) { m_isActive = active; }


private:
	ClothParticle* m_point1;
	ClothParticle* m_point2;
	float m_restLength;
	float m_k = 1500.f;	// spring constant
	float m_c = 0.1f;	// damping constant
	bool m_isSelected = false;
	bool m_isActive = true;
};

