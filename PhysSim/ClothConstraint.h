#pragma once

class ClothParticle;

class ClothConstraint
{
public:
	ClothConstraint(ClothParticle* p1, ClothParticle* p2);
	~ClothConstraint();

	void CalculateSpringForces();

private:
	ClothParticle* m_point1;
	ClothParticle* m_point2;
	float m_restLength;
	float m_k = 1000.f;	// spring constant
	float m_c = 1.f;	// damping constant
};

