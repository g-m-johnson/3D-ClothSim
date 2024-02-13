#pragma once
#include "3D-ClothSimMain.h"

class ClothParticle;

class ClothConstraint
{
public:
	ClothConstraint(ClothParticle* p1, ClothParticle* p2);
	~ClothConstraint();

	void CalculateSpringForces() const;
	void DebugDrawConstraints() const;

private:
	ClothParticle* const m_point1;
	ClothParticle* const m_point2;

	float m_restLength;
	const float m_k = SPRING_CONSTANT;	// spring constant
	const float m_c = DAMPING_CONSTANT;	// damping constant
	
	const bool m_drawConstraints = false;
};

