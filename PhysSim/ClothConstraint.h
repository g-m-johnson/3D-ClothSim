#pragma once
#include "PhysSimMain.h"

class ClothParticle;

class ClothConstraint
{
public:
	ClothConstraint(ClothParticle* p1, ClothParticle* p2);
	~ClothConstraint();

	void CalculateSpringForces();
	void DebugDrawConstraints();

private:
	ClothParticle* m_point1;
	ClothParticle* m_point2;

	float m_restLength;
	float m_k = SPRING_CONSTANT;	// spring constant
	float m_c = DAMPING_CONSTANT;	// damping constant
	
	bool m_drawConstraints = false;
};

