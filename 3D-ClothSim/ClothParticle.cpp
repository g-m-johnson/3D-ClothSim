#include "3D-ClothSimMain.h"
#include "Cloth.h"
#include "ClothParticle.h"
#include "ClothConstraint.h"
#include "Mouse.h"

ClothParticle::ClothParticle(Cloth* cloth, Vector3f pos)
	: m_cloth(cloth)
	, m_position(pos)
{}

ClothParticle::~ClothParticle()
{}

void ClothParticle::VerletIntegration()
{
	float dT = System::GetDeltaTime();
	m_prevAccn = m_acceleration;

	m_position = m_position + m_velocity * dT + m_acceleration * dT * dT * 0.5f;
 	m_acceleration = m_forces;
	m_velocity = m_velocity + (m_prevAccn + m_acceleration) * dT * 0.5f;
}

void ClothParticle::CalculateForces()
{
	// Gravity
	m_forces.y += m_cloth->GetGravity().y * m_mass;
	
	// Drag 
	Vector3f drag = -m_velocity;
	drag = normalize(drag);
	m_forces += drag * (length(m_velocity) * length(m_velocity)) * m_cloth->GetDrag();

	// Wind
	//m_forces += m_cloth->GetWindForce();

	// External forces
	m_forces += m_forcesExt;
	m_forcesExt = Vector3f(0, 0, 0);
}
