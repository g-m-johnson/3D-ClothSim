#include "PhysSimMain.h"
#include "Cloth.h"
#include "ClothPoint.h"
#include "ClothStick.h"
#include "Mouse.h"

ClothParticle::ClothParticle(Cloth* cloth, Vector3f pos)
	: m_cloth(cloth)
	, m_position(pos)
	, m_prevPos(pos)
	, m_initPos(pos)
{}

ClothParticle::~ClothParticle()
{}

void ClothParticle::VerletIntegration()
{
	m_prevPos = m_position;
	float dT = System::GetDeltaTime();
 	m_acceleration = m_forces * 1.0f;
	m_position = 2.0f * m_position - m_prevPos + m_acceleration * dT * dT;
}

void ClothParticle::AddStick(ClothSpring* stick, int index)
{
	m_sticks[index] = stick;
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
	m_forces += m_cloth->GetWindForce();

	// External forces
	m_forces += m_forcesExt;
	m_forcesExt = Vector3f(0, 0, 0);
}

void ClothParticle::ApplyExternalForce(Vector3f force)
{
	m_forcesExt = force;
}
