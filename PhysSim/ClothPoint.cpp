#include "PhysSimMain.h"
#include "Cloth.h"
#include "ClothPoint.h"
#include "ClothStick.h"
#include "Mouse.h"

ClothPoint::ClothPoint(Cloth* cloth, Vector3f pos)
	: m_cloth(cloth)
	, m_position(pos)
	, m_prevPos(pos)
	, m_initPos(pos)
{}

ClothPoint::~ClothPoint()
{}

void ClothPoint::VerletIntegration()
{
	m_prevPos = m_position;
	float dT = System::GetDeltaTime();
 	m_acceleration = m_forces * 1.0f;
	m_position = 2.0f * m_position - m_prevPos + m_acceleration * dT * dT;
}

void ClothPoint::AddStick(ClothStick* stick, int index)
{
	m_sticks[index] = stick;
}

void ClothPoint::CalculateForces()
{
	// Gravity
	m_forces.y += m_cloth->GetGravity().y * m_mass;
	
	// Drag 
	Vector3f drag = -m_velocity;
	drag = normalize(drag);
	m_forces += drag * (length(m_velocity) * length(m_velocity)) * m_cloth->GetDrag();

	// Wind
	Vector3f wind = Vector3f((float)((rand() % 200) - 100) / 100.f, 0, ((float)((rand() % 200) - 100) / 100.f));
	if (wind == Vector3f(0, 0, 0))
	{
		wind = Vector3f(1, 0, 0);
	}
	wind = normalize(wind);
	m_forces += wind * ((float)(rand() % 100));
}
