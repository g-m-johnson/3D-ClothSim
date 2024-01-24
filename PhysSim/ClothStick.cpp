#include "ClothStick.h"
#include "ClothPoint.h"
#include "../Play3d/Play3d.h"

ClothSpring::ClothSpring(ClothParticle* p1, ClothParticle* p2)
	: m_point1(p1)
	, m_point2(p2)
{
	m_restLength = length(p2->GetPosition() - p1->GetPosition());
}

ClothSpring::~ClothSpring()
{
}

void ClothSpring::Update()
{
	if (!m_isActive)
	{
		return;
	}

	Vector3f p0Pos = m_point1->GetPosition();
	Vector3f p1Pos = m_point2->GetPosition();

	Vector3f diff = p0Pos - p1Pos;
	float dist = sqrtf(diff.x * diff.x + diff.y * diff.y);
	float diffFactor = (m_restLength - dist) / dist;
	Vector3f offset = diff * diffFactor * 0.5f;

	m_point1->SetPosition(Vector3f(p0Pos.x + offset.x, p0Pos.y + offset.y, 0));
	m_point2->SetPosition(Vector3f(p1Pos.x - offset.x, p1Pos.y - offset.y, 0));
}

void ClothSpring::Render()
{
	if (!m_isActive)
	{
		return;
	}
	Play3d::Graphics::DrawLine(m_point1->GetPosition(), m_point2->GetPosition(), Play3d::Colour::White);
}

void ClothSpring::CalculateSpringForces()
{
	if (!m_isActive)
	{
		return;
	}

	Vector3f dP = m_point1->GetPosition() - m_point2->GetPosition();
	Vector3f dV = m_point1->GetVelocity() - m_point2->GetVelocity();

	Vector3f f0 = -(m_k * (length(dP) - m_restLength) + m_c * (dot(dV, dP)/length(dP))) * (dP / length(dP));
	Vector3f f1 = -f0;

	if (!m_point1->GetIsPinned())
	{
		m_point1->GetForceVector() += f0;
	}
	if (!m_point2->GetIsPinned())
	{
		m_point2->GetForceVector() += f1;
	}
}
