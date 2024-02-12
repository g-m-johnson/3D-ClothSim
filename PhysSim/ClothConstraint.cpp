#include "ClothConstraint.h"
#include "ClothParticle.h"

ClothConstraint::ClothConstraint(ClothParticle* p1, ClothParticle* p2)
	: m_point1(p1)
	, m_point2(p2)
{
	m_restLength = length(p2->GetPosition() - p1->GetPosition());
}

ClothConstraint::~ClothConstraint()
{
}

void ClothConstraint::CalculateSpringForces() const
{
	Vector3f dP = m_point1->GetPosition() - m_point2->GetPosition();
	Vector3f dV = m_point1->GetVelocity() - m_point2->GetVelocity();

	Vector3f f0 = -(m_k * (length(dP) - m_restLength) + m_c * (dot(dV, dP)/length(dP))) * (dP / length(dP)) * 0.01f;
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

void ClothConstraint::DebugDrawConstraints() const
{
	if (m_drawConstraints)
	{
		Graphics::BeginPrimitiveBatch();
		Graphics::DrawLine(m_point1->GetPosition(), m_point2->GetPosition(), Colour::Red);
		Graphics::EndPrimitiveBatch();
	}
}
