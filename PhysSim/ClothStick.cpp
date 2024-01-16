#include "ClothStick.h"
#include "ClothPoint.h"
#include "../Play3d/Play3d.h"

ClothStick::ClothStick(ClothPoint* leftPoint, ClothPoint* rightPoint)
	: m_leftPoint(leftPoint)
	, m_rightPoint(rightPoint)
{
	m_length = length(rightPoint->GetPosition() - leftPoint->GetPosition());
}

ClothStick::~ClothStick()
{
}

void ClothStick::Update()
{
	if (!m_isActive)
	{
		return;
	}

	Vector3f p0Pos = m_leftPoint->GetPosition();
	Vector3f p1Pos = m_rightPoint->GetPosition();

	Vector3f diff = p0Pos - p1Pos;
	float dist = sqrtf(diff.x * diff.x + diff.y * diff.y);
	float diffFactor = (m_length - dist) / dist;
	Vector3f offset = diff * diffFactor * 0.5f;

	m_leftPoint->SetPosition(Vector3f(p0Pos.x + offset.x, p0Pos.y + offset.y, 0));
	m_rightPoint->SetPosition(Vector3f(p1Pos.x - offset.x, p1Pos.y - offset.y, 0));
}

void ClothStick::Render()
{
	if (!m_isActive)
	{
		return;
	}
	Play3d::Graphics::DrawLine(m_leftPoint->GetPosition(), m_rightPoint->GetPosition(), Play3d::Colour::White);
}