#include "ClothStick.h"
#include "ClothPoint.h"
#include "Play.h"

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

	Vector2f p0Pos = m_leftPoint->GetPosition();
	Vector2f p1Pos = m_rightPoint->GetPosition();

	Vector2f diff = p0Pos - p1Pos;
	float dist = sqrtf(diff.x * diff.x + diff.y * diff.y);
	float diffFactor = (m_length - dist) / dist;
	Vector2f offset = diff * diffFactor * 0.5f;

	m_leftPoint->SetPosition(Vector2f(p0Pos.x + offset.x, p0Pos.y + offset.y));
	m_rightPoint->SetPosition(Vector2f(p1Pos.x - offset.x, p1Pos.y - offset.y));
}

void ClothStick::Render()
{
	if (!m_isActive)
	{
		return;
	}

	Play::DrawLine(m_leftPoint->GetPosition(), m_rightPoint->GetPosition(), Play::cWhite);
}
