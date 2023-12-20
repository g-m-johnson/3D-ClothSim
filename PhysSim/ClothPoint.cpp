#include "ClothPoint.h"
#include "ClothStick.h"
#include "Cloth.h"

ClothPoint::ClothPoint(Cloth* cloth, Vector2f pos)
	: m_cloth(cloth)
	, m_position(pos)
	, m_prevPos(pos)
	, m_initPos(pos)
{
}

ClothPoint::~ClothPoint()
{
}

void ClothPoint::Update(float dT)
{
	if (m_isPinned)
	{
		m_position = m_initPos;
	}
	else
	{
		Vector2f g = m_cloth->GetGravity();
		float d = m_cloth->GetDrag();
		float e = m_cloth->GetElasticity();
		
		Vector2f newPos = m_position + (m_position - m_prevPos) * (1.0f - d) + g * (1.0f - d) * dT * dT;
		m_prevPos = m_position;
		m_position = newPos;
	}
}

void ClothPoint::Render()
{
	Play::DrawSprite("ClothPoint", m_position, 0);
}

void ClothPoint::AddStick(ClothStick* stick, int index)
{
	m_sticks[index] = stick;
}
