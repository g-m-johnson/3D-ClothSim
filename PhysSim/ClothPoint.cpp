#include "PhysSimMain.h"
#include "Cloth.h"
#include "ClothPoint.h"
#include "ClothStick.h"
#include "Mouse.h"

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
	Vector2f cursorToPoint = m_position - Mouse::Instance().GetMousePos();
	float cursorSize = Mouse::Instance().GetMouseCursorSize();
	bool isSelected = lengthSqr(cursorToPoint) < (cursorSize * cursorSize);

	for (ClothStick* stick : m_sticks)
	{
		if (stick != nullptr)
		{
			stick->SetIsSelected(isSelected);
		}
	}

	float e = m_cloth->GetElasticity();

	if (Play::GetMouseButton(Play::LEFT) && isSelected)
	{
		Vector2f diff = Mouse::Instance().GetMousePos() - Mouse::Instance().GetMousePrevPos();

		if (diff.x > e)
		{
			diff.x = e;
		}
		if (diff.y > e)
		{
			diff.y = e;
		}
		if (diff.x < -e)
		{
			diff.x = -e;
		}
		if (diff.y < -e)
		{
			diff.y = -e;
		}
		m_prevPos = m_position - diff;
	}

	if (Play::GetMouseButton(Play::RIGHT) && isSelected)
	{
		for (ClothStick* stick : m_sticks)
		{
			if (stick != nullptr)
			{
				stick->SetIsActive(false);
			}
		}
	}
	
	if (m_isPinned)
	{
		m_position = m_initPos;
	}

	Vector2f g = m_cloth->GetGravity();
	float d = m_cloth->GetDrag();

	Vector2f newPos = m_position + (m_position - m_prevPos) * (1.0f - d) + g * (1.0f - d) * dT * dT;
	m_prevPos = m_position;
	m_position = newPos;

}

void ClothPoint::AddStick(ClothStick* stick, int index)
{
	m_sticks[index] = stick;
}
