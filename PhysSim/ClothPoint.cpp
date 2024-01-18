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
{
}

ClothPoint::~ClothPoint()
{
}

void ClothPoint::Update()
{
	float dT = System::GetDeltaTime();

	Vector3f cursorToPoint = m_position - Vector3f(Mouse::Instance().GetMousePos(), 0);
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

	if (Play3d::Input::GetMouseState().m_leftButton && isSelected)
	{
		float diffX = Play3d::Input::GetMouseState().m_deltaX;
		float diffY = Play3d::Input::GetMouseState().m_deltaY;

		if (diffX > e)
		{
			diffX = e;
		}
		if (diffY > e)
		{
			diffY = e;
		}
		if (diffX < -e)
		{
			diffX = -e;
		}
		if (diffY < -e)
		{
			diffY = -e;
		}

		Vector3f diff = Vector3f(diffX, diffY, 0);
		m_prevPos = m_position - diff;
	}

	
	if (Play3d::Input::GetMouseState().m_rightButton && isSelected)
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

	Vector3f g = m_cloth->GetGravity();
	float d = m_cloth->GetDrag();

	Vector3f newPos = m_position + (m_position - m_prevPos) * (1.0f - d) + g * (1.0f - d) * dT * dT;
	m_prevPos = m_position;
	m_position = newPos;

}

void ClothPoint::AddStick(ClothStick* stick, int index)
{
	m_sticks[index] = stick;
}
