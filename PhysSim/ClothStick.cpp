#include "ClothStick.h"
#include "ClothPoint.h"
#include "Play.h"

ClothStick::ClothStick(ClothPoint* leftPoint, ClothPoint* rightPoint)
	: m_leftPoint(leftPoint)
	, m_rightPoint(rightPoint)
{
}

ClothStick::~ClothStick()
{
}

void ClothStick::Update()
{
}

void ClothStick::Render()
{
	Play::DrawLine(m_leftPoint->GetPosition(), m_rightPoint->GetPosition(), Play::cWhite);
}
