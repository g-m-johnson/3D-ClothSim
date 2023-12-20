#include "ClothPoint.h"

ClothPoint::ClothPoint(Vector2f pos)
	: m_position(pos)
{
}

ClothPoint::~ClothPoint()
{
}

void ClothPoint::Update()
{

}

void ClothPoint::Render()
{
	Play::DrawSprite("ClothPoint", m_position, 0);
}
