#include "Mouse.h"

void Mouse::Initialise()
{
	m_position = Play::GetMousePos();
	m_prevPos = m_position;
}

void Mouse::Update()
{
	m_prevPos = m_position;
	m_position = Play::GetMousePos();
}
