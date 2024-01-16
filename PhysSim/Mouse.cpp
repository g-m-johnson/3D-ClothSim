#include "Mouse.h"

void Mouse::Initialise()
{
	Play3d::Input::MouseState mouse = Play3d::Input::GetMouseState();

	m_position =  Play3d::Vector2f(mouse.m_x, mouse.m_y);
	m_prevPos = m_position;
}

void Mouse::Update()
{
	Play3d::Input::MouseState mouse = Play3d::Input::GetMouseState();
	
	m_prevPos = m_position;
	m_position = Play3d::Vector2f(mouse.m_x, mouse.m_y);
}
