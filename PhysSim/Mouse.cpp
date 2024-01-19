#include "Mouse.h"

void Mouse::Initialise()
{
	Input::MouseState mouse = Input::GetMouseState();

	m_position = Vector2f(mouse.m_x, mouse.m_y);
	m_prevPos = m_position;
}

void Mouse::Update()
{
	Input::MouseState mouse = Input::GetMouseState();
	
	m_prevPos = m_position;
	m_position = Vector2f(mouse.m_x, mouse.m_y);
}

Vector3f Mouse::GetMouseRaycast()
{
	Input::MouseState m = Input::GetMouseState();
	Graphics::SurfaceSize surface = Graphics::GetDisplaySurfaceSize();

	float x = (2.0f * m.m_x) / surface.m_width - 1.0f;
	float y = 1.0f - (2.0f * m.m_y) / surface.m_height;
	float z = 1.0f;

	Vector3f ray_nds = Vector3f(x, y, z);
	Vector4f ray_clip = Vector4f(ray_nds.x, ray_nds.y, -1.0f, 1.0f);

	//Vector3f camForward = Demo::GetDebugCameraForwardVector();

	return Vector3f();
}
