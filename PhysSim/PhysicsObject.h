#pragma once
#include "Play.h"

class PhysicsObject
{
public:

	const Vector2f GetPosition() const { return m_position; }
	void SetPosition(Vector2f pos) { m_position = pos; }

	const Vector2f GetVelocity() const { return m_velocity; }
	void SetVelocity(Vector2f vel) { m_velocity = vel; }

private:
	Vector2f m_position;
	Vector2f m_velocity;

};

