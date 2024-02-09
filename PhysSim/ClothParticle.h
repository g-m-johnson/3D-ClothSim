#pragma once
#include "../Play3d/Play3d.h"
using namespace Play3d;

class Cloth;
class ClothConstraint;

class ClothParticle
{
public:
	ClothParticle(Cloth* cloth, Vector3f pos);
	~ClothParticle();

	void VerletIntegration();

	void AddStick(ClothConstraint* stick, int index);

	void CalculateForces();

	void ApplyExternalForce(Vector3f force);

	Vector3f GetPosition() { return m_position; }
	void SetPosition(Vector3f pos) { m_position = pos; }

	bool GetIsPinned() { return m_isPinned; }
	void SetIsPinned(bool isStatic) { m_isPinned = isStatic; }

	bool GetIsCorner() { return m_isCorner; }
	void SetIsCorner(bool isCorner) { m_isCorner = isCorner; }

	Vector3f& GetForceVector() { return m_forces; }
	void ZeroForceVector() { m_forces = Vector3f(0, 0, 0); }

	float GetMass() { return m_mass; }
	void SetMass(float m) { m_mass = m; }

	Vector3f GetVelocity() { return m_velocity; }
	void SetVelocity(Vector3f v) { m_velocity = v; }
private:
	Vector3f m_position;
	Vector3f m_prevPos;
	Vector3f m_initPos;

	Vector3f m_forcesExt;
	Vector3f m_forces{ 0, 0, 0 };
	Vector3f m_velocity{ 0, 0, 0 };
	Vector3f m_acceleration{ 0, 0, 0 };

	Cloth* m_cloth;
	ClothConstraint* m_sticks[2] = {nullptr};

	float m_mass;

	bool m_isPinned = false;
	bool m_isCorner = false;
};

