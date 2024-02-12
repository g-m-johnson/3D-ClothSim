#pragma once
#include "../Play3d/Play3d.h"
using namespace Play3d;
#include "PhysSimMain.h"

class Cloth;
class ClothConstraint;

class ClothParticle
{
public:
	ClothParticle(Cloth* cloth, Vector3f pos);
	~ClothParticle();

	void VerletIntegration();

	void CalculateForces();
	void ApplyExternalForce(const Vector3f force) { m_forcesExt = force; }


	const Vector3f GetPosition() const { return m_position; }

	const bool GetIsPinned() const { return m_isPinned; }
	void SetIsPinned(const bool isStatic) { m_isPinned = isStatic; }

	Vector3f& GetForceVector() { return m_forces; }
	void ZeroForceVector() { m_forces = Vector3f(0, 0, 0); }

	const Vector3f GetVelocity() const { return m_velocity; }
	void SetVelocity(const Vector3f v) { m_velocity = v; }

private:

	Vector3f m_forcesExt;
	Vector3f m_forces{ 0, 0, 0 };
	
	Vector3f m_position;
	Vector3f m_velocity{ 0, 0, 0 };
	Vector3f m_acceleration{ 0, 0, 0 };
	Vector3f m_prevAccn{ 0, 0, 0 };

	Cloth* m_cloth; // Making this const ptr breaks sim?? 

	const float m_mass = PARTICLE_MASS;

	bool m_isPinned = false;
};

