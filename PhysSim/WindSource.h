#pragma once
#include "../Play3d/Play3d.h"
using namespace Play3d;


class WindSource
{
public:

	Vector3f CalculateForceAtPoint(Vector3f p);

	const Vector3f GetSourcePos() const { return m_sourcePos; }
	void SetSourcePos(Vector3f pos) { m_sourcePos = pos; }

	const Vector3f GetWindForce () const { return m_force; }
	void SetWindForce(Vector3f force) { m_force = force; }

private:

	Vector3f m_force;
	Vector3f m_sourcePos;
};

