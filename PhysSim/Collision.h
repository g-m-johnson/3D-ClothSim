#pragma once
#include "Geometry.h"

namespace Collision
{

	bool IntersectRayPlane(Geometry::Ray r, Geometry::Plane p, Vector3f& intersection);

	bool IntersectRayQuad(Geometry::Ray r, Geometry::Quad q, Vector3f& intersection);

};

