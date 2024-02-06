#pragma once
#include "../Play3d/Play3d.h"
using namespace Play3d;

namespace Collision
{

	bool IntersectRayTriangle(Vector3f p, Vector3f d, Vector3f A, Vector3f B, Vector3f C, Vector3f& i);

};

