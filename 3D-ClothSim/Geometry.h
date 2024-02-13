#pragma once
#include "../Play3d/Play3d.h"
using namespace Play3d;

namespace Geometry
{
	struct Ray
	{
		Vector3f start;
		Vector3f end;
	};

	struct Plane
	{
		Vector3f normal;
		float d;
	};

	struct Quad
	{
		Vector3f a;
		Vector3f b;
		Vector3f c;
		Vector3f d;
	};
}
