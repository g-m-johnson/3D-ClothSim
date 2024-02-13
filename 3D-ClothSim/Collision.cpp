#include "Collision.h"

bool Collision::IntersectRayPlane(Geometry::Ray r, Geometry::Plane p, Vector3f& intersection)
{
    Vector3f AB = r.end - r.start;
    float t = (p.d - dot(p.normal, r.start)) / dot(p.normal, AB);

    if (t >= 0.0f && t <= 1.0f)
    {
        intersection = r.start + t * AB;
        return true;
    }

    return false;
}

bool Collision::IntersectRayQuad(Geometry::Ray r, Geometry::Quad q, Vector3f& intersection)
{
    Vector3f line = r.end - r.start;
    Vector3f OA = q.a - r.start;
    Vector3f OB = q.b - r.start;
    Vector3f OC = q.c - r.start;

    Vector3f m = cross(OC, line);
    float v = dot(OA, m);
    if (v >= 0.0f)
    {
        float u = -dot(OB, m);
        if(u < 0.0f) return false;
        float w = scalarTriple(line, OB, OA);
        if(w < 0.0f) return false;

        float denom = 1.0f / (u + v + w);
        u *= denom;
        v *= denom;
        w *= denom;
        intersection = u*q.a + v*q.b + w*q.c;
    }
    else
    {
        Vector3f OD = q.d - r.start;
        float u = dot(OD, m);
        if(u < 0.0f) return false;
        float w = scalarTriple(line, OA, OD);
        if(w < 0.0f) return false;
        v = -v;

        float denom = 1.0f / (u + v + w);
		u *= denom;
		v *= denom;
		w *= denom;
        intersection = u*q.a + v*q.d + w*q.c;
    }
    return true;
}
