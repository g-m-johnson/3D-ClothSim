#include "WindSource.h"

Vector3f WindSource::CalculateForceAtPoint(Vector3f p)
{
    // Get distance between point and wind source
    Vector3f AB = m_sourcePos - p;

    // Want a radial falloff distance but no falloff in the direction of force

    return Vector3f();
}
