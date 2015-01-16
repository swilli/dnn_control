#ifndef UTILITY_H
#define UTILITY_H

#include "vector.h"

class UtilityException {};
class PositionInsideEllipsoidException : public UtilityException {};
class PositionInsideEllipseException : public UtilityException {};

// c = u x v
inline Vector3D CrossProduct(const Vector3D &vector_u, const Vector3D &vector_v) {
    Vector3D result = {vector_u[1] * vector_v[2] - vector_u[2] * vector_v[1],
                       vector_u[2] * vector_v[0] - vector_u[0] * vector_v[2],
                       vector_u[0] * vector_v[1] - vector_u[1] * vector_v[0]};
    return result;
}

// Required for asteroid NearestPointOnEllipsoidFirstQuadrant
double BisectEllipsoid(const Vector3D &semi_axis_mul_pos, const Vector3D &semi_axis_pow2);

// Required for asteroid NearestPointOnEllipseFirstQuadrant
double BisectEllipse(const Vector2D &semi_axis_mul_pos, const Vector2D &semi_axis_pow2);


#endif // UTILITY_H
