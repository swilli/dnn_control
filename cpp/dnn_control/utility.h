#ifndef UTILITY_H
#define UTILITY_H

#include "vector.h"

class UtilityException {};
class PositionInsideEllipsoidException : public UtilityException {};
class PositionInsideEllipseException : public UtilityException {};

//double BisectEllipsoid(const Vector3D &semi_axis_mul_pos, const Vector3D &semi_axis_pow2);

//double BisectEllipse(const Vector2D &semi_axis_mul_pos, const Vector2D &semi_axis_pow2);

// Required for asteroid NearestPointOnEllipsoidFirstQuadrant
double NewtonRaphsonEllipsoid(const Vector3D &semi_axis_mul_pos, const Vector3D &semi_axis_pow2);

// Required for asteroid NearestPointOnEllipseFirstQuadrant
double NewtonRaphsonEllipse(const Vector2D &semi_axis_mul_pos, const Vector2D &semi_axis_pow2);

#endif // UTILITY_H
