#ifndef UTILITY_H
#define UTILITY_H

#include "vector.h"

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

// X ~ U(minimum, maximum)
double SampleUniform(const double &minimum, const double &maximum);

// X ~ U({-1,1})
double SampleSign();

// Returns a point point, whereas
// semi_axis_[0] * cos(u) * sin(v) < point[0] < semi_axis[0] * band_width_scale * cos(u) * sin(v)
// semi_axis_[1] * sin(u) * sin(v) < point[1] < semi_axis[1] * band_width_scale * sin(u) * sin(v)
// semi_axis_[2] * cos(v)          < point[2] < semi_axis[1] * band_width_scale * cos(v)
void SamplePointOutSideEllipsoid(const Vector3D &semi_axis, const double &band_width_scale, Vector3D &point);

#endif // UTILITY_H
