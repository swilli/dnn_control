#ifndef UTILITY_H
#define UTILITY_H

#include "vector.h"

inline void CrossProduct(const Vector3D &vector_u, const Vector3D &vector_v, Vector3D &crossed) {
    crossed[0] = vector_u[1] * vector_v[2] - vector_u[2] * vector_v[1];
    crossed[1] = vector_u[2] * vector_v[0] - vector_u[0] * vector_v[2];
    crossed[2] = vector_u[0] * vector_v[1] - vector_u[1] * vector_v[0];
}

double BisectEllipsoid(const Vector3D &semi_axis_mul_pos, const Vector3D &semi_axis_pow2, const double &tolerance);

double BisectEllipse(const Vector2D &semi_axis_mul_pos, const Vector2D &semi_axis_pow2, const double &tolerance);

double SampleUniform(const double &minimum, const double &maximum);

double SampleSign();

void SamplePointOutSideEllipsoid(const Vector3D &semi_axis, const double &band_width_scale, Vector3D &point);

#endif // UTILITY_H
