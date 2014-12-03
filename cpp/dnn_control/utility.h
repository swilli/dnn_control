#ifndef UTILITY_H
#define UTILITY_H

#include "vector.h"

class Functor {
public:
    virtual double Evaluate(double value) {
        return value;
    }
};

inline void CrossProduct(const Vector3D &vector_u, const Vector3D &vector_v, Vector3D &crossed) {
    crossed[0] = vector_u[1] * vector_v[2] - vector_u[2] * vector_v[1];
    crossed[1] = vector_u[2] * vector_v[0] - vector_u[0] * vector_v[2];
    crossed[2] = vector_u[0] * vector_v[1] - vector_u[1] * vector_v[0];
}

double Bisection(Functor *container, const double &minimum, const double &maximum, const int &max_iterations, const double &tolerance);
void SamplePointOutSideEllipsoid(const Vector3D &semi_axis, const double &band_width_scale, Vector3D &point);

#endif // UTILITY_H
