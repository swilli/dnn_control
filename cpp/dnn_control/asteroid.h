#ifndef ASTEROID_H
#define ASTEROID_H

#include "vector.h"

class Asteroid
{
public:
    Asteroid(const Vector3D &semi_axis, const double &density, const Vector3D &angular_velocity, const double &time_bias);

    void GravityAtPosition(const Vector3D &position, Vector3D &gravity) const;
    void AngularVelocityAndAccelerationAtTime(const double &time, Vector3D &velocity, Vector3D &acceleration) const;
    void NearestPointOnSurface(const Vector3D &position, Vector3D &point, double *distance) const;

    double SemiAxis(const int &dimension) const;
    double Inertia(const int &dimension) const;

private:
    void NearestPointOnEllipsoidFirstQuadrant(const Vector3D &position, Vector3D &point) const;
    void NearestPointOnEllipseFirstQuadrant(const Vector2D &semi_axis, const Vector2D &position, Vector2D &point) const;

    double mass_;
    double density_;
    Vector3D angular_velocity_;
    Vector3D semi_axis_;
    Vector3D semi_axis_pow2_;
    Vector3D inertia_;
    Vector3D inertia_pow2_;
    double time_bias_;
    double energy_mul2_;
    double momentum_pow2_;
    double gamma_;
    Vector3D elliptic_coefficients_;
    double elliptic_tau_;
    double elliptic_modulus_;

    bool inversion_;
};

#endif // ASTEROID_H
