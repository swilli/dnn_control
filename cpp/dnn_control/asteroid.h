#ifndef ASTEROID_H
#define ASTEROID_H

#include "vector.h"

class Asteroid
{
    /*
        This class represents the dynamics and gravity of an asteroid shaped as a rigid ellipsoid having different
        rotational properties along its three principal inertia axis (Ix, Iy, Iz).

        For the angular velocity dynamics the analytical solution in terms of Jacobi elliptic functions is used (Implementation
        largely inspired from Landau Lifshitz Mechanics paragrpah 37). We thus assume w_y(-t_bias) = 0. So we only are free to specify two
        more initial conditions and we do so by assigning kinetic energy and angular momentum. As a result the user is not free to specify
        the initial angular velocity, but only the values of the two prime integrals. He does so by defining an angular velocity vector in
        the constructor, this is used only to compute kinetic energy and rotational momentum, the second of its components (w_y) will thus be disregarded
        as 0 will instead be used.

        For the asteroid gravity, we use the implementation of Dario Cersosimo who kindly sent us his matlab files.
    */
public:
    Asteroid(const Vector3D &semi_axis, const double &density, const Vector3D &angular_velocity, const double &time_bias);

    void GravityAtPosition(const Vector3D &position, Vector3D &gravity) const;
    void AngularVelocityAndAccelerationAtTime(const double &time, Vector3D &velocity, Vector3D &acceleration) const;
    void NearestPointOnSurfaceToPosition(const Vector3D &position, Vector3D &point, double *distance) const;

    double SemiAxis(const int &dimension) const;
    double Inertia(const int &dimension) const;

private:
    void NearestPointOnEllipsoidFirstQuadrant(const Vector3D &position, Vector3D &point) const;
    void NearestPointOnEllipseFirstQuadrant(const Vector2D &semi_axis, const Vector2D &position, Vector2D &point) const;

    double mass_;
    double density_;
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
