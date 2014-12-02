#ifndef ASTEROID_H
#define ASTEROID_H

class Asteroid
{
public:
    Asteroid(const double *semi_axis, const double &density, const double *angular_velocity, const double &time_bias);

    void GravityAtPosition(double *position, double *gravity) const;
    void AngularVelocityAndAccelerationAtTime(const double &time, double *velocity, double *acceleration) const;
    void NearestPointOnSurface(const double *position, double *point, double *distance) const;

private:
    void NearestPointOnEllipsoidFirstQuadrant(const double *position, double *point) const;
    void NearestPointOnEllipseFirstQuadrant(const double *semi_axis, const double *position, double *point) const;

    double mass_;
    double density_;
    double angular_velocity_[3];
    double semi_axis_[3];
    double semi_axis_pow2_[3];
    double inertia_[3];
    double inertia_pow2_[3];
    double time_bias_;
    double energy_mul2_;
    double momentum_pow2_;
    double gamma_;
    double elliptic_coefficients_[3];
    double elliptic_tau_;
    double elliptic_modulus_;

    bool inversion_;
};

#endif // ASTEROID_H
