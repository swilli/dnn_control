#include "asteroid.h"
#include <gsl/gsl_poly.h>
#include <gsl/gsl_sf_ellint.h>
#include <gsl/gsl_sf_elljac.h>
#include <math.h>
#include "constants.h"
#include "utility.h"

Asteroid::Asteroid(const Vector3D &semi_axis, const double &density, const Vector3D &angular_velocity, const double &time_bias)
{
    time_bias_ = time_bias;

    density_ = density;

    mass_ = 4.0 / 3.0 * k_pi * density;
    for (int i = 0; i < 3; ++i) {
        angular_velocity_[i] = angular_velocity[i];
        semi_axis_[i] = semi_axis[i];
        semi_axis_pow2_[i] = semi_axis_[i] * semi_axis_[i];
        mass_ *= semi_axis_[i];
    }

    Vector3D signs;
    signs[0] = (angular_velocity_[0] > 0 ? 1.0 : -1.0);
    signs[1] = (angular_velocity_[0] * angular_velocity_[2] > 0 ? 1.0 : -1.0);
    signs[2] = (angular_velocity_[2] > 0 ? 1.0 : -1.0);

    inertia_[0] = 0.2 * mass_ * (semi_axis_pow2_[1] + semi_axis_pow2_[2]);
    inertia_[1] = 0.2 * mass_ * (semi_axis_pow2_[0] + semi_axis_pow2_[2]);
    inertia_[2] = 0.2 * mass_ * (semi_axis_pow2_[0] + semi_axis_pow2_[1]);

    gamma_ = 4.0 * k_pi * k_gravitational_constant * density;
    energy_mul2_ = 0.0;
    momentum_pow2_ = 0.0;
    Vector3D inertia;
    for (int i = 0; i < 3; ++i) {
        inertia[i] = inertia_[i];
        inertia_pow2_[i] = inertia_[i] * inertia_[i];
        gamma_ *= semi_axis_[i];
        energy_mul2_ += inertia_[i] * angular_velocity_[i] * angular_velocity_[i];
        momentum_pow2_ += inertia_[i] * inertia_[i] * angular_velocity_[i] * angular_velocity_[i];
    }
    gamma_ /= sqrt(semi_axis_pow2_[0] - semi_axis_pow2_[2]);

    inversion_ = false;
    if (momentum_pow2_ < energy_mul2_ * inertia_[1]) {
        inversion_ = true;
        inertia[2] = inertia_[0];
        inertia[0] = inertia_[2];
        const double tmp = signs[0];
        signs[0] = signs[2];
        signs[2] = tmp;
    }

    elliptic_coefficients_[0] = signs[0] * sqrt((energy_mul2_ * inertia[2] - momentum_pow2_) / (inertia[0] * (inertia[2] - inertia[0])));
    elliptic_coefficients_[1] = signs[1] * sqrt((energy_mul2_ * inertia[2] - momentum_pow2_) / (inertia[1] * (inertia[2] - inertia[1])));
    elliptic_coefficients_[2] = signs[2] * sqrt((momentum_pow2_ - energy_mul2_ * inertia[0]) / (inertia[2] * (inertia[2] - inertia[0])));

    elliptic_tau_ = sqrt((inertia[2] - inertia[1]) * (momentum_pow2_ - energy_mul2_ * inertia[0]) / (inertia[0] * inertia[1] * inertia[2]));

    elliptic_modulus_ = (inertia[1] - inertia[0]) * (energy_mul2_ * inertia[2] - momentum_pow2_) / ((inertia[2] - inertia[1]) * (momentum_pow2_ - energy_mul2_ * inertia[0]));
}

double Asteroid::SemiAxis(const int &dimension) const
{
    return semi_axis_[dimension];
}

double Asteroid::Inertia(const int &dimension) const
{
    return inertia_[dimension];
}

void Asteroid::GravityAtPosition(const Vector3D &position, Vector3D &gravity) const
{
    gravity[0] = 0.0;
    gravity[1] = 0.0;
    gravity[2] = 0.0;

    const double pos_x_pow2 = position[0] * position[0];
    const double pos_y_pow2 = position[1] * position[1];
    const double pos_z_pow2 = position[2] * position[2];

    const double coef_2 = -(pos_x_pow2 + pos_y_pow2 + pos_z_pow2 - semi_axis_pow2_[0] - semi_axis_pow2_[1] - semi_axis_pow2_[2]);
    const double coef_1 = -(semi_axis_pow2_[1] * pos_x_pow2 + semi_axis_pow2_[2] * pos_x_pow2
            + semi_axis_pow2_[0] * pos_y_pow2 + semi_axis_pow2_[2] * pos_y_pow2
            + semi_axis_pow2_[0] * pos_z_pow2 + semi_axis_pow2_[1] * pos_z_pow2
            - semi_axis_pow2_[0] * semi_axis_pow2_[2] - semi_axis_pow2_[1] * semi_axis_pow2_[2] - semi_axis_pow2_[0] * semi_axis_pow2_[1]);
    const double coef_0 = -(semi_axis_pow2_[1] * semi_axis_pow2_[2] * pos_x_pow2
            + semi_axis_pow2_[0] * semi_axis_pow2_[2] * pos_y_pow2
            + semi_axis_pow2_[0] * semi_axis_pow2_[1] * pos_z_pow2
            - semi_axis_pow2_[0] * semi_axis_pow2_[1] * semi_axis_pow2_[2]);

    double root_1 = 0.0, root_2 = 0.0, root_3 = 0.0;
    double kappa = 0.0;
    const int num_roots = gsl_poly_solve_cubic(coef_2, coef_1, coef_0, &root_1, &root_2, &root_3);
    if(num_roots == 1) {
        kappa = root_1;
    } else {
        kappa = root_3;
    }

    const double phi = asin(sqrt((semi_axis_pow2_[0] - semi_axis_pow2_[2]) / (kappa + semi_axis_pow2_[0])));
    const double k = sqrt((semi_axis_pow2_[0] - semi_axis_pow2_[1]) / (semi_axis_pow2_[0] - semi_axis_pow2_[2]));

    const double integral_F = gsl_sf_ellint_F(phi,k,0);
    const double integral_E = gsl_sf_ellint_E(phi,k,0);

    const double delta = sqrt((semi_axis_pow2_[0] - semi_axis_pow2_[2]) / ((semi_axis_pow2_[0] + kappa) * (semi_axis_pow2_[1] + kappa) * (semi_axis_pow2_[2] + kappa)));

    gravity[0] = gamma_ / (semi_axis_pow2_[0] - semi_axis_pow2_[1]) * (integral_E - integral_F);
    gravity[1] = gamma_ * ((-semi_axis_pow2_[0] + semi_axis_pow2_[2]) * integral_E / ((semi_axis_pow2_[0] - semi_axis_pow2_[1]) * (semi_axis_pow2_[1] - semi_axis_pow2_[2]))
            + integral_F / (semi_axis_pow2_[0] - semi_axis_pow2_[1]) + delta * (semi_axis_pow2_[2] + kappa) / (semi_axis_pow2_[1] - semi_axis_pow2_[2]));
    gravity[2] = gamma_ / (semi_axis_pow2_[2] - semi_axis_pow2_[1]) * (-integral_E + (semi_axis_pow2_[1] + kappa) * delta);

    gravity[0] *= position[0];
    gravity[1] *= position[1];
    gravity[2] *= position[2];
}

void Asteroid::AngularVelocityAndAccelerationAtTime(const double &time, Vector3D &velocity, Vector3D &acceleration) const
{
    const double t = (time + time_bias_) * elliptic_tau_;

    double sn_tau = 0.0, cn_tau = 0.0, dn_tau = 0.0;
    gsl_sf_elljac_e(t,elliptic_modulus_,&sn_tau, &cn_tau, & dn_tau);

    if (inversion_) {
        velocity[0] = elliptic_coefficients_[2] * dn_tau;
        velocity[1] = elliptic_coefficients_[1] * sn_tau;
        velocity[2] = elliptic_coefficients_[0] * cn_tau;
    } else {
        velocity[0] = elliptic_coefficients_[0] * cn_tau;
        velocity[1] = elliptic_coefficients_[1] * sn_tau;
        velocity[2] = elliptic_coefficients_[2] * dn_tau;
    }

    acceleration[0] = (inertia_[1] - inertia_[2]) * velocity[1] * velocity[2] / inertia_[0];
    acceleration[1] = (inertia_[2] - inertia_[0]) * velocity[2] * velocity[0] / inertia_[1];
    acceleration[2] = (inertia_[0] - inertia_[1]) * velocity[0] * velocity[1] / inertia_[2];
}

void Asteroid::NearestPointOnSurface(const Vector3D &position, Vector3D &point, double *distance) const
{
    Vector3D signs;
    Vector3D abs_position;
    for (int i = 0; i < 3; ++i) {
        point[i] = 0.0;
        signs[i] = (position[i] >= 0.0 ? 1.0 : -1.0);
        abs_position[i] = abs(position[i]);
    }

    NearestPointOnEllipsoidFirstQuadrant(abs_position, point);

    point[0] *= signs[0];
    point[1] *= signs[1];
    point[2] *= signs[2];

    double result = 0.0;
    for(int i = 0; i < 3; ++i) {
        result += (point[i] - position[i]) * (point[i] - position[i]);
    }

    *distance = sqrt(result);
}

void Asteroid::NearestPointOnEllipsoidFirstQuadrant(const Vector3D &position, Vector3D &point) const
{
    point[0] = 0.0; point[1] = 0.0; point[2] = 0.0;

    if (position[2] > 0.0) {
        if (position[1] > 0.0) {
            if (position[0] > 0.0) {
                Vector3D semi_axis_mul_pos = {semi_axis_[0] * position[0], semi_axis_[1] * position[1], semi_axis_[2] * position[2]};
                const double time = BisectEllipsoid(semi_axis_mul_pos, semi_axis_pow2_, 1e-15);
                point[0] = semi_axis_pow2_[0] * position[0] / (time + semi_axis_pow2_[0]);
                point[1] = semi_axis_pow2_[1] * position[1] / (time + semi_axis_pow2_[1]);
                point[2] = semi_axis_pow2_[2] * position[2] / (time + semi_axis_pow2_[2]);
            } else {
                point[0] = 0.0;
                const Vector2D semi_axis_2d = {semi_axis_[1], semi_axis_[2]};
                const Vector2D position_2d = {position[1], position[2]};
                Vector2D point_2d;
                NearestPointOnEllipseFirstQuadrant(semi_axis_2d, position_2d, point_2d);
                point[1] = point_2d[0];
                point[2] = point_2d[1];
            }
        } else {
            point[1] = 0.0;
            if (position[0] > 0.0) {
                const Vector2D semi_axis_2d = {semi_axis_[0], semi_axis_[2]};
                const Vector2D position_2d = {position[0], position[2]};
                Vector2D point_2d;
                NearestPointOnEllipseFirstQuadrant(semi_axis_2d, position_2d, point_2d);
                point[0] = point_2d[0];
                point[2] = point_2d[1];
            } else {
                point[0] = 0.0;
                point[2] = semi_axis_[2];
            }
        }
    } else {
        const Vector2D denominator = {semi_axis_pow2_[0] - semi_axis_pow2_[2], semi_axis_pow2_[1] - semi_axis_pow2_[2]};
        const Vector2D semi_axis_mul_pos = {semi_axis_[0] * position[0], semi_axis_[1] * position[1]};

        if (semi_axis_mul_pos[0] < denominator[0] && semi_axis_mul_pos[1] < denominator[1]) {
            const Vector2D semi_axis_div_denom = {semi_axis_mul_pos[0] / denominator[0] ,
                                                   semi_axis_mul_pos[1] / denominator[1]};
            const Vector2D semi_axis_div_denom_pow2 = {semi_axis_div_denom[0] * semi_axis_div_denom[0],
                                                        semi_axis_div_denom[1] * semi_axis_div_denom[1]};
            const double discr = 1.0 - semi_axis_div_denom_pow2[0] - semi_axis_div_denom_pow2[1];
            if (discr > 0.0) {
                point[0] = semi_axis_[0] * semi_axis_div_denom[0];
                point[1] = semi_axis_[1] * semi_axis_div_denom[1];
                point[2] = semi_axis_[2] * sqrt(discr);
            } else {
                point[2] = 0.0;
                const Vector2D semi_axis_2d = {semi_axis_[0], semi_axis_[1]};
                const Vector2D position_2d = {position[0], position[1]};
                Vector2D point_2d;
                NearestPointOnEllipseFirstQuadrant(semi_axis_2d, position_2d, point_2d);
                point[0] = point_2d[0];
                point[1] = point_2d[1];
            }
        } else {
            point[2] = 0.0;
            const Vector2D semi_axis_2d = {semi_axis_[0], semi_axis_[1]};
            const Vector2D position_2d = {position[0], position[1]};
            Vector2D point_2d;
            NearestPointOnEllipseFirstQuadrant(semi_axis_2d, position_2d, point_2d);
            point[0] = point_2d[0];
            point[1] = point_2d[1];
        }
    }
}

void Asteroid::NearestPointOnEllipseFirstQuadrant(const Vector2D &semi_axis, const Vector2D &position, Vector2D &point) const
{
    point[0] = 0.0; point[1] = 0.0;

    const Vector2D semi_axis_pow2 = {semi_axis[0] * semi_axis[0], semi_axis[1] * semi_axis[1]};

    if (position[1] > 0.0) {
        if (position[0] > 0.0) {
            Vector2D semi_axis_mul_pos = {semi_axis_[0] * position[0], semi_axis_[1] * position[1]};
            const double time = BisectEllipse(semi_axis_mul_pos, semi_axis_pow2, 1e-15);
            point[0] = semi_axis_pow2[0] * position[0] / (time + semi_axis_pow2[0]);
            point[1] = semi_axis_pow2[1] * position[1] / (time + semi_axis_pow2[1]);
        } else {
            point[0] = 0.0;
            point[1] = semi_axis[1];
        }
    } else {
        const double denominator = semi_axis_pow2[0] - semi_axis_pow2[1];
        const double semi_axis_mul_pos = semi_axis[0] * position[0];
        if (semi_axis_mul_pos < denominator) {
            const double semi_axis_div_denom = semi_axis_mul_pos / denominator;
            const double semi_axis_div_denom_pow2 = semi_axis_div_denom * semi_axis_div_denom;
            point[0] = semi_axis[0] * semi_axis_div_denom;
            point[1] = semi_axis[1] * sqrt(abs(1.0 - semi_axis_div_denom_pow2));
        } else {
            point[0] = semi_axis[0];
            point[1] = 0.0;
        }
    }
}
