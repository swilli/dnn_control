#include "asteroid.h"
#include "constants.h"
#include "utility.h"

#include <gsl/gsl_poly.h>
#include <gsl/gsl_sf_ellint.h>
#include <gsl/gsl_sf_elljac.h>

#include <math.h>

Asteroid::Asteroid() {

}

Asteroid::Asteroid(const Vector3D &semi_axis, const double &density, const Vector3D &angular_velocity, const double &time_bias) {
    time_bias_ = time_bias;

    density_ = density;

    mass_ = 4.0 / 3.0 * k_pi * density;
    for (int i = 0; i < 3; ++i) {
        semi_axis_[i] = semi_axis[i];
        semi_axis_pow2_[i] = semi_axis_[i] * semi_axis_[i];
        initial_angular_velocity_[i] = angular_velocity[i];
        mass_ *= semi_axis_[i];
    }

    Vector3D signs;
    signs[0] = (initial_angular_velocity_[0] > 0 ? 1.0 : -1.0);
    signs[1] = (initial_angular_velocity_[0] * initial_angular_velocity_[2] > 0 ? 1.0 : -1.0);
    signs[2] = (initial_angular_velocity_[2] > 0 ? 1.0 : -1.0);

    inertia_[0] = 0.2 * mass_ * (semi_axis_pow2_[1] + semi_axis_pow2_[2]);
    inertia_[1] = 0.2 * mass_ * (semi_axis_pow2_[0] + semi_axis_pow2_[2]);
    inertia_[2] = 0.2 * mass_ * (semi_axis_pow2_[0] + semi_axis_pow2_[1]);

    energy_mul2_ = 0.0;
    momentum_pow2_ = 0.0;
    Vector3D inertia;
    for (int i = 0; i < 3; ++i) {
        inertia[i] = inertia_[i];
        inertia_pow2_[i] = inertia_[i] * inertia_[i];
        energy_mul2_ += inertia_[i] * initial_angular_velocity_[i] * initial_angular_velocity_[i];
        momentum_pow2_ += inertia_[i] * inertia_[i] * initial_angular_velocity_[i] * initial_angular_velocity_[i];
    }

    inversion_ = false;
    if (momentum_pow2_ < energy_mul2_ * inertia_[1]) {
        inversion_ = true;
        inertia[2] = inertia_[0];
        inertia[0] = inertia_[2];
        const double tmp = signs[0];
        signs[0] = signs[2];
        signs[2] = tmp;
    }

    // Lifshitz eq (37.10)
    elliptic_coefficients_[0] = signs[0] * sqrt((energy_mul2_ * inertia[2] - momentum_pow2_) / (inertia[0] * (inertia[2] - inertia[0])));
    elliptic_coefficients_[1] = signs[1] * sqrt((energy_mul2_ * inertia[2] - momentum_pow2_) / (inertia[1] * (inertia[2] - inertia[1])));
    elliptic_coefficients_[2] = signs[2] * sqrt((momentum_pow2_ - energy_mul2_ * inertia[0]) / (inertia[2] * (inertia[2] - inertia[0])));

    // Lifshitz eq (37.8)
    elliptic_tau_ = sqrt((inertia[2] - inertia[1]) * (momentum_pow2_ - energy_mul2_ * inertia[0]) / (inertia[0] * inertia[1] * inertia[2]));

    // Lifshitz eq (37.9)
    elliptic_modulus_ = (inertia[1] - inertia[0]) * (energy_mul2_ * inertia[2] - momentum_pow2_) / ((inertia[2] - inertia[1]) * (momentum_pow2_ - energy_mul2_ * inertia[0]));

    coef_mass_gravitational_constant_ = mass_ * k_gravitational_constant;
}

double Asteroid::SemiAxis(const int &dimension) const {
    return semi_axis_[dimension];
}

double Asteroid::Inertia(const int &dimension) const {
    return inertia_[dimension];
}

double Asteroid::Density() const
{
    return density_;
}

double Asteroid::TimeBias() const
{
    return time_bias_;
}

Vector3D Asteroid::GravityAtPosition(const Vector3D &position) const
{
    Vector3D gravity = {0.0, 0.0, 0.0};

    const double pos_x_pow2 = position[0] * position[0];
    const double pos_y_pow2 = position[1] * position[1];
    const double pos_z_pow2 = position[2] * position[2];

    // Cersosimo eq (3.7)
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

    // Improvement of Dario Izzo
    gravity[0] = -coef_mass_gravitational_constant_ * gsl_sf_ellint_RD(semi_axis_pow2_[2] + kappa, semi_axis_pow2_[1] + kappa, semi_axis_pow2_[0] + kappa, 0);
    gravity[1] = -coef_mass_gravitational_constant_ * gsl_sf_ellint_RD(semi_axis_pow2_[0] + kappa, semi_axis_pow2_[2] + kappa, semi_axis_pow2_[1] + kappa, 0);
    gravity[2] = -coef_mass_gravitational_constant_ * gsl_sf_ellint_RD(semi_axis_pow2_[1] + kappa, semi_axis_pow2_[0] + kappa, semi_axis_pow2_[2] + kappa, 0);

    gravity[0] *= position[0];
    gravity[1] *= position[1];
    gravity[2] *= position[2];

    return gravity;
}

boost::tuple<Vector3D, Vector3D> Asteroid::AngularVelocityAndAccelerationAtTime(const double &time) const {
    Vector3D velocity;
    Vector3D acceleration;
    // Lifshitz eq (37.8)
    const double t = (time + time_bias_) * elliptic_tau_;

    // Get analytical solution
    double sn_tau = 0.0, cn_tau = 0.0, dn_tau = 0.0;
    gsl_sf_elljac_e(t,elliptic_modulus_,&sn_tau, &cn_tau, & dn_tau);

    // Lifshitz eq (37.10)
    if (inversion_) {
        velocity[0] = elliptic_coefficients_[2] * dn_tau;
        velocity[1] = elliptic_coefficients_[1] * sn_tau;
        velocity[2] = elliptic_coefficients_[0] * cn_tau;
    } else {
        velocity[0] = elliptic_coefficients_[0] * cn_tau;
        velocity[1] = elliptic_coefficients_[1] * sn_tau;
        velocity[2] = elliptic_coefficients_[2] * dn_tau;
    }

    // Lifshitz eq (36.5)
    acceleration[0] = (inertia_[1] - inertia_[2]) * velocity[1] * velocity[2] / inertia_[0];
    acceleration[1] = (inertia_[2] - inertia_[0]) * velocity[2] * velocity[0] / inertia_[1];
    acceleration[2] = (inertia_[0] - inertia_[1]) * velocity[0] * velocity[1] / inertia_[2];

    return make_tuple(velocity, acceleration);
}

boost::tuple<Vector3D, double> Asteroid::NearestPointOnSurfaceToPosition(const Vector3D &position) const {
    Vector3D signs;
    Vector3D abs_position;

    // Project point to first quadrant, keep in mind the original point signs
    for (int i = 0; i < 3; ++i) {
        signs[i] = (position[i] >= 0.0 ? 1.0 : -1.0);
        abs_position[i] = signs[i] * position[i];
    }

    // Look for the closest point in the first quadrant
    Vector3D point = NearestPointOnEllipsoidFirstQuadrant(abs_position);

    // Project point from first quadrant back to original quadrant
    point[0] *= signs[0];
    point[1] *= signs[1];
    point[2] *= signs[2];

    double result = 0.0;
    for(int i = 0; i < 3; ++i) {
        result += (point[i] - position[i]) * (point[i] - position[i]);
    }

    const double distance = sqrt(result);

    return make_tuple(point, distance);
}

Vector3D Asteroid::InitialAngularVelocity() const
{
    return initial_angular_velocity_;
}

Vector3D Asteroid::NearestPointOnEllipsoidFirstQuadrant(const Vector3D &position) const {
    Vector3D point = {0.0, 0.0, 0.0};

    // Check if all dimensions are non zero
    if (position[2] > 0.0) {
        if (position[1] > 0.0) {
            if (position[0] > 0.0) {
                // Perform bisection to find the root (David Eberly eq (26))
                Vector3D semi_axis_mul_pos = {semi_axis_[0] * position[0], semi_axis_[1] * position[1], semi_axis_[2] * position[2]};
                const double time = BisectEllipsoid(semi_axis_mul_pos, semi_axis_pow2_);
                point[0] = semi_axis_pow2_[0] * position[0] / (time + semi_axis_pow2_[0]);
                point[1] = semi_axis_pow2_[1] * position[1] / (time + semi_axis_pow2_[1]);
                point[2] = semi_axis_pow2_[2] * position[2] / (time + semi_axis_pow2_[2]);
            } else {
                // One Dimension is zero: 2D case
                point[0] = 0.0;
                const Vector2D semi_axis_2d = {semi_axis_[1], semi_axis_[2]};
                const Vector2D position_2d = {position[1], position[2]};
                const Vector2D point_2d = NearestPointOnEllipseFirstQuadrant(semi_axis_2d, position_2d);
                point[1] = point_2d[0];
                point[2] = point_2d[1];
            }
        } else {
            point[1] = 0.0;
            if (position[0] > 0.0) {
                // One Dimension is zero: 2D case
                const Vector2D semi_axis_2d = {semi_axis_[0], semi_axis_[2]};
                const Vector2D position_2d = {position[0], position[2]};
                const Vector2D point_2d = NearestPointOnEllipseFirstQuadrant(semi_axis_2d, position_2d);
                point[0] = point_2d[0];
                point[2] = point_2d[1];
            } else {
                // Simple: only one non-zero dimension: closest point to that is the semi axis length in that dimension
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
                const Vector2D point_2d = NearestPointOnEllipseFirstQuadrant(semi_axis_2d, position_2d);
                point[0] = point_2d[0];
                point[1] = point_2d[1];
            }
        } else {
            point[2] = 0.0;
            const Vector2D semi_axis_2d = {semi_axis_[0], semi_axis_[1]};
            const Vector2D position_2d = {position[0], position[1]};
            const Vector2D point_2d = NearestPointOnEllipseFirstQuadrant(semi_axis_2d, position_2d);
            point[0] = point_2d[0];
            point[1] = point_2d[1];
        }
    }

    return point;
}

Vector2D Asteroid::NearestPointOnEllipseFirstQuadrant(const Vector2D &semi_axis, const Vector2D &position) const {
    Vector2D point = {0.0, 0.0};

    const Vector2D semi_axis_pow2 = {semi_axis[0] * semi_axis[0], semi_axis[1] * semi_axis[1]};

    // Check if all dimensions are non zero
    if (position[1] > 0.0) {
        if (position[0] > 0.0) {
            // Perform bisection to find the root (David Eberly eq (11))
            Vector2D semi_axis_mul_pos = {semi_axis_[0] * position[0], semi_axis_[1] * position[1]};
            const double time = BisectEllipse(semi_axis_mul_pos, semi_axis_pow2);
            point[0] = semi_axis_pow2[0] * position[0] / (time + semi_axis_pow2[0]);
            point[1] = semi_axis_pow2[1] * position[1] / (time + semi_axis_pow2[1]);
        } else {
            // Simple: only one non-zero dimension: closest point to that is the semi axis length in that dimension
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
            double value = 1.0 - semi_axis_div_denom_pow2;
            if (value < 0.0) {
                value = -value;
            }
            point[1] = semi_axis[1] * sqrt(value);
        } else {
            // Simple: only one non-zero dimension: closest point to that is the semi axis length in that dimension
            point[0] = semi_axis[0];
            point[1] = 0.0;
        }
    }

    return point;
}
