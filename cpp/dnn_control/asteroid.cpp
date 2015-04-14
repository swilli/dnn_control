#include "asteroid.h"
#include "constants.h"

#include <cmath>
#include <gsl/gsl_poly.h>
#include <gsl/gsl_sf_ellint.h>
#include <gsl/gsl_sf_elljac.h>

#include <iostream>

Asteroid::Asteroid() {

}

Asteroid::Asteroid(const Vector3D &semi_axis, const double &density, const Vector2D &angular_velocity_xz, const double &time_bias) {
    time_bias_ = time_bias;

    density_ = density;

    constructor_angular_velocities_xz_ = angular_velocity_xz;

    const Vector3D &angular_velocity_3d = {angular_velocity_xz[0], 0.0, angular_velocity_xz[1]};

    mass_ = 4.0 / 3.0 * kPi * density_;
    for (unsigned int i = 0; i < 3; ++i) {
        semi_axis_[i] = semi_axis[i];
        semi_axis_pow2_[i] = semi_axis_[i] * semi_axis_[i];
        mass_ *= semi_axis_[i];
    }

    Vector3D signs;
    signs[0] = (angular_velocity_3d[0] > 0 ? 1.0 : -1.0);
    signs[1] = (angular_velocity_3d[0] * angular_velocity_3d[2] > 0 ? 1.0 : -1.0);
    signs[2] = (angular_velocity_3d[2] > 0 ? 1.0 : -1.0);

    inertia_[0] = 0.2 * mass_ * (semi_axis_pow2_[1] + semi_axis_pow2_[2]);
    inertia_[1] = 0.2 * mass_ * (semi_axis_pow2_[0] + semi_axis_pow2_[2]);
    inertia_[2] = 0.2 * mass_ * (semi_axis_pow2_[0] + semi_axis_pow2_[1]);

    energy_mul2_ = 0.0;
    momentum_pow2_ = 0.0;
    Vector3D inertia;
    for (unsigned int i = 0; i < 3; ++i) {
        inertia[i] = inertia_[i];
        inertia_pow2_[i] = inertia_[i] * inertia_[i];
        energy_mul2_ += inertia_[i] * angular_velocity_3d[i] * angular_velocity_3d[i];
        momentum_pow2_ += inertia_[i] * inertia_[i] * angular_velocity_3d[i] * angular_velocity_3d[i];
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
    elliptic_modulus_pow_2_ = (inertia[1] - inertia[0]) * (energy_mul2_ * inertia[2] - momentum_pow2_) / ((inertia[2] - inertia[1]) * (momentum_pow2_ - energy_mul2_ * inertia[0]));

    // Lifshitz eq (37.12)
    const double val_K = gsl_sf_ellint_Kcomp(sqrt(elliptic_modulus_pow_2_), GSL_PREC_DOUBLE);
    angular_velocity_period_ = 4.0 * val_K * sqrt(inertia[0] * inertia[1] * inertia[2] / ((inertia[2] - inertia[1]) * (momentum_pow2_ - energy_mul2_ * inertia[0])));

    mass_gravitational_constant_ = mass_ * kGravitationalConstant;
}

Vector3D Asteroid::SemiAxis() const {
    return semi_axis_;
}

Vector3D Asteroid::Inertia() const {
    return inertia_;
}

double Asteroid::Density() const {
    return density_;
}

double Asteroid::TimeBias() const {
    return time_bias_;
}

double Asteroid::MassGravitationalConstant() const {
    return mass_gravitational_constant_;
}

double Asteroid::EvaluatePointWithStandardEquation(const Vector3D &position) const {
    double result = 0.0;
    for (unsigned int i = 0; i < 3; ++i) {
        result += position[i] * position[i] / semi_axis_pow2_[i];
    }
    return result;
}

double Asteroid::AngularVelocityPeriod() const {
    return angular_velocity_period_;
}

double Asteroid::NewtonRaphsonNearestPointOnSurfaceToPositionEllipse(const Vector2D &semi_axis_mul_pos, const Vector2D &semi_axis_pow2) {
    double root = 0.0;

    const double tolerance = 1e-3;
    double old_root = root;
    double error = 0.0;

    do {
        old_root = root;
        double f_root = 0.0;
        for (unsigned int i = 0; i < 2; ++i) {
            f_root += (semi_axis_mul_pos[i] / (root + semi_axis_pow2[i])) * (semi_axis_mul_pos[i] / (root + semi_axis_pow2[i]));
        }
        f_root -= 1.0;
        double df_root = 0.0;
        for (unsigned int i = 0; i < 2; ++i) {
            df_root += (semi_axis_mul_pos[i] * semi_axis_mul_pos[i]) * (-2.0) / ((root + semi_axis_pow2[i]) * (root + semi_axis_pow2[i]) * (root + semi_axis_pow2[i]));
        }
        root = root - f_root/df_root;
        error = root - old_root;
        error = (error < 0.0 ? -error : error);
        if (std::isinf(root) || std::isnan(root)) {
            throw PositionInsideException();
        }
    } while (error > tolerance);

    return root;
}

double Asteroid::NewtonRaphsonNearestPointOnSurfaceToPositionEllipsoid(const Vector3D &semi_axis_mul_pos, const Vector3D &semi_axis_pow2) {
    double root = 0.0;

    const double tolerance = 1e-3;
    double old_root = root;
    double error = 0.0;

    do {
        old_root = root;
        double f_root = 0.0;
        for (unsigned int i = 0; i < 3; ++i) {
            f_root += (semi_axis_mul_pos[i] / (root + semi_axis_pow2[i])) * (semi_axis_mul_pos[i] / (root + semi_axis_pow2[i]));
        }
        f_root -= 1.0;
        double df_root = 0.0;
        for (unsigned int i = 0; i < 3; ++i) {
            df_root += (semi_axis_mul_pos[i] * semi_axis_mul_pos[i]) * (-2.0) / ((root + semi_axis_pow2[i]) * (root + semi_axis_pow2[i]) * (root + semi_axis_pow2[i]));
        }
        root = root - f_root/df_root;
        error = root - old_root;
        error = (error < 0.0 ? -error : error);
        if (std::isinf(root) || std::isnan(root)) {
            throw PositionInsideException();
        }
    } while (error > tolerance);

    return root;
}

Vector3D Asteroid::GravityAccelerationAtPosition(const Vector3D &position) const {
    Vector3D acceleration;

    const double eval = EvaluatePointWithStandardEquation(position);
    if (eval < 1.0) {
        throw PositionInsideException();
    }

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
    if (num_roots == 1) {
        kappa = root_1;
    } else {
        kappa = root_3;
    }

    // Improvement of Dario Izzo
    acceleration[0] = -mass_gravitational_constant_ * gsl_sf_ellint_RD(semi_axis_pow2_[1] + kappa, semi_axis_pow2_[2] + kappa, semi_axis_pow2_[0] + kappa, 0) * position[0];
    acceleration[1] = -mass_gravitational_constant_ * gsl_sf_ellint_RD(semi_axis_pow2_[0] + kappa, semi_axis_pow2_[2] + kappa, semi_axis_pow2_[1] + kappa, 0) * position[1];
    acceleration[2] = -mass_gravitational_constant_ * gsl_sf_ellint_RD(semi_axis_pow2_[0] + kappa, semi_axis_pow2_[1] + kappa, semi_axis_pow2_[2] + kappa, 0) * position[2];

    return acceleration;
}

boost::tuple<Vector3D, Vector3D> Asteroid::AngularVelocityAndAccelerationAtTime(const double &time) const {
    Vector3D velocity;
    Vector3D acceleration;

    // Lifshitz eq (37.8)
    const double t = (time + time_bias_) * elliptic_tau_;

    // Get analytical solution
    double sn_tau = 0.0, cn_tau = 0.0, dn_tau = 0.0;
    gsl_sf_elljac_e(t, elliptic_modulus_pow_2_, &sn_tau, &cn_tau, &dn_tau);

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

    return boost::make_tuple(velocity, acceleration);
}

boost::tuple<Vector3D, double> Asteroid::NearestPointOnSurfaceToPosition(const Vector3D &position) const {
    Vector3D signs;
    Vector3D abs_position;

    // Project point to first quadrant, keep in mind the original point signs
    for (unsigned int i = 0; i < 3; ++i) {
        signs[i] = (position[i] >= 0.0 ? 1.0 : -1.0);
        abs_position[i] = signs[i] * position[i];
    }

    // Look for the closest point in the first quadrant
    Vector3D point = NearestPointOnEllipsoidFirstQuadrant(abs_position);

    // Project point from first quadrant back to original quadrant
    point[0] *= signs[0];
    point[1] *= signs[1];
    point[2] *= signs[2];

    const double distance = VectorNorm(VectorSub(point, position));

    return boost::make_tuple(point, distance);
}

Vector3D Asteroid::IntersectLineToCenterFromPosition(const Vector3D &position) const {
    double root = 0.0;

    for (unsigned int i = 0; i < 3; ++i) {
        root += position[i] * position[i] / semi_axis_pow2_[i];
    }
    root = 1.0 / sqrt(root);

    // Compute the intersection
    const Vector3D point = VectorMul(root, position);

    return point;
}

Vector2D Asteroid::ConstructorAngularVelocitiesXZ() const {
    return constructor_angular_velocities_xz_;
}

Vector3D Asteroid::NearestPointOnEllipsoidFirstQuadrant(const Vector3D &position) const {
    Vector3D point = {0.0, 0.0, 0.0};

    // Check if all dimensions are non zero
    if (position[2] > 0.0) {
        if (position[1] > 0.0) {
            if (position[0] > 0.0) {
                // Perform bisection to find the root (David Eberly eq (26))
                const Vector3D &semi_axis_mul_pos = {semi_axis_[0] * position[0], semi_axis_[1] * position[1], semi_axis_[2] * position[2]};
                const double time = NewtonRaphsonNearestPointOnSurfaceToPositionEllipsoid(semi_axis_mul_pos, semi_axis_pow2_);
                point[0] = semi_axis_pow2_[0] * position[0] / (time + semi_axis_pow2_[0]);
                point[1] = semi_axis_pow2_[1] * position[1] / (time + semi_axis_pow2_[1]);
                point[2] = semi_axis_pow2_[2] * position[2] / (time + semi_axis_pow2_[2]);
            } else {
                // One Dimension is zero: 2D case
                point[0] = 0.0;
                const Vector2D &semi_axis_2d = {semi_axis_[1], semi_axis_[2]};
                const Vector2D &position_2d = {position[1], position[2]};
                const Vector2D point_2d = NearestPointOnEllipseFirstQuadrant(semi_axis_2d, position_2d);
                point[1] = point_2d[0];
                point[2] = point_2d[1];
            }
        } else {
            point[1] = 0.0;
            if (position[0] > 0.0) {
                // One Dimension is zero: 2D case
                const Vector2D &semi_axis_2d = {semi_axis_[0], semi_axis_[2]};
                const Vector2D &position_2d = {position[0], position[2]};
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
        const Vector2D &denominator = {semi_axis_pow2_[0] - semi_axis_pow2_[2], semi_axis_pow2_[1] - semi_axis_pow2_[2]};
        const Vector2D &semi_axis_mul_pos = {semi_axis_[0] * position[0], semi_axis_[1] * position[1]};

        if (semi_axis_mul_pos[0] < denominator[0] && semi_axis_mul_pos[1] < denominator[1]) {
            const Vector2D &semi_axis_div_denom = {semi_axis_mul_pos[0] / denominator[0] ,
                                                  semi_axis_mul_pos[1] / denominator[1]};
            const Vector2D &semi_axis_div_denom_pow2 = {semi_axis_div_denom[0] * semi_axis_div_denom[0],
                                                       semi_axis_div_denom[1] * semi_axis_div_denom[1]};
            const double discr = 1.0 - semi_axis_div_denom_pow2[0] - semi_axis_div_denom_pow2[1];
            if (discr > 0.0) {
                point[0] = semi_axis_[0] * semi_axis_div_denom[0];
                point[1] = semi_axis_[1] * semi_axis_div_denom[1];
                point[2] = semi_axis_[2] * sqrt(discr);
            } else {
                point[2] = 0.0;
                const Vector2D &semi_axis_2d = {semi_axis_[0], semi_axis_[1]};
                const Vector2D &position_2d = {position[0], position[1]};
                const Vector2D point_2d = NearestPointOnEllipseFirstQuadrant(semi_axis_2d, position_2d);
                point[0] = point_2d[0];
                point[1] = point_2d[1];
            }
        } else {
            point[2] = 0.0;
            const Vector2D &semi_axis_2d = {semi_axis_[0], semi_axis_[1]};
            const Vector2D &position_2d = {position[0], position[1]};
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
            const Vector2D &semi_axis_mul_pos = {semi_axis_[0] * position[0], semi_axis_[1] * position[1]};
            const double time = NewtonRaphsonNearestPointOnSurfaceToPositionEllipse(semi_axis_mul_pos, semi_axis_pow2);
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
