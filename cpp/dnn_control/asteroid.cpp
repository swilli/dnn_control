#include "asteroid.h"
#include "constants.h"
#include "utility.h"

#include <algorithm>
#include <set>
#include <math.h>
#include <gsl/gsl_poly.h>
#include <gsl/gsl_sf_ellint.h>
#include <gsl/gsl_sf_elljac.h>

#include <iostream>

Asteroid::Asteroid() {

}

Asteroid::Asteroid(const Vector3D &semi_axis, const double &density, const Vector2D &angular_velocity_xz, const double &time_bias) {
    time_bias_ = time_bias;

    density_ = density;

    constructor_angular_velocities_xz_[0] = angular_velocity_xz[0];
    constructor_angular_velocities_xz_[1] = angular_velocity_xz[1];

    const Vector3D angular_velocity_3d = {angular_velocity_xz[0], 0.0, angular_velocity_xz[1]};

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
    elliptic_modulus_ = (inertia[1] - inertia[0]) * (energy_mul2_ * inertia[2] - momentum_pow2_) / ((inertia[2] - inertia[1]) * (momentum_pow2_ - energy_mul2_ * inertia[0]));

    mass_gravitational_constant_ = mass_ * kGravitationalConstant;

    num_points_ = 100;
    const double dpi = kPi / 2.0 / (num_points_-1);
    std::set<std::vector<double> > configurations;
    for (unsigned int i = 0; i < num_points_; ++i) {
        for (unsigned int j = 0; j < num_points_; ++j) {
            const double u = i * dpi;
            const double v = j * dpi;
            std::vector<double> cur_conf(3);
            cur_conf[0] = cos(u) * sin(v);
            cur_conf[1] = sin(u) * sin(v);
            cur_conf[2] =  cos(v);
            if (configurations.find(cur_conf) == configurations.end()) {
                configurations.insert(cur_conf);
                const Vector3D point = {semi_axis_[0] * cur_conf[0],
                                    semi_axis_[1] * cur_conf[1],
                                    semi_axis_[2] * cur_conf[2]};

                points_.push_back(point);
            }
        }
    }
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

bool Asteroid::SortMinimumZ(const Vector3D &first, const Vector3D &second) {
    return first[2] < second[2];
}

Vector3D Asteroid::GravityAccelerationAtPosition(const Vector3D &position) const {
    Vector3D acceleration = {0.0, 0.0, 0.0};

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
    if(num_roots == 1) {
        kappa = root_1;
    } else {
        kappa = root_3;
    }

    // Improvement of Dario Izzo
    acceleration[0] = -mass_gravitational_constant_ * gsl_sf_ellint_RD(semi_axis_pow2_[2] + kappa, semi_axis_pow2_[1] + kappa, semi_axis_pow2_[0] + kappa, 0) * position[0];
    acceleration[1] = -mass_gravitational_constant_ * gsl_sf_ellint_RD(semi_axis_pow2_[0] + kappa, semi_axis_pow2_[2] + kappa, semi_axis_pow2_[1] + kappa, 0) * position[1];
    acceleration[2] = -mass_gravitational_constant_ * gsl_sf_ellint_RD(semi_axis_pow2_[1] + kappa, semi_axis_pow2_[0] + kappa, semi_axis_pow2_[2] + kappa, 0) * position[2];

    return acceleration;
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

boost::tuple<Vector3D, double> Asteroid::NearestPointOnSurfaceToPositionImpl2(const Vector3D &position) const {
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

    double result = 0.0;
    for(unsigned int i = 0; i < 3; ++i) {
        result += (point[i] - position[i]) * (point[i] - position[i]);
    }

    const double distance = sqrt(result);

    return make_tuple(point, distance);
}

boost::tuple<Vector3D, double> Asteroid::NearestPointOnSurfaceToPosition(const Vector3D &position) const {
    Vector3D signs;
    Vector3D abs_position;

    // Project point to first quadrant, keep in mind the original point signs
    for (unsigned int i = 0; i < 3; ++i) {
        signs[i] = (position[i] >= 0.0 ? 1.0 : -1.0);
        abs_position[i] = signs[i] * position[i];
    }

    const unsigned int num_points = points_.size();

    double min_distance = 1e20;
    Vector3D point;
    for (unsigned int i = 0; i < num_points; ++i) {
        const Vector3D &cur_pos = points_.at(i);
        const double dist = VectorNorm(VectorSub(cur_pos, abs_position));
        if (dist < min_distance) {
            min_distance = dist;
            point = cur_pos;
        }
    }

    // Project point from first quadrant back to original quadrant
    point[0] *= signs[0];
    point[1] *= signs[1];
    point[2] *= signs[2];

    return make_tuple(point, min_distance);

    /*std::vector<std::pair<double, unsigned int> > nearest_neigh(num_points, std::make_pair(0.0, 0));
    for (unsigned int i = 0; i < num_points; ++i) {
        nearest_neigh.at(i) = std::make_pair(VectorNorm(VectorSub(abs_position, points_.at(i))), i);
    }
    sort(nearest_neigh.begin(), nearest_neigh.end());

    std::vector<Vector3D> plane_points(3);
    plane_points[0] = points_.at(nearest_neigh.at(0).second);
    plane_points[1] = points_.at(nearest_neigh.at(1).second);
    plane_points[2] = points_.at(nearest_neigh.at(2).second);

    std::cout << VectorToString(abs_position) << std::endl;
    for (unsigned int i = 0; i < 3; ++i) {
        std::cout << VectorToString(plane_points[i]) << std::endl;
    }
    sort(plane_points.begin(), plane_points.end(), SortMinimumZ);

    const Vector3D &point_21 = {plane_points[1][0] - plane_points[0][0], plane_points[1][1] - plane_points[0][1], plane_points[1][2] - plane_points[0][2]};
    const Vector3D &point_31 = {plane_points[2][0] - plane_points[0][0], plane_points[2][1] - plane_points[0][1], plane_points[2][2] - plane_points[0][2]};

    Vector3D plane_normal = VectorCrossProduct(point_31, point_21);
    const double coef_plane_norm = 1.0 / VectorNorm(plane_normal);

    plane_normal[0] *= coef_plane_norm;
    plane_normal[1] *= coef_plane_norm;
    plane_normal[2] *= coef_plane_norm;

    Vector3D tmp = VectorSub(abs_position, points_.at(nearest_neigh.at(0).second));
    double distance = VectorDotProduct(plane_normal, tmp);
    distance = (distance < 0.0 ? -distance : distance);

    double coeftmp_norm = 1.0 / VectorNorm(tmp);
    tmp[0] *= coeftmp_norm;
    tmp[1] *= coeftmp_norm;
    tmp[2] *= coeftmp_norm;

    Vector3D point = {abs_position[0] - distance * plane_normal[0], abs_position[1] - distance * plane_normal[1], abs_position[2] - distance * plane_normal[2]};

    // Project point from first quadrant back to original quadrant
    point[0] *= signs[0];
    point[1] *= signs[1];
    point[2] *= signs[2];

    return make_tuple(point, distance);
    */
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
                Vector3D semi_axis_mul_pos = {semi_axis_[0] * position[0], semi_axis_[1] * position[1], semi_axis_[2] * position[2]};
                double time = 0.0;
                try {
                    time = BisectEllipsoid(semi_axis_mul_pos, semi_axis_pow2_);
                } catch (const UtilityException &exception) {
                    throw PositionInsideException();
                }
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
            double time = 0.0;
            try {
                time = BisectEllipse(semi_axis_mul_pos, semi_axis_pow2);
            } catch (const UtilityException &exception) {
                throw PositionInsideException();
            }
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
