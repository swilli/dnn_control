#include "asteroid.h"
#include <gsl/gsl_poly.h>
#include <gsl/gsl_sf_ellint.h>
#include <gsl/gsl_sf_elljac.h>
#include <math.h>
#include "constants.h"
#include "utility.h"

Asteroid::Asteroid(const bp::list &param_semi_axis, const double &density, const bp::list &param_angular_velocity, const double &time_bias) {
    const Vector3D semi_axis = {boost::python::extract<double>(param_semi_axis[0]),
        boost::python::extract<double>(param_semi_axis[1]),
        boost::python::extract<double>(param_semi_axis[2])};
    const Vector3D angular_velocity = {boost::python::extract<double>(param_angular_velocity[0]),
        boost::python::extract<double>(param_angular_velocity[1]),
        boost::python::extract<double>(param_angular_velocity[2])};

    time_bias_ = time_bias;

    density_ = density;

    mass_ = 4.0 / 3.0 * k_pi * density;
    for (int i = 0; i < 3; ++i) {
        semi_axis_[i] = semi_axis[i];
        semi_axis_pow2_[i] = semi_axis_[i] * semi_axis_[i];
        mass_ *= semi_axis_[i];
    }

    Vector3D signs;
    signs[0] = (angular_velocity[0] > 0 ? 1.0 : -1.0);
    signs[1] = (angular_velocity[0] * angular_velocity[2] > 0 ? 1.0 : -1.0);
    signs[2] = (angular_velocity[2] > 0 ? 1.0 : -1.0);

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
        energy_mul2_ += inertia_[i] * angular_velocity[i] * angular_velocity[i];
        momentum_pow2_ += inertia_[i] * inertia_[i] * angular_velocity[i] * angular_velocity[i];
    }
    // Cersosimo eq (3.12)
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

    // Lifshitz eq (37.10)
    elliptic_coefficients_[0] = signs[0] * sqrt((energy_mul2_ * inertia[2] - momentum_pow2_) / (inertia[0] * (inertia[2] - inertia[0])));
    elliptic_coefficients_[1] = signs[1] * sqrt((energy_mul2_ * inertia[2] - momentum_pow2_) / (inertia[1] * (inertia[2] - inertia[1])));
    elliptic_coefficients_[2] = signs[2] * sqrt((momentum_pow2_ - energy_mul2_ * inertia[0]) / (inertia[2] * (inertia[2] - inertia[0])));

    // Lifshitz eq (37.8)
    elliptic_tau_ = sqrt((inertia[2] - inertia[1]) * (momentum_pow2_ - energy_mul2_ * inertia[0]) / (inertia[0] * inertia[1] * inertia[2]));

    // Lifshitz eq (37.9)
    elliptic_modulus_ = (inertia[1] - inertia[0]) * (energy_mul2_ * inertia[2] - momentum_pow2_) / ((inertia[2] - inertia[1]) * (momentum_pow2_ - energy_mul2_ * inertia[0]));
}

Asteroid::Asteroid(const Vector3D &semi_axis, const double &density, const Vector3D &angular_velocity, const double &time_bias) {
    time_bias_ = time_bias;

    density_ = density;

    mass_ = 4.0 / 3.0 * k_pi * density;
    for (int i = 0; i < 3; ++i) {
        semi_axis_[i] = semi_axis[i];
        semi_axis_pow2_[i] = semi_axis_[i] * semi_axis_[i];
        mass_ *= semi_axis_[i];
    }

    Vector3D signs;
    signs[0] = (angular_velocity[0] > 0 ? 1.0 : -1.0);
    signs[1] = (angular_velocity[0] * angular_velocity[2] > 0 ? 1.0 : -1.0);
    signs[2] = (angular_velocity[2] > 0 ? 1.0 : -1.0);

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
        energy_mul2_ += inertia_[i] * angular_velocity[i] * angular_velocity[i];
        momentum_pow2_ += inertia_[i] * inertia_[i] * angular_velocity[i] * angular_velocity[i];
    }
    // Cersosimo eq (3.12)
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

    // Lifshitz eq (37.10)
    elliptic_coefficients_[0] = signs[0] * sqrt((energy_mul2_ * inertia[2] - momentum_pow2_) / (inertia[0] * (inertia[2] - inertia[0])));
    elliptic_coefficients_[1] = signs[1] * sqrt((energy_mul2_ * inertia[2] - momentum_pow2_) / (inertia[1] * (inertia[2] - inertia[1])));
    elliptic_coefficients_[2] = signs[2] * sqrt((momentum_pow2_ - energy_mul2_ * inertia[0]) / (inertia[2] * (inertia[2] - inertia[0])));

    // Lifshitz eq (37.8)
    elliptic_tau_ = sqrt((inertia[2] - inertia[1]) * (momentum_pow2_ - energy_mul2_ * inertia[0]) / (inertia[0] * inertia[1] * inertia[2]));

    // Lifshitz eq (37.9)
    elliptic_modulus_ = (inertia[1] - inertia[0]) * (energy_mul2_ * inertia[2] - momentum_pow2_) / ((inertia[2] - inertia[1]) * (momentum_pow2_ - energy_mul2_ * inertia[0]));
}

double Asteroid::SemiAxis(const int &dimension) const {
    return semi_axis_[dimension];
}

double Asteroid::Inertia(const int &dimension) const {
    return inertia_[dimension];
}

Vector3D Asteroid::GravityAtPosition(const Vector3D &position) const {
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

    // Cersosimo eq (3.8)
    const double phi = asin(sqrt((semi_axis_pow2_[0] - semi_axis_pow2_[2]) / (kappa + semi_axis_pow2_[0])));

    // Cersosimo eq (3.9)
    const double k = sqrt((semi_axis_pow2_[0] - semi_axis_pow2_[1]) / (semi_axis_pow2_[0] - semi_axis_pow2_[2]));

    // Cersosimo eq (3.10)
    const double integral_F = gsl_sf_ellint_F(phi,k,0);
    const double integral_E = gsl_sf_ellint_E(phi,k,0);

    // Cersosimo eq (3.13)
    const double delta = sqrt((semi_axis_pow2_[0] - semi_axis_pow2_[2]) / ((semi_axis_pow2_[0] + kappa) * (semi_axis_pow2_[1] + kappa) * (semi_axis_pow2_[2] + kappa)));

    // Cersosimo eq (3.11a-c)
    gravity[0] = gamma_ / (semi_axis_pow2_[0] - semi_axis_pow2_[1]) * (integral_E - integral_F);
    gravity[1] = gamma_ * ((-semi_axis_pow2_[0] + semi_axis_pow2_[2]) * integral_E / ((semi_axis_pow2_[0] - semi_axis_pow2_[1]) * (semi_axis_pow2_[1] - semi_axis_pow2_[2]))
            + integral_F / (semi_axis_pow2_[0] - semi_axis_pow2_[1]) + delta * (semi_axis_pow2_[2] + kappa) / (semi_axis_pow2_[1] - semi_axis_pow2_[2]));
    gravity[2] = gamma_ / (semi_axis_pow2_[2] - semi_axis_pow2_[1]) * (-integral_E + (semi_axis_pow2_[1] + kappa) * delta);

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
}

bp::list Asteroid::GravityAtPositionWrapper(const bp::list &param_position) const {
    const Vector3D position = {boost::python::extract<double>(param_position[0]),
        boost::python::extract<double>(param_position[1]),
        boost::python::extract<double>(param_position[2])};

    const Vector3D gravity = GravityAtPosition(position);

    bp::list result;
    result.append(gravity[0]);
    result.append(gravity[1]);
    result.append(gravity[2]);
    return result;
}

bp::tuple Asteroid::AngularVelocityAndAccelerationAtTimeWrapper(const double &time) const {
    const boost::tuple<Vector3D, Vector3D> result = AngularVelocityAndAccelerationAtTime(time);
    const Vector3D velocity = boost::get<0>(result);
    const Vector3D acceleration = boost::get<1>(result);

    bp::list vel, acc;
    for (int i = 0; i < 3; ++i) {
        vel.append(velocity[i]);
        acc.append(acceleration[i]);
    }
    return make_tuple(vel, acc);
}

bp::tuple Asteroid::NearestPointOnSurfaceToPositionWrapper(const bp::list &param_position) const {
    const Vector3D position = {boost::python::extract<double>(param_position[0]),
        boost::python::extract<double>(param_position[1]),
        boost::python::extract<double>(param_position[2])};

    const boost::tuple<Vector3D, double> result = NearestPointOnSurfaceToPosition(position);
    const Vector3D surface_position = boost::get<0>(result);
    const double distance = boost::get<1>(result);

    bp::list result_bp;
    result_bp.append(surface_position[0]);
    result_bp.append(surface_position[1]);
    result_bp.append(surface_position[2]);

    return make_tuple(result_bp, distance);
}

BOOST_PYTHON_MODULE(boost_asteroid)
{
    bp::class_<Asteroid>("Asteroid", bp::init<const bp::list&, const double&, const bp::list&, const double&>())
        .def("get_semi_axis", &Asteroid::SemiAxis)
        .def("get_inertia", &Asteroid::Inertia)
        .def("gravity_at_position", &Asteroid::GravityAtPositionWrapper)
        .def("angular_velocity_and_acceleration_at_time", &Asteroid::AngularVelocityAndAccelerationAtTimeWrapper)
        .def("nearest_point_on_surface_to_position", &Asteroid::NearestPointOnSurfaceToPositionWrapper)
    ;
}