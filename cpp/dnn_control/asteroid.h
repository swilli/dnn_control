#ifndef ASTEROID_H
#define ASTEROID_H

#include "vector.h"

#include <boost/tuple/tuple.hpp>
#include <map>

class Asteroid {
    /*
     * This class represents the dynamics and gravity of an asteroid shaped as a rigid ellipsoid having different
     * rotational properties along its three principal inertia axis (Ix, Iy, Iz).
     *
     * For the angular velocity dynamics the analytical solution in terms of Jacobi elliptic functions is used (Implementation
     * largely inspired from Landau Lifshitz Mechanics paragraph 37). We thus assume w_y(-t_bias) = 0. So we only are free to specify two
     * more initial conditions and we do so by assigning kinetic energy and angular momentum. As a result the user is not free to specify
     * the initial angular velocity, but only the values of the two prime integrals. He does so by defining an angular velocity vector in
     * the constructor, this is used only to compute kinetic energy and rotational momentum, the second of its components (w_y) will thus be disregarded
     * as 0 will instead be used.
     *
     * Implementation for the gravity is largely inspired from Dario Cersosimo EVALUATION OF NOVEL HOVERING STRATEGIES TO IMPROVE GRAVITY-TRACTOR DEFLECTION MERITS paragraph 3.2.2.
     *
     * Implementation for the nearest point on the surface is largely ported from David Eberly "Distance from a Point to an Ellipse, an Ellipsoid, or a Hyperellipsoid".
*/
public:
    Asteroid();

    // Requires: semi_axis[0] > semi_axis[1] > semi_axis[2]
    // angular_velocity only requires values for x and z since y == 0.
    Asteroid(const Vector3D &semi_axis, const double &density, const Vector2D &angular_velocity_xz, const double &time_bias);

    // Computes the gravity components in asteroid centered RF at an outside point "position" which is also in asteroid centered RF
    Vector3D GravityAccelerationAtPosition(const Vector3D &position) const;

    // Computes w ("velocity") and d/dt ("acceleration") w of the asteroid rotating RF at time "time"
    boost::tuple<Vector3D, Vector3D> AngularVelocityAndAccelerationAtTime(const double &time) const;

    // Computes the distance "distance" and orthogonal projection of a position "position" outside the asteroid onto the asteroid's surface "point" in asteroid centered RF
    boost::tuple<Vector3D, double> NearestPointOnSurfaceToPosition(const Vector3D &position) const;

    // Returns the euler angles (theta, psi, phi) in an inertial reference frame where the axis coalign with the rotating body frame in the beginning
    Vector3D InertialEulerAnglesAtTime(const double &time) const;

    // Returns the initially in the constructor provided angular velocity in x and z dimension
    Vector2D ConstructorAngularVelocitiesXZ() const;

    // Returns the 3 semi axis
    Vector3D SemiAxis() const;

    // Returns the 3 inertia
    Vector3D Inertia() const;

    // Returns the density
    double Density() const;

    // Returns time_bias
    double TimeBias() const;

    // Returns coef_mass_gravitational_constant
    double MassGravitationalConstant() const;

    // Returns x^2/a^2 + y^2/b^2 + z^2/c^2
    double EvaluatePointWithStandardEquation(const Vector3D &position) const;

    class Exception {};
    class PositionInsideException : public Exception {};
    class InvalidParametersException: public Exception {};

private:
    // Helper functions for NearestPointOnSurfaceToPosition
    static double NewtonRaphsonEllipse(const Vector2D &semi_axis_mul_pos, const Vector2D &semi_axis_pow2);
    static double NewtonRaphsonEllipsoid(const Vector3D &semi_axis_mul_pos, const Vector3D &semi_axis_pow2);

    // Helper function for NearestPointOnSurfaceToPosition since we assume a symmetric ellipsoid. Position "position" has to be in first quadrant.
    Vector3D NearestPointOnEllipsoidFirstQuadrant(const Vector3D &position) const;

    // Helper function for NearestPointOnEllipsoidFirstQuadrant when one dimension of the 3D position is zero
    // (i.e., we have a 2D problem where we need to find the closest point on an ellipse).
    // "semi_axis": the 2 semi axis relevant for position "position"
    Vector2D NearestPointOnEllipseFirstQuadrant(const Vector2D &semi_axis, const Vector2D &position) const;

    // Mass of asteroid
    double mass_;

    // Density of asteroid
    double density_;

    // All three semi axis ordered a,b,c
    Vector3D semi_axis_;

    // Cached power of 2 of the semi axis
    Vector3D semi_axis_pow2_;

    // All three inertia ordered a, b, c
    Vector3D inertia_;

    // Cached power of 2 of the inertia
    Vector3D inertia_pow2_;

    // Cached initial angular velocity in x and z dimension
    Vector2D constructor_angular_velocities_xz_;

    // Time bias for the analytical angular velocity computation: w_y(-time_bias_) = 0.
    double time_bias_;

    // 2*Energy of asteroid
    double energy_mul2_;

    // Momentum^2 of asteroid
    double momentum_pow2_;

    // Cached m * G
    double mass_gravitational_constant_;

    // Lifshitz eq (37.10)
    Vector3D elliptic_coefficients_;

    // Lifshitz eq (37.8)
    double elliptic_tau_;

    // Lifshitz eq (37.9)
    double elliptic_modulus_;

    // True if momentum_pow2_ < energy_mul2_ * inertia_[1]
    bool inversion_;
};

#endif // ASTEROID_H
