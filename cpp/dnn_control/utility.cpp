#include "utility.h"
#include "constants.h"
#include "samplefactory.h"

#include <boost/math/tools/roots.hpp>

static struct TerminationCondition  {
    double tolerance;
    bool operator() (double min, double max)  {
        double discr = min - max;
        if (discr < 0.0) {
            discr = -discr;
        }
        return discr <= tolerance;
    };
    TerminationCondition(const double &tolerance) : tolerance(tolerance) {};
} termination_condition(1e-15);

static struct BisectEllipsoidApproximator  {
    Vector3D semi_axis_mul_pos;
    Vector3D semi_axis_pow2;
    double operator() (double time)  {
        double result = 0.0;
        for (unsigned int i = 0; i < 3; ++i) {
            result += (semi_axis_mul_pos[i] / (time + semi_axis_pow2[i])) * (semi_axis_mul_pos[i] / (time + semi_axis_pow2[i]));
        }
        result -= 1.0;
        return result;
    };
} bisect_ellipsoid_approximator;

double BisectEllipsoid(const Vector3D &semi_axis_mul_pos, const Vector3D &semi_axis_pow2) {
    using boost::math::tools::bisect;

    double lower_boundary = 0.0;
    double upper_boundary = 0.0;
    for (unsigned int i = 0; i < 3; ++i) {
        upper_boundary += semi_axis_mul_pos[i] * semi_axis_mul_pos[i];
    }
    upper_boundary = sqrt(upper_boundary);

    VectorCopy3D(semi_axis_mul_pos, bisect_ellipsoid_approximator.semi_axis_mul_pos);
    VectorCopy3D(semi_axis_pow2, bisect_ellipsoid_approximator.semi_axis_pow2);

    try {
        std::pair<double, double> result = bisect(bisect_ellipsoid_approximator, lower_boundary, upper_boundary, termination_condition);

        const double root = (result.first + result.second) / 2.0;
        return root;

    } catch (boost::exception const &exception) {
        throw PositionInsideEllipsoidException();
    }
}

static struct BisectEllipseApproximator {
    Vector2D semi_axis_mul_pos;
    Vector2D semi_axis_pow2;
    double operator() (double time)  {
        double result = 0.0;
        for (unsigned int i = 0; i < 2; ++i) {
            result += (semi_axis_mul_pos[i] / (time + semi_axis_pow2[i])) * (semi_axis_mul_pos[i] / (time + semi_axis_pow2[i]));
        }
        result -= 1.0;
        return result;
    };
} bisect_ellipse_approximator;

double BisectEllipse(const Vector2D &semi_axis_mul_pos, const Vector2D &semi_axis_pow2) {
    using boost::math::tools::bisect;

    double lower_boundary = 0.0;
    double upper_boundary = 0.0;
    for (unsigned int i = 0; i < 2; ++i) {
        upper_boundary += semi_axis_mul_pos[i] * semi_axis_mul_pos[i];
    }
    upper_boundary = sqrt(upper_boundary);

    VectorCopy2D(semi_axis_mul_pos, bisect_ellipse_approximator.semi_axis_mul_pos);
    VectorCopy2D(semi_axis_pow2, bisect_ellipse_approximator.semi_axis_pow2);

    try {
        std::pair<double, double> result = bisect(bisect_ellipse_approximator, lower_boundary, upper_boundary, termination_condition);

        const double root = (result.first + result.second) / 2.0;
        return root;

    } catch (boost::exception const &exception) {
        throw PositionInsideEllipseException();
    }
}
