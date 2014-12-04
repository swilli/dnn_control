#include "utility.h"

#include <gsl/gsl_errno.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_roots.h>
#include <boost/math/tools/roots.hpp>

#include "constants.h"

using boost::math::tools::bisect;

static std::random_device rd;
static std::mt19937 generator(rd());
static std::uniform_real_distribution<double> distribution(0.0, 1.0);

double SampleUniform(const double &minimum, const double &maximum) {
    return minimum + (maximum - minimum) * distribution(generator);
}

void SamplePointOutSideEllipsoid(const Vector3D &semi_axis, const double &band_width_scale, Vector3D &point) {
    const double u = SampleUniform(0.0, 2.0 * k_pi);
    const double v = SampleUniform(0.0, k_pi);
    point[0] = (1.0 + distribution(generator) * (band_width_scale - 1.0)) * semi_axis[0] * cos(u) * sin(v);
    point[1] = (1.0 + distribution(generator) * (band_width_scale - 1.0)) * semi_axis[1] * sin(u) * sin(v);
    point[2] = (1.0 + distribution(generator) * (band_width_scale - 1.0)) * semi_axis[2] * cos(v);
}

double BisectEllipsoid(const Vector3D &semi_axis_mul_pos, const Vector3D &semi_axis_pow2, const double &tolerance)
{
    double lower_boundary = 0.0;
    double upper_boundary = 0.0;
    for (int i = 0; i < 3; ++i) {
        upper_boundary += semi_axis_mul_pos[i] * semi_axis_mul_pos[i];
    }
    upper_boundary = sqrt(upper_boundary);

    struct TerminationCondition  {
        double tolerance;
        bool operator() (double min, double max)  {
            return abs(min - max) <= tolerance;
        };
        TerminationCondition(const double &tolerance) : tolerance(tolerance) {};
    };

    struct FunctionToApproximate  {
        Vector3D semi_axis_mul_pos;
        Vector3D semi_axis_pow2;
        double operator() (double time)  {
            double result = 0.0;
            for (int i = 0; i < 3; ++i) {
                result += (semi_axis_mul_pos[i] / (time + semi_axis_pow2[i])) * (semi_axis_mul_pos[i] / (time + semi_axis_pow2[i]));
            }
            result -= 1.0;
            return result;
        };
        FunctionToApproximate(const Vector3D &par_semi_axis_mul_pos, const Vector3D &par_semi_axis_pow2) {
            for(int i = 0; i < 3; ++i) {
                semi_axis_mul_pos[i] = par_semi_axis_mul_pos[i];
                semi_axis_pow2[i] = par_semi_axis_pow2[i];
            }
        };
    };

    std::pair<double, double> result = bisect<FunctionToApproximate, double, TerminationCondition>(FunctionToApproximate(semi_axis_mul_pos, semi_axis_pow2), lower_boundary, upper_boundary, TerminationCondition(tolerance));

    const double root = (result.first + result.second) / 2.0;
    return root;
}

double BisectEllipse(const Vector2D &semi_axis_mul_pos, const Vector2D &semi_axis_pow2, const double &tolerance)
{
    double lower_boundary = 0.0;
    double upper_boundary = 0.0;
    for (int i = 0; i < 2; ++i) {
        upper_boundary += semi_axis_mul_pos[i] * semi_axis_mul_pos[i];
    }
    upper_boundary = sqrt(upper_boundary);

    struct TerminationCondition  {
        double tolerance;
        bool operator() (double min, double max)  {
            return abs(min - max) <= tolerance;
        };
        TerminationCondition(const double &tolerance) : tolerance(tolerance) {};
    };

    struct FunctionToApproximate  {
        Vector2D semi_axis_mul_pos;
        Vector2D semi_axis_pow2;
        double operator() (double time)  {
            double result = 0.0;
            for (int i = 0; i < 2; ++i) {
                result += (semi_axis_mul_pos[i] / (time + semi_axis_pow2[i])) * (semi_axis_mul_pos[i] / (time + semi_axis_pow2[i]));
            }
            result -= 1.0;
            return result;
        };
        FunctionToApproximate(const Vector2D &par_semi_axis_mul_pos, const Vector2D &par_semi_axis_pow2) {
            for(int i = 0; i < 2; ++i) {
                semi_axis_mul_pos[i] = par_semi_axis_mul_pos[i];
                semi_axis_pow2[i] = par_semi_axis_pow2[i];
            }
        };
    };

    std::pair<double, double> result = bisect<FunctionToApproximate, double, TerminationCondition>(FunctionToApproximate(semi_axis_mul_pos, semi_axis_pow2), lower_boundary, upper_boundary, TerminationCondition(tolerance));

    const double root = (result.first + result.second) / 2.0;
    return root;
}
