#include "utility.h"

#include <gsl/gsl_errno.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_roots.h>
#include <boost/math/tools/roots.hpp>

#include "constants.h"

static struct TerminationCondition {
  double tolerance = 1e-10;
  bool operator() (double min, double max)  {
    return abs(min - max) <= 1e-10;
  }
} termination_condition;

static struct FunctionToApproximate {
  Functor *container;
  double operator() (double x)  {
    return container->Evaluate(x);
  }
} function_to_approximate;

double Bisection(Functor *container, const double &minimum, const double &maximum, const int &max_iterations, const double &tolerance)
{
    using boost::math::tools::bisect;
    termination_condition.tolerance = tolerance;
    function_to_approximate.container = container;
    std::pair<double, double> result = bisect<FunctionToApproximate, double, TerminationCondition>(function_to_approximate, minimum, maximum, termination_condition);

    const double root = (result.first + result.second) / 2.0;
    return root;
}

static std::random_device rd;
static std::mt19937 generator(rd());
static std::uniform_real_distribution<double> distribution(0.0, 1.0);
void SamplePointOutSideEllipsoid(const Vector3D &semi_axis, const double &band_width_scale, Vector3D &point) {
    const double u = distribution(generator) * 2.0 * k_pi;
    const double v = distribution(generator) * k_pi;
    point[0] = (1.0 + distribution(generator) * (band_width_scale - 1.0)) * semi_axis[0] * cos(u) * sin(v);
    point[1] = (1.0 + distribution(generator) * (band_width_scale - 1.0)) * semi_axis[1] * sin(u) * sin(v);
    point[2] = (1.0 + distribution(generator) * (band_width_scale - 1.0)) * semi_axis[2] * cos(v);
}
