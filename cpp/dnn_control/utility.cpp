#include "utility.h"

#include <gsl/gsl_errno.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_roots.h>
#include <boost/math/tools/roots.hpp>

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

    /*const gsl_root_fsolver_type *solver_type = gsl_root_fsolver_bisection;
    gsl_root_fsolver *solver = gsl_root_fsolver_alloc (solver_type);

    bisection_function = fun;

    gsl_function function;
    function.function = &bisection_adapter;

    gsl_root_fsolver_set(solver, &function, minimum, maximum);

    int status = GSL_CONTINUE;
    double root = 0.0;
    double root_min = minimum;
    double root_max = maximum;
    for (int i = 0; i < max_iterations && status == GSL_CONTINUE; ++i) {
        status = gsl_root_fsolver_iterate(solver);
        if (status != GSL_SUCCESS)
            break;

        root = gsl_root_fsolver_root(solver);
        root_min = gsl_root_fsolver_x_lower(solver);
        root_max = gsl_root_fsolver_x_upper(solver);

        status = gsl_root_test_interval(root_min, root_max, 0, tolerance);
        if (status == GSL_SUCCESS) {
            break;
        }
    }
    gsl_root_fsolver_free(solver);
    return root;
*/
}
