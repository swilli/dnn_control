#ifndef UTILITY_H
#define UTILITY_H

class Functor {
public:
    virtual double Evaluate(double value) {
        return value;
    }
};

inline void CrossProduct(double *vector_u, double *vector_v, double *crossed) {
    crossed[0] = vector_u[1] * vector_v[2] - vector_u[2] * vector_v[1];
    crossed[1] = vector_u[2] * vector_v[0] - vector_u[0] * vector_v[2];
    crossed[2] = vector_u[0] * vector_v[1] - vector_u[1] * vector_v[0];
}

/*static double (*bisection_function)(double);
static double bisection_adapter(double value, void *params) {
    return bisection_function(value);
}
*/


double Bisection(Functor *container, const double &minimum, const double &maximum, const int &max_iterations, const double &tolerance);


#endif // UTILITY_H
