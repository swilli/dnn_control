#ifndef SAMPLEFACTORY_H
#define SAMPLEFACTORY_H

#include "vector.h"

#include <boost/tuple/tuple.hpp>
#include <boost/random.hpp>

class SampleFactory {
    /*
    * This class represents a factory from which different distributed samples can be drawn.
    */
public:
    SampleFactory();
    SampleFactory(const unsigned int &seed);


    // X ~ U(N)
    unsigned int SampleRandomNatural();

    // X ~ U(min,max)
    unsigned int SampleUniformNatural(const int &minimum, const int &maximum);

    // X ~ U(minimum, maximum)
    double SampleUniformReal(const double &minimum, const double &maximum);

    // X ~ N(minimum, maximum)
    double SampleNormal(const double &mean, const double &standard_deviation);

    // X ~ U({-1,1})
    double SampleSign();

    // X ~ U({true, false})
    bool SampleBoolean();

    // Returns a point point, whereas
    // semi_axis_[0] * cos(u) * sin(v) * min_scale <= point[0] <= semi_axis[0] * cos(u) * sin(v) * max_scale
    // semi_axis_[1] * sin(u) * sin(v) * min_scale <= point[1] <= semi_axis[1] * sin(u) * sin(v) * max_scale
    // semi_axis_[2] * cos(v) * min_scale          <= point[2] <= semi_axis[1] * cos(v) * max_scale
    boost::tuple<Vector3D, double, double, double> SamplePointOutSideEllipsoid(const Vector3D &semi_axis, const double &min_scale, const double &max_scale);

    // Returns a point point, whereas
    // semi_axis_[0] * cos(u) * sin(v) = point[0]
    // semi_axis_[1] * sin(u) * sin(v) = point[1]
    // semi_axis_[2] * cos(v)          = point[2]
    boost::tuple<Vector3D, double, double> SamplePointOnEllipsoidSurface(const Vector3D &semi_axis);

    // Set the seed of the factory
    void SetSeed(const unsigned int &random_seed);

    // Get the seed of the factory
    unsigned int Seed() const;

private:
    // The seed the random number generator is initialized with
    unsigned int seed_;

    // The random number generator used for producing different samples
    boost::mt19937 generator_;

    // Uniform distribution between [0,1]
    boost::random::uniform_real_distribution<> uniform_distribution_;

    // Normal distribution (0,1)
    boost::random::normal_distribution<> normal_distribution_;
};

#endif // SAMPLEFACTORY_H
