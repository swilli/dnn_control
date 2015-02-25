#ifndef SAMPLEFACTORY_H
#define SAMPLEFACTORY_H

#include "vector.h"

#include <boost/random.hpp>

class SampleFactory {
    /*
    * This class represents a factory from which different distributed samples can be drawn.
    */
public:
    SampleFactory();
    SampleFactory(const unsigned int &seed);
    ~SampleFactory();

    // X ~ U(N) 
    unsigned int SampleRandomInteger();

    // X ~ U(minimum, maximum)
    double SampleUniform(const double &minimum, const double &maximum);

    // X ~ N(minimum, maximum)    
    double SampleNormal(const double &mean, const double &standard_deviation);

    // X ~ U({-1,1})
    double SampleSign();

    // Returns a point point, whereas
    // semi_axis_[0] * cos(u) * sin(v) * min_scale <= point[0] <= semi_axis[0] * cos(u) * sin(v) * max_scale
    // semi_axis_[1] * sin(u) * sin(v) * min_scale <= point[1] <= semi_axis[1] * sin(u) * sin(v) * max_scale
    // semi_axis_[2] * cos(v) * min_scale          <= point[2] <= semi_axis[1] * cos(v) * max_scale
    Vector3D SamplePointOutSideEllipsoid(const Vector3D &semi_axis, const double &min_scale, const double &max_scale);

    // Returns a point point, whereas
    // semi_axis_[0] * cos(u) * sin(v) = point[0]
    // semi_axis_[1] * sin(u) * sin(v) = point[1]
    // semi_axis_[2] * cos(v)          = point[2]
    Vector3D SamplePointOnEllipsoidSurface(const Vector3D &semi_axis);

    // Set the seed of the factory
    void SetSeed(const unsigned int &random_seed);

private:    
    // The random number generator used for producing different samples
    boost::mt19937 generator_;

    // Normal distribution (0,1)
    boost::random::uniform_real_distribution<> uniform_distribution_;

    // Uniform distribution between [0,1]
    boost::random::normal_distribution<> normal_distribution_;
};

#endif // SAMPLEFACTORY_H
