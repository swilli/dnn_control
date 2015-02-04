#ifndef SAMPLEFACTORY_H
#define SAMPLEFACTORY_H

#include "vector.h"

#include <boost/random.hpp>

class SampleFactory {
public:
    SampleFactory();
    SampleFactory(const unsigned int &seed);
    ~SampleFactory();

    unsigned int SampleRandomInteger();

    // X ~ U(minimum, maximum)
    double SampleUniform(const double &minimum, const double &maximum);

    double SampleNormal(const double &mean, const double &standard_deviation);

    // X ~ U({-1,1})
    double SampleSign();

    // Returns a point point, whereas
    // semi_axis_[0] * cos(u) * sin(v) < point[0] < semi_axis[0] * band_width_scale * cos(u) * sin(v)
    // semi_axis_[1] * sin(u) * sin(v) < point[1] < semi_axis[1] * band_width_scale * sin(u) * sin(v)
    // semi_axis_[2] * cos(v)          < point[2] < semi_axis[1] * band_width_scale * cos(v)
    Vector3D SamplePointOutSideEllipsoid(const Vector3D &semi_axis, const double &min_scale, const double &max_scale);

    void SetSeed(const unsigned int &random_seed);

private:
    boost::mt19937 generator_;
    boost::random::uniform_real_distribution<> uniform_distribution_;
    boost::random::normal_distribution<> normal_distribution_;
};

#endif // SAMPLEFACTORY_H
