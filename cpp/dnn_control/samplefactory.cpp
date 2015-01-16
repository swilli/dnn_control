#include "samplefactory.h"
#include "constants.h"

SampleFactory::SampleFactory() {
    srand(time(0));
    generator_ = boost::mt19937(rand());
    uniform_distribution_ = boost::random::uniform_real_distribution<>(0.0, 1.0);
    normal_distribution_ = boost::random::normal_distribution<>(0.0, 1.0);
}

SampleFactory::SampleFactory(const unsigned int &random_seed) {
    generator_ = boost::mt19937(random_seed);
    uniform_distribution_ = boost::random::uniform_real_distribution<>(0.0, 1.0);
    normal_distribution_ = boost::random::normal_distribution<>(0.0, 1.0);
}

SampleFactory::~SampleFactory() {

}

boost::random::mt19937& SampleFactory::RandomNumberGenerator() {
    return generator_;
}

double SampleFactory::SampleUniform(const double &minimum, const double &maximum) {
    return minimum + (maximum - minimum) * uniform_distribution_(generator_);
}

double SampleFactory::SampleNormal(const double &mean, const double &variance) {
    return normal_distribution_(generator_) * variance + mean;
}

double SampleFactory::SampleSign() {
    if (rand() % 2) {
        return 1.0;
    } else {
        return -1.0;
    }
}

Vector3D SampleFactory::SamplePointOutSideEllipsoid(const Vector3D &semi_axis, const double &band_width_scale) {
    Vector3D point;

    const double u = SampleUniform(0.0, 2.0 * kPi);
    const double v = SampleUniform(0.0, kPi);
    point[0] = SampleUniform(1.1, band_width_scale) * semi_axis[0] * cos(u) * sin(v);
    point[1] = SampleUniform(1.1, band_width_scale) * semi_axis[1] * sin(u) * sin(v);
    point[2] = SampleUniform(1.1, band_width_scale) * semi_axis[2] * cos(v);
    return point;
}
