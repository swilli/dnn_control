#include "samplefactory.h"
#include "constants.h"

SampleFactory::SampleFactory() {
    srand(time(0));
    seed_ = rand();
    generator_ = boost::mt19937(seed_);
    uniform_distribution_ = boost::random::uniform_real_distribution<>(0.0, 1.0);
    normal_distribution_ = boost::random::normal_distribution<>(0.0, 1.0);
}

SampleFactory::SampleFactory(const unsigned int &random_seed) {
    seed_ = random_seed;
    generator_ = boost::mt19937(seed_);
    uniform_distribution_ = boost::random::uniform_real_distribution<>(0.0, 1.0);
    normal_distribution_ = boost::random::normal_distribution<>(0.0, 1.0);
}

unsigned int SampleFactory::SampleRandomNatural() {
    return generator_();
}

unsigned int SampleFactory::SampleUniformNatural(const int &minimum, const int &maximum) {
    const double rand_val = SampleUniformReal(0.0, 1.0);
    return minimum + rand_val * (maximum - minimum);
}

double SampleFactory::SampleUniformReal(const double &minimum, const double &maximum) {
    return minimum + (maximum - minimum) * uniform_distribution_(generator_);
}

double SampleFactory::SampleNormal(const double &mean, const double &standard_deviation) {
    return normal_distribution_(generator_) * standard_deviation + mean;
}

double SampleFactory::SampleSign() {
    if (generator_() % 2) {
        return 1.0;
    } else {
        return -1.0;
    }
}

bool SampleFactory::SampleBoolean() {
    if (generator_() % 2) {
        return true;
    } else {
        return false;
    }
}

boost::tuple<Vector3D, double, double, double> SampleFactory::SamplePointOutSideEllipsoid(const Vector3D &semi_axis, const double &min_scale, const double &max_scale) {
    Vector3D point;

    const double u = 2.0 * kPi * SampleUniformReal(0.0, 1.0 - 1e-10);
    const double v = acos(2.0 * SampleUniformReal(0.0, 1.0) - 1.0);
    const double s = SampleUniformReal(min_scale, max_scale);
    point[0] = s * semi_axis[0] * cos(u) * sin(v);
    point[1] = s * semi_axis[1] * sin(u) * sin(v);
    point[2] = s * semi_axis[2] * cos(v);

    return boost::make_tuple(point, u, v, s);
}

boost::tuple<Vector3D, double, double> SampleFactory::SamplePointOnEllipsoidSurface(const Vector3D &semi_axis) {
    Vector3D point;

    const double u = 2.0 * kPi * SampleUniformReal(0.0, 1.0 - 1e-10);
    const double v = acos(2.0 * SampleUniformReal(0.0, 1.0) - 1.0);
    point[0] = semi_axis[0] * cos(u) * sin(v);
    point[1] = semi_axis[1] * sin(u) * sin(v);
    point[2] = semi_axis[2] * cos(v);

    return boost::make_tuple(point, u, v);
}

void SampleFactory::SetSeed(const unsigned int &random_seed) {
    generator_.seed(random_seed);
}

unsigned int SampleFactory::Seed() const {
    return seed_;
}
