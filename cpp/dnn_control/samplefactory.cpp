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

unsigned int SampleFactory::SampleRandomInteger() {
    return generator_();
}

unsigned int SampleFactory::SampleRandomInteger(const double &time, const int &dim) {
    const std::vector<double> key = {time, (double) dim};
    unsigned int sample = 0;
    if (integer_history_.find(key) == integer_history_.end()) {
        sample = SampleRandomInteger();
        integer_history_[key] = sample;
    } else {
        sample = integer_history_[key];
    }
    return sample;
}

double SampleFactory::SampleUniform(const double &minimum, const double &maximum) {
    return minimum + (maximum - minimum) * uniform_distribution_(generator_);
}

double SampleFactory::SampleUniform(const double &time, const double &minimum, const double &maximum, const int &dim) {
    std::vector<double> key = {time, (double) dim, minimum, maximum};
    double sample = 0.0;
    if (uniform_history_.find(key) == uniform_history_.end()) {
        sample = SampleUniform(minimum, maximum);
        uniform_history_[key] = sample;
    } else {
        sample = uniform_history_[key];
    }
    return sample;
}

double SampleFactory::SampleNormal(const double &mean, const double &variance) {
    return normal_distribution_(generator_) * variance + mean;
}

double SampleFactory::SampleNormal(const double &time, const double &mean, const double &variance, const int &dim) {
    std::vector<double> key = {time, (double) dim, mean, variance};
    double sample = 0.0;
    if (normal_history_.find(key) == normal_history_.end()) {
        sample = SampleNormal(mean, variance);
        normal_history_[key] = sample;
    } else {
        sample = normal_history_[key];
    }
    return sample;
}

double SampleFactory::SampleSign() {
    if (generator_() % 2) {
        return 1.0;
    } else {
        return -1.0;
    }
}

double SampleFactory::SampleSign(const double &time, const int &dim) {
    const std::vector<double> key = {time, (double) dim};
    double sample = 0.0;
    if (sign_history_.find(key) == sign_history_.end()) {
        sample = SampleSign();
        sign_history_[key] = sample;
    } else {
        sample = sign_history_[key];
    }
    return sample;
}

Vector3D SampleFactory::SamplePointOutSideEllipsoid(const Vector3D &semi_axis, const double &min_scale, const double &max_scale) {
    Vector3D point;

    const double u = SampleUniform(0.0, 2.0 * kPi);
    const double v = SampleUniform(0.0, kPi);
    point[0] = SampleUniform(min_scale, max_scale) * semi_axis[0] * cos(u) * sin(v);
    point[1] = SampleUniform(min_scale, max_scale) * semi_axis[1] * sin(u) * sin(v);
    point[2] = SampleUniform(min_scale, max_scale) * semi_axis[2] * cos(v);
    return point;
}

void SampleFactory::SetSeed(const unsigned int &random_seed) {
    generator_.seed(random_seed);
}
