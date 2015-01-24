#ifndef SAMPLEFACTORY_H
#define SAMPLEFACTORY_H

#include "vector.h"

#include <boost/functional/hash.hpp>
#include <boost/random.hpp>
#include <unordered_map>

template <typename Container> // we can make this generic for any container [1]
struct container_hash {
    std::size_t operator()(Container const& c) const {
        return boost::hash_range(c.begin(), c.end());
    }
};

class SampleFactory {
public:
    SampleFactory();
    SampleFactory(const unsigned int &seed);
    ~SampleFactory();

    unsigned int SampleRandomInteger();
    unsigned int SampleRandomInteger(const double &time, const int &dim=-1);

    // X ~ U(minimum, maximum)
    double SampleUniform(const double &minimum, const double &maximum);
    double SampleUniform(const double &time, const double &minimum, const double &maximum, const int &dim=-1);

    double SampleNormal(const double &mean, const double &variance);
    double SampleNormal(const double &time, const double &mean, const double &variance, const int &dim=-1);

    // X ~ U({-1,1})
    double SampleSign();
    double SampleSign(const double &time, const int &dim=-1);

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

    std::unordered_map<std::vector<double>, double, container_hash<std::vector<double> > > uniform_history_;
    std::unordered_map<std::vector<double>, double, container_hash<std::vector<double> > > normal_history_;
    std::unordered_map<std::vector<double>, double, container_hash<std::vector<double> > > sign_history_;
    std::unordered_map<std::vector<double>, double, container_hash<std::vector<double> > > integer_history_;
};

#endif // SAMPLEFACTORY_H
