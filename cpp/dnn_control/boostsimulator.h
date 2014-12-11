#include "simulator.h"
#include <boost/python.hpp>
#include <boost/python/tuple.hpp>

namespace bp = boost::python;

class BoostSimulator
{
public:
    BoostSimulator(const bp::list &asteroid_semi_axis, const double &asteroid_density, const bp::list &asteroid_angular_velocity, const double &time_bias,
                   const bp::list &spacecraft_position, const bp::list &spacecraft_velocity, const double &spacecraft_mass, const double &spacecraft_specific_impulse,
                   const bp::list &target_position, const double &control_frequency, const double &sensor_noise, const double &perturbation_noise);

    bp::tuple Run(const double &time, const bool &log_sensor_data);

    bp::list NextState(const bp::list state, const bp::list thrust, const double &time);
    
private:
    Simulator *simulator_cpp_;
};
