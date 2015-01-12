#include "simulator.h"
#include <boost/python.hpp>
#include <boost/python/tuple.hpp>

namespace bp = boost::python;

class BoostSimulator
{
public:
    BoostSimulator(const bp::list &asteroid_semi_axis, const double &asteroid_density, const bp::list &asteroid_angular_velocity_xz, const double &time_bias,
                   const bool &full_state_controlled, const std::string &controlling_type, const bp::list &target_position, const double &maximum_thrust, const double &control_frequency, 
                   const bp::list &sensor_noise, const double &perturbation_noise, const double &control_noise);
    ~BoostSimulator();

    void InitSpacecraft(const bp::list &spacecraft_position, const bp::list &spacecraft_velocity, const double &spacecraft_mass, const double &spacecraft_specific_impulse);

    void InitSpacecraftSpecificImpulse(const double &specific_impulse);

    bp::tuple Run(const double &time, const bool &log_sensor_data);

    bp::list NextState(const bp::list state, const bp::list thrust, const double &time);

    double ControlFrequency() const;

private:
	// cpp simulator
    Simulator *simulator_cpp_;
};
