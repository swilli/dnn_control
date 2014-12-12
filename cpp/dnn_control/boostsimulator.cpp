#include "boostsimulator.h"
#include "asteroid.h"
#include "fullstatesensorsimulator.h"
#include "fullstatecontroller.h"

BoostSimulator::BoostSimulator(const bp::list &asteroid_semi_axis, const double &asteroid_density, const bp::list &asteroid_angular_velocity, const double &time_bias,
                               const bp::list &target_position, const double &control_frequency, const double &sensor_noise, const double &perturbation_noise, const double &control_noise) {

    const Vector3D asteroid_semi_axis_cpp = {bp::extract<double>(asteroid_semi_axis[0]),
                                             bp::extract<double>(asteroid_semi_axis[1]),
                                             bp::extract<double>(asteroid_semi_axis[2])};

    const Vector3D asteroid_angular_velocity_cpp = {bp::extract<double>(asteroid_angular_velocity[0]),
                                                    bp::extract<double>(asteroid_angular_velocity[1]),
                                                    bp::extract<double>(asteroid_angular_velocity[2])};


    const Vector3D target_position_cpp = {bp::extract<double>(target_position[0]),
                                          bp::extract<double>(target_position[1]),
                                          bp::extract<double>(target_position[2])};


    Asteroid asteroid(asteroid_semi_axis_cpp, asteroid_density, asteroid_angular_velocity_cpp, time_bias);

    FullStateSensorSimulator *sensor_simulator = new FullStateSensorSimulator(asteroid, sensor_noise);
    FullStateController *spacecraft_controller = new FullStateController(control_frequency, target_position_cpp);

    simulator_cpp_ = new Simulator(asteroid, sensor_simulator, spacecraft_controller, control_frequency, perturbation_noise, control_noise);
}

void BoostSimulator::InitSpacecraft(const boost::python::list &spacecraft_position, const boost::python::list &spacecraft_velocity, const double &spacecraft_mass, const double &spacecraft_specific_impulse) {
    const Vector3D spacecraft_position_cpp = {bp::extract<double>(spacecraft_position[0]),
                                              bp::extract<double>(spacecraft_position[1]),
                                              bp::extract<double>(spacecraft_position[2])};

    const Vector3D spacecraft_velocity_cpp = {bp::extract<double>(spacecraft_velocity[0]),
                                              bp::extract<double>(spacecraft_velocity[1]),
                                              bp::extract<double>(spacecraft_velocity[2])};
    simulator_cpp_->InitSpacecraft(spacecraft_position_cpp, spacecraft_velocity_cpp, spacecraft_mass, spacecraft_specific_impulse);
}

void BoostSimulator::InitSpacecraftSpecificImpulse(const double &specific_impulse) {
    simulator_cpp_->InitSpacecraftSpecificImpulse(specific_impulse);
}

bp::tuple BoostSimulator::Run(const double &time, const bool &log_sensor_data) {
    const boost::tuple<double, std::vector<Vector3D>,  std::vector<Vector3D>, std::vector<SensorData> > result = simulator_cpp_->Run(time, log_sensor_data);
    const double simulation_time = boost::get<0>(result);
    std::vector<Vector3D> positions = boost::get<1>(result);
    std::vector<Vector3D> heights = boost::get<2>(result);
    std::vector<SensorData> sensor_data = boost::get<3>(result);

    bp::list positions_py;
    bp::list heights_py;
    bp::list sensor_data_py;
    for (int i = 0; i < positions.size(); ++i) {
        bp::list result;
        Vector3D &pos = positions.at(i);
        for (int j = 0; j < pos.size(); ++j) {
            result.append(pos.at(j));
        }
        positions_py.append(result);
    }
    for (int i = 0; i < sensor_data.size(); ++i) {
        bp::list result;
        SensorData &data = sensor_data.at(i);
        for (int j = 0; j < data.size(); ++j) {
            result.append(data.at(j));
        }
        sensor_data_py.append(result);
    }
    for (int i = 0; i < heights.size(); ++i) {
        bp::list result;
        Vector3D &h = heights.at(i);
        for (int j = 0; j < h.size(); ++j) {
            result.append(h.at(j));
        }
        heights_py.append(result);
    }

    return make_tuple(simulation_time, positions_py, heights_py, sensor_data_py);
}

bp::list BoostSimulator::NextState(const bp::list state, const bp::list thrust, const double &time) {
    State state_cpp;
    for (int i = 0; i < 7; ++i) {
    	state_cpp[i] = bp::extract<double>(state[i]);
    } 

    const Vector3D thrust_cpp = {bp::extract<double>(thrust[0]),
                                 bp::extract<double>(thrust[1]),
                                 bp::extract<double>(thrust[2])};

    const State next_state_cpp = simulator_cpp_->NextState(state_cpp, thrust_cpp, time);

    bp::list result;
    for (int i = 0; i < 7; ++i) {
        result.append(next_state_cpp[i]);
    }
    return result;
}

BOOST_PYTHON_MODULE(boost_simulator)
{
    bp::class_<BoostSimulator>("BoostSimulator", bp::init<const bp::list &, const double &, const bp::list &, const double &,
                               const bp::list &, const double &, const double &, const double &, const double &>())
            .def("run", &BoostSimulator::Run)
            .def("next_state", &BoostSimulator::NextState)
            .def("init_spacecraft", &BoostSimulator::InitSpacecraft)
            .def("init_spacecraft_specific_impulse", &BoostSimulator::InitSpacecraftSpecificImpulse)
            ;
}

