#include "fixedsimulation.h"
#include "datacollector.h"
#include "odeint.h"
#include "odesystem.h"

// DEFINE SENSORSIMULATOR AND CONTROLLER HERE
#include "sensorsimulatorfullstate.h"
#include "controllerfullstate.h"

FixedSimulation::FixedSimulation(const unsigned int &random_seed)
    : Simulation(random_seed) {
    fixed_step_size_ = 0.1;
}

FixedSimulation::~FixedSimulation() {

}

boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > FixedSimulation::Evaluate() {
    SampleFactory sample_factory(random_seed_);
    SampleFactory sf_sensor_simulator(sample_factory.SampleRandomInteger());
    SampleFactory sf_odesystem(sample_factory.SampleRandomInteger());

    SensorSimulatorFullState sensor_simulator(sf_sensor_simulator, asteroid_);
    ControllerFullState controller(spacecraft_maximum_thrust_, target_position_);

    if (sensor_simulator.Dimensions() != controller.Dimensions()) {
        throw SizeMismatchException();
    }

    std::vector<double> time_points;
    std::vector<double> evaluated_masses;
    std::vector<Vector3D> evaluated_positions;
    std::vector<Vector3D> evaluated_velocities;
    std::vector<Vector3D> evaluated_heights;

    DataCollector collector(asteroid_, time_points, evaluated_masses, evaluated_positions, evaluated_heights, evaluated_velocities);
    SystemState system_state(initial_system_state_);


    ODESystem sys(sf_odesystem, asteroid_, sensor_simulator, controller, spacecraft_specific_impulse_, perturbation_noise_, engine_noise_);

    odeint::runge_kutta4<SystemState> stepper;
    try {
         integrate_const(stepper , sys, system_state, 0.0, simulation_time_, fixed_step_size_, collector);
    } catch (const Asteroid::Exception &exception) {
        std::cout << "The spacecraft crashed into the asteroid's surface." << std::endl;
    } catch (const ODESystem::Exception &exception) {
         std::cout << "The spacecraft is out of fuel." << std::endl;
    }

    return boost::make_tuple(time_points, evaluated_masses, evaluated_positions, evaluated_heights, evaluated_velocities);
}

double FixedSimulation::FixedStepSize() const {
    return fixed_step_size_;
}


