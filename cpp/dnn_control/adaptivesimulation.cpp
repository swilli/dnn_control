#include "adaptivesimulation.h"
#include "datacollector.h"
#include "modifiedcontrolledrungekutta.h"
#include "odeint.h"
#include "odesystem.h"

// DEFINE SENSORSIMULATOR AND CONTROLLER HERE
#include "sensorsimulatorfullstate.h"
#include "controllerfullstate.h"

AdaptiveSimulation::AdaptiveSimulation(const unsigned int &random_seed)
    : Simulation(random_seed) {
    minimum_step_size_ = 0.1;
}

AdaptiveSimulation::~AdaptiveSimulation() {

}

boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > AdaptiveSimulation::Evaluate() {
    SampleFactory sample_factory(random_seed_);
    SampleFactory sf_sensor_simulator(sample_factory.SampleRandomInteger());
    SampleFactory sf_odesystem(sample_factory.SampleRandomInteger());

    SensorSimulatorFullState sensor_simulator(sf_sensor_simulator, asteroid_);
    ControllerFullState controller(spacecraft_maximum_thrust_, target_position_);

    std::vector<double> time_points;
    std::vector<double> evaluated_masses;
    std::vector<Vector3D> evaluated_positions;
    std::vector<Vector3D> evaluated_velocities;
    std::vector<Vector3D> evaluated_heights;

    DataCollector collector(asteroid_, time_points, evaluated_masses, evaluated_positions, evaluated_heights, evaluated_velocities);
    SystemState system_state(initial_system_state_);

    typedef odeint::runge_kutta_cash_karp54<SystemState> ErrorStepper;
    typedef odeint::modified_controlled_runge_kutta<ErrorStepper> ControlledStepper;
    ControlledStepper controlled_stepper;

    ODESystem sys(sf_odesystem, asteroid_, sensor_simulator, controller, spacecraft_specific_impulse_, perturbation_noise_, engine_noise_);

    try {
        integrate_adaptive(controlled_stepper , sys, system_state, 0.0, simulation_time_, minimum_step_size_, collector);
    } catch (const Asteroid::Exception &exception) {
        std::cout << "The spacecraft crashed into the asteroid's surface." << std::endl;
    } catch (const ODESystem::Exception &exception) {
        std::cout << "The spacecraft is out of fuel." << std::endl;
   }

    return boost::make_tuple(time_points, evaluated_masses, evaluated_positions, evaluated_heights, evaluated_velocities);
}

double AdaptiveSimulation::MinimumStepSize() const {
    return minimum_step_size_;
}
