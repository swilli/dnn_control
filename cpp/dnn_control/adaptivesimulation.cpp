#include "adaptivesimulation.h"
#include "odeint.h"
#include "odesystem.h"
#include "modifiedcontrolledrungekutta.h"

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
    typedef odeint::runge_kutta_cash_karp54<SystemState> ErrorStepper;
    typedef odeint::modified_controlled_runge_kutta<ErrorStepper> ControlledStepper;

    SampleFactory sample_factory(random_seed_);
    SampleFactory sf_sensor_simulator(sample_factory.SampleRandomInteger());

    SensorSimulatorFullState sensor_simulator(sf_sensor_simulator, asteroid_);
    ControllerFullState controller(spacecraft_maximum_thrust_, target_position_);
    if (controller_parameters_.size()) {
        controller.SetCoefficients(controller_parameters_[0], controller_parameters_[1], controller_parameters_[2]);
    }

    if (sensor_simulator.Dimensions() != controller.Dimensions()) {
        throw SizeMismatchException();
    }

    const unsigned int num_iterations = simulation_time_ / interaction_interval_;

    std::vector<double> evaluated_times(num_iterations + 1);
    std::vector<double> evaluated_masses(num_iterations + 1);
    std::vector<Vector3D> evaluated_positions(num_iterations + 1);
    std::vector<Vector3D> evaluated_velocities(num_iterations + 1);
    std::vector<Vector3D> evaluated_heights(num_iterations + 1);

    SystemState system_state(initial_system_state_);

    double current_time = 0.0;
    double current_time_observer = 0.0;
    Observer observer(current_time_observer);
    unsigned int iteration = 0;
    bool exception_thrown = false;
    try {
        for (iteration = 0; iteration < num_iterations; ++iteration) {
            const Vector3D &position = {system_state[0], system_state[1], system_state[2]};
            const Vector3D &velocity = {system_state[3], system_state[4], system_state[5]};
            const double &mass = system_state[6];

            const Vector3D &surf_pos = boost::get<0>(asteroid_.NearestPointOnSurfaceToPosition(position));
            const Vector3D &height = {position[0] - surf_pos[0], position[1] - surf_pos[1], position[2] - surf_pos[2]};

            evaluated_times[iteration] = current_time;
            evaluated_masses[iteration] = mass;
            evaluated_positions[iteration] = position;
            evaluated_velocities[iteration] = velocity;
            evaluated_heights[iteration] = height;

            Vector3D perturbations_acceleration;
            for (unsigned int i = 0; i < 3; ++i) {
                perturbations_acceleration[i] = mass * sample_factory.SampleNormal(perturbation_mean_, perturbation_noise_);
            }

            const SensorData sensor_data = sensor_simulator.Simulate(system_state, height, perturbations_acceleration, current_time);
            const Vector3D thrust = controller.GetThrustForSensorData(sensor_data);
            const double engine_noise = sample_factory.SampleNormal(0.0, spacecraft_engine_noise_);

            ODESystem ode_system(asteroid_, perturbations_acceleration, thrust, spacecraft_specific_impulse_, spacecraft_minimum_mass_, engine_noise);

            ControlledStepper controlled_stepper;
            integrate_adaptive(controlled_stepper, ode_system, system_state, current_time, current_time + interaction_interval_, minimum_step_size_, observer);

            current_time += interaction_interval_;
        }
    } catch (const Asteroid::Exception &exception) {
        std::cout << "The spacecraft crashed into the asteroid's surface." << std::endl;
        exception_thrown = true;
    } catch (const ODESystem::Exception &exception) {
        std::cout << "The spacecraft is out of fuel." << std::endl;
        exception_thrown = true;
    }
    if (exception_thrown) {
        const unsigned int new_size = iteration + 2;
        evaluated_times.resize(new_size);
        evaluated_masses.resize(new_size);
        evaluated_positions.resize(new_size);
        evaluated_velocities.resize(new_size);
        evaluated_heights.resize(new_size);
    }

    const Vector3D &position = {system_state[0], system_state[1], system_state[2]};
    const Vector3D &velocity = {system_state[3], system_state[4], system_state[5]};
    const double &mass = system_state[6];

    const Vector3D &surf_pos = boost::get<0>(asteroid_.NearestPointOnSurfaceToPosition(position));
    const Vector3D &height = {position[0] - surf_pos[0], position[1] - surf_pos[1], position[2] - surf_pos[2]};

    evaluated_times.back() = current_time_observer;
    evaluated_masses.back() = mass;
    evaluated_positions.back() = position;
    evaluated_velocities.back() = velocity;
    evaluated_heights.back() = height;

    return boost::make_tuple(evaluated_times, evaluated_masses, evaluated_positions, evaluated_heights, evaluated_velocities);
}

double AdaptiveSimulation::MinimumStepSize() const {
    return minimum_step_size_;
}
