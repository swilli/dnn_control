#include "fixedsimulation.h"
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

    SensorSimulatorFullState sensor_simulator(sf_sensor_simulator, asteroid_);
    ControllerFullState controller(spacecraft_maximum_thrust_, target_position_);
    if (controller_parameters_.size()) {
        controller.SetCoefficients(controller_parameters_[0], controller_parameters_[1], controller_parameters_[2]);
    }

    if (sensor_simulator.Dimensions() != controller.Dimensions()) {
        throw SizeMismatchException();
    }

    std::vector<double> evaluated_times;
    std::vector<double> evaluated_masses;
    std::vector<Vector3D> evaluated_positions;
    std::vector<Vector3D> evaluated_velocities;
    std::vector<Vector3D> evaluated_heights;

    SystemState system_state(initial_system_state_);

    Vector3D perturbations_acceleration;
    for (unsigned int i = 0; i < 3; ++i) {
        perturbations_acceleration[i] = sample_factory.SampleNormal(0.0, perturbation_noise_);
    }

    double current_time = 0.0;
    double engine_noise = 0.0;
    Vector3D thrust = {0.0, 0.0, 0.0};
    const unsigned int num_steps = interaction_interval_ / fixed_step_size_;
    odeint::runge_kutta4<SystemState> stepper;
    try {
        while (current_time < simulation_time_) {
            const Vector3D &position = {system_state[0], system_state[1], system_state[2]};
            const Vector3D &velocity = {system_state[3], system_state[4], system_state[5]};
            const double &mass = system_state[6];

            const Vector3D &surf_pos = boost::get<0>(asteroid_.NearestPointOnSurfaceToPosition(position));
            const Vector3D &height = {position[0] - surf_pos[0], position[1] - surf_pos[1], position[2] - surf_pos[2]};

            evaluated_times.push_back(current_time);
            evaluated_masses.push_back(mass);
            evaluated_positions.push_back(position);
            evaluated_velocities.push_back(velocity);
            evaluated_heights.push_back(height);

            const SensorData sensor_data = sensor_simulator.Simulate(system_state, height, perturbations_acceleration, current_time);
            thrust = controller.GetThrustForSensorData(sensor_data);
            engine_noise = sample_factory.SampleNormal(0.0, engine_noise_);
            ODESystem ode_system(asteroid_, perturbations_acceleration, thrust, spacecraft_specific_impulse_, spacecraft_minimum_mass_, engine_noise);
            for (unsigned int i = 0; i < num_steps; ++i) {
                stepper.do_step(ode_system, system_state, current_time, fixed_step_size_);
                current_time += fixed_step_size_;
            }
        }
    } catch (const Asteroid::Exception &exception) {
        std::cout << "The spacecraft crashed into the asteroid's surface." << std::endl;
    } catch (const ODESystem::Exception &exception) {
        std::cout << "The spacecraft is out of fuel." << std::endl;
    }

    return boost::make_tuple(evaluated_times, evaluated_masses, evaluated_positions, evaluated_heights, evaluated_velocities);
}

double FixedSimulation::FixedStepSize() const {
    return fixed_step_size_;
}


