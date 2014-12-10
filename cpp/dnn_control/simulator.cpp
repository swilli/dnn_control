#include "simulator.h"
#include "constants.h"
#include "odeint.h"

Simulator::Simulator(Asteroid &asteroid, SensorSimulator *sensor_simulator, SpacecraftController *spacecraft_controller, const double &control_frequency, const double &perturbation_noise) :
    sensor_simulator_(sensor_simulator), spacecraft_controller_(spacecraft_controller), system_(ODESystem(asteroid)), normal_distribution_(boost::mt19937(time(0)),boost::normal_distribution<>(0.0, perturbation_noise)) {
    control_frequency_ = control_frequency;
    control_interval_ = 1.0 / control_frequency;
}

Simulator::~Simulator() {
    delete sensor_simulator_;
    delete spacecraft_controller_;
}

void Simulator::InitSpacecraft(const Vector3D &position, const Vector3D &velocity, const double &mass, const double &specific_impulse) {
    // Transform position, velocity and mass to state
    for (int i = 0; i < 3; ++i) {
        system_.state_[i] = position[i];
        system_.state_[3+i] = velocity[i];
    }
    system_.state_[6] = mass;

    // Cache g * Isp
    system_.coef_earth_acceleration_mul_specific_impulse_ = 1.0 / (specific_impulse * k_earth_acceleration);
}

void Simulator::InitSpacecraftSpecificImpulse(const double &specific_impulse) {
    // Cache g * Isp
    system_.coef_earth_acceleration_mul_specific_impulse_ = 1.0 / (specific_impulse * k_earth_acceleration);
}

void Simulator::NextState(const State &state, const Vector3D &thrust, const double &time, State &next_state) {
    using namespace boost::numeric::odeint;

    // Assign state
    for (int i = 0; i < 3; ++i) {
        system_.state_[i] = state[i];
        system_.state_[3+i] = state[3+i];
        system_.thrust_[i] = thrust[i];
    }
    system_.state_[6] = state[6];

    // Simulate perturbations, directly write it to the ode system
    SimulatePerturbations(system_.perturbations_acceleration_);

    // Integrate for one time step control_interval_
    runge_kutta4<State> integrator;
    integrator.do_step(system_, system_.state_, time, control_interval_);

    // Extract new state out of system
    for (int i = 0; i < 7; ++i) {
        next_state[i] = system_.state_[i];
    }
}

boost::tuple<double, std::vector<Vector3D>, std::vector<SensorData> > Simulator::Run(const double &time, const bool &log_sensor_data) {
    using namespace boost::numeric::odeint;

    std::vector<SensorData> logged_sensor_data;
    std::vector<Vector3D> logged_positions;

    const int sensor_dimensions = sensor_simulator_->Dimensions();
    if (sensor_dimensions != spacecraft_controller_->Dimensions()) {
        std::cout << "Warning: sensor simulator and spacecraft controller have different output/input dimensions." << std::endl;
    }
    const double dt = control_interval_;
    const int iterations = time / dt;

    logged_positions = std::vector<Vector3D>(iterations);

    if (log_sensor_data) {
        logged_sensor_data = std::vector<SensorData>(iterations, std::vector<double>(sensor_dimensions,0));
    }

    runge_kutta4<State> integrator;
    double current_time = 0.0;
    SensorData sensor_data(sensor_dimensions,0);

    // Stepwise integration for "iterations" iterations
    try {
        for(int iteration = 0; iteration < iterations; ++iteration) {
            Vector3D &iter_pos = logged_positions.at(iteration);
            iter_pos[0] = system_.state_[0];
            iter_pos[1] = system_.state_[1];
            iter_pos[2] = system_.state_[2];

            // Simulate perturbations, directly write it to the ode system
            SimulatePerturbations(system_.perturbations_acceleration_);

            // Simulate sensor data
            sensor_simulator_->Simulate(system_.state_, system_.perturbations_acceleration_, current_time, sensor_data);

            if (log_sensor_data) {
                // Log sensor data, if enabled
                SensorData &data = logged_sensor_data.at(iteration);
                for (int i = 0; i < sensor_dimensions; ++i) {
                    data[i] = sensor_data[i];
                }
            }

            // Poll controller for control thrust, write it to the ode system
            spacecraft_controller_->GetThrustForSensorData(sensor_data, system_.thrust_);

            // Integrate the system for control_interval_ time
            integrator.do_step(system_, system_.state_, current_time, dt);

            current_time += dt;

            // Check if spacecraft is out of fuel
            if (system_.state_[6] <= 0.0) {
                std::cout << "The spacecraft is out of fuel." << std::endl;
                break;
            }
        }
    } catch (boost::exception const &exception) {
        std::cout << "The spacecraft crashed into the asteroid's surface." << std::endl;
    }

    return boost::make_tuple(current_time, logged_positions, logged_sensor_data);
}

void Simulator::SimulatePerturbations(Vector3D &perturbations) {
    double mass = system_.state_[6];
    for(int i = 0; i < 3; ++i) {
        perturbations[i] = mass * normal_distribution_();
    }
}
