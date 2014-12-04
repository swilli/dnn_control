#include "simulator.h"
#include "constants.h"

#include <boost/numeric/odeint.hpp>
namespace odeint = boost::numeric::odeint;

Simulator::Simulator(Asteroid &asteroid, SensorSimulator &sensor_simulator, SpacecraftController &spacecraft_controller, const double &control_frequency, const double &perturbation_noise) :
    sensor_simulator_(sensor_simulator), spacecraft_controller_(spacecraft_controller), system_(ODESystem(asteroid)), normal_distribution_(boost::mt19937(time(0)),boost::normal_distribution<>(0.0, perturbation_noise)) {
    control_frequency_ = control_frequency;
    control_interval_ = 1.0 / control_frequency;
}

void Simulator::InitSpacecraft(const double *position, const double *velocity, const double &mass, const double &specific_impulse)
{
    for (int i = 0; i < 3; ++i) {
        system_.state_[i] = position[i];
        system_.state_[3+i] = velocity[i];
    }
    system_.state_[6] = mass;
    system_.coef_earth_acceleration_mul_specific_impulse_ = specific_impulse * k_earth_acceleration;
}

void Simulator::InitSpacecraftSpecificImpulse(const double &specific_impulse)
{
    system_.coef_earth_acceleration_mul_specific_impulse_ = specific_impulse * k_earth_acceleration;
}

void Simulator::NextState(const State &state, const double *thrust, const double &time, State &next_state)
{
    for (int i = 0; i < 3; ++i) {
        system_.state_[i] = state[i];
        system_.state_[3+i] = state[3+i];
        system_.thrust_[i] = thrust[i];
    }
    system_.state_[6] = state[6];

    odeint::runge_kutta4<State> integrator;
    SimulatePerturbations(system_.perturbations_acceleration_);
    integrator.do_step(system_, system_.state_, time, control_interval_);

    for (int i = 0; i < 7; ++i) {
        next_state[i] = system_.state_[i];
    }
}

int Simulator::Run(const double &time, const bool &log_data) {
    const double dt = control_interval_;
    const int iterations = time / dt;

    if (log_data) {
        log_states_ = std::vector<LogState>(iterations);
    }

    odeint::runge_kutta4<State> integrator;
    double current_time = 0.0;
    double sensor_data[5];
    for(int iteration = 0; iteration < iterations; ++iteration) {
        if (log_data) {
            const Vector3D position = {system_.state_[0], system_.state_[1], system_.state_[2]};
            double distance;
            Vector3D surface_point;
            system_.asteroid_.NearestPointOnSurfaceToPosition(position, surface_point, &distance);
            LogState state;
            for (int i = 0; i < 3; ++i) {
                state.trajectory_position[i] = position[i];
                state.height[i] = position[i] - surface_point[i];
            }
            log_states_.at(iteration) = state;
        }
        SimulatePerturbations(system_.perturbations_acceleration_);
        sensor_simulator_.Simulate(system_.state_, system_.perturbations_acceleration_, current_time, sensor_data);
        spacecraft_controller_.GetThrustForSensorData(sensor_data, system_.thrust_);
        integrator.do_step(system_, system_.state_, current_time, dt);
        current_time += dt;
    }

    return iterations;
}

void Simulator::SimulatePerturbations(double *perturbations)
{
    double mass = system_.state_[6];
    for(int i = 0; i < 3; ++i) {
        perturbations[i] = mass * normal_distribution_();
    }
}

void Simulator::FlushLogToFile(const std::string &path_to_file)
{
    log_file_.open(path_to_file);
    log_file_ << std::setprecision(10);
    log_file_ << system_.asteroid_.SemiAxis(0) << ",\t" << system_.asteroid_.SemiAxis(1) << ",\t" << system_.asteroid_.SemiAxis(2) << ",\t" << control_frequency_ << std::endl;
    for (int i = 0; i < log_states_.size(); ++i) {
        LogState state = log_states_.at(i);
        log_file_ << state.trajectory_position[0] << ",\t" << state.trajectory_position[1] << ",\t" << state.trajectory_position[2] << ",\t" << state.height[0] << ",\t" << state.height[1] << ",\t" << state.height[2] << "\n";
    }
    log_states_.clear();
    log_file_.close();
}
