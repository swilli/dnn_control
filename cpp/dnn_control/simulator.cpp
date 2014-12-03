#include "simulator.h"
#include "constants.h"

#include <vector>
#include <boost/numeric/odeint.hpp>
namespace odeint = boost::numeric::odeint;

Simulator::Simulator(Asteroid &asteroid, SensorSimulator &sensor_simulator, SpacecraftController &spacecraft_controller, const double &control_frequency, const double &perturbation_noise) :
    sensor_simulator_(sensor_simulator), spacecraft_controller_(spacecraft_controller), system_(ODESystem(asteroid)), normal_distribution_(boost::mt19937(time(0)),boost::normal_distribution<>(0.0, perturbation_noise)) {
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

int Simulator::Run(const double &time, const bool &collect_data, std::vector<std::vector<double> > **positions, std::vector<std::vector<double> > **heights) {
    const double dt = control_interval_;
    const int iterations = time / dt;

    if (collect_data) {
        *positions = new std::vector<std::vector<double> >(iterations, std::vector<double>(3,0));
        *heights = new std::vector<std::vector<double> >(iterations, std::vector<double>(3,0));
    }

    odeint::runge_kutta4<State> integrator;
    double current_time = 0.0;
    double sensor_data[5];
    for(int iteration = 0; iteration < iterations; ++iteration) {
        if (collect_data) {
            const double position[3] = {system_.state_[0], system_.state_[1], system_.state_[2]};
            double surface_point[3];
            double distance;
            system_.asteroid_.NearestPointOnSurface(position, surface_point, &distance);
            for (int i = 0; i < 3; ++i) {
                (*positions)->at(iteration).at(i) = position[i];
                (*heights)->at(iteration).at(i) = position[i] - surface_point[i];
            }

        }
        SimulatePerturbations(system_.perturbations_acceleration_);
        sensor_simulator_.Simulate(system_.state_, system_.perturbations_acceleration_, current_time, sensor_data);
        spacecraft_controller_.GetThrustForSensorData(sensor_data, system_.thrust_);
        integrator.do_step(system_, system_.state_, current_time, dt);
        current_time += dt;
    }

    return iterations;
}

double Simulator::ControlInterval() const
{
    return control_interval_;
}

void Simulator::SimulatePerturbations(double *perturbations)
{
    double mass = system_.state_[6];
    for(int i = 0; i < 3; ++i) {
        perturbations[i] = mass * normal_distribution_();
    }
}
