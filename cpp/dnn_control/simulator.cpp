#include "simulator.h"
#include "constants.h"
#include <boost/numeric/odeint.hpp>
namespace odeint = boost::numeric::odeint;

Simulator::Simulator(const Asteroid &asteroid, const double *spacecraft_position, const double *spacecraft_velocity, const double &spacecraft_mass, const double &spacecraft_specific_impulse, const SensorSimulator &sensor_simulator,
const SpacecraftController &spacecraft_controller, const double &control_frequency) :
    sensor_simulator_(sensor_simulator), spacecraft_controller_(spacecraft_controller), system_(ODESystem(asteroid, spacecraft_position, spacecraft_velocity, spacecraft_mass, k_earth_acceleration * spacecraft_specific_impulse)),
    normal_distribution_(boost::mt19937(time(0)),
                         boost::normal_distribution<>(0.0, 1e-7)) {
    control_interval_ = 1.0 / control_frequency;
}

void Simulator::Run(const double &time, const bool &collect_data) {
    double current_time = 0.0;
    odeint::runge_kutta4<State> integrator;

    double sensor_data[5];
    const double dt = control_interval_;
    while (current_time < time) {
        SimulatePerturbations(system_.perturbations_acceleration_);
        sensor_simulator_.Simulate(system_.state_, system_.perturbations_acceleration_, current_time, sensor_data);
        spacecraft_controller_.GetThrustForSensorData(sensor_data, system_.thrust_);
        integrator.do_step(system_, system_.state_, current_time, dt);
        current_time += dt;
    }
}

void Simulator::SimulatePerturbations(double *perturbations)
{
    double mass = system_.state_[6];
    for(int i = 0; i < 3; ++i) {
        perturbations[i] = mass * normal_distribution_();
    }
}
