#include "simulator.h"
#include "constants.h"
#include <vector>
#include <boost/numeric/odeint.hpp>
namespace odeint = boost::numeric::odeint;

Simulator::Simulator(const Asteroid &asteroid, const double *spacecraft_position, const double *spacecraft_velocity, const double &spacecraft_mass, const double &spacecraft_specific_impulse, const SensorSimulator &sensor_simulator,
const SpacecraftController &spacecraft_controller, const double &control_frequency, std::ostream *result_file) :
    sensor_simulator_(sensor_simulator), spacecraft_controller_(spacecraft_controller), system_(ODESystem(asteroid, spacecraft_position, spacecraft_velocity, spacecraft_mass, k_earth_acceleration * spacecraft_specific_impulse)),
    normal_distribution_(boost::mt19937(time(0)),
                         boost::normal_distribution<>(0.0, 1e-7)) {
    control_interval_ = 1.0 / control_frequency;
    result_file_ = result_file;
}

void Simulator::Run(const double &time) {
    const bool collect_data = result_file_ != NULL;
    const double dt = control_interval_;
    const int iterations = time / dt;

    std::vector<std::vector<double> > *positions = NULL;
    std::vector<std::vector<double> > *heights = NULL;
    if (collect_data) {
        positions = new std::vector<std::vector<double> >(iterations, std::vector<double>(3,0));
        heights = new std::vector<std::vector<double> >(iterations, std::vector<double>(3,0));
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
                positions->at(iteration).at(i) = position[i];
                heights->at(iteration).at(i) = position[i] - surface_point[i];
            }

        }
        SimulatePerturbations(system_.perturbations_acceleration_);
        sensor_simulator_.Simulate(system_.state_, system_.perturbations_acceleration_, current_time, sensor_data);
        spacecraft_controller_.GetThrustForSensorData(sensor_data, system_.thrust_);
        integrator.do_step(system_, system_.state_, current_time, dt);
        current_time += dt;
    }

    if (collect_data) {
        *result_file_ << system_.asteroid_.SemiAxis(0) << "," << system_.asteroid_.SemiAxis(1) << "," << system_.asteroid_.SemiAxis(2) << "," << 1.0 / control_interval_<< "\n";
        for (int i = 0; i < iterations; ++i) {
            std::vector<double> *position = &positions->at(i);
            std::vector<double> *height = &heights->at(i);
            *result_file_ << position->at(0) << "," << position->at(1) << "," << position->at(2) << "," << height->at(0) << "," << height->at(1) << "," << height->at(2) << "\n";
        }
        delete(heights);
        delete(positions);
    }
}

void Simulator::SimulatePerturbations(double *perturbations)
{
    double mass = system_.state_[6];
    for(int i = 0; i < 3; ++i) {
        perturbations[i] = mass * normal_distribution_();
    }
}
