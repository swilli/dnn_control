#include "test.h"
#include "asteroid.h"
#include "utility.h"
#include "odesystem.h"
#include "simulator.h"

#include <random>
#include <boost/numeric/odeint.hpp>
#include <iostream>
#include <iomanip>

#define PATH_TO_FILE    "../result.txt"

namespace odeint = boost::numeric::odeint;

void UnitTestAngularVelocity() {
    const Vector3D semi_axis = {10000.0, 6000.0, 4000.0};
    const double density = 2215.0;
    const Vector3D angular_velocity = {-0.0002, 0.0, -0.0008};
    const double time_bias = 0.0;

    Asteroid asteroid(semi_axis, density, angular_velocity, time_bias);

    std::random_device rd;
    std::mt19937 generator(rd());
    std::uniform_real_distribution<double> distribution(0.0, 100.0);

    typedef double State[3];

    struct System {
        Vector3D inertia;
        void operator()(const State &state, State &d_state_dt, double time) const {
            d_state_dt[0] = (inertia[1] - inertia[2]) * state[1] * state[2] / inertia[0];
            d_state_dt[1] = (inertia[2] - inertia[0]) * state[2] * state[0] / inertia[1];
            d_state_dt[2] = (inertia[0] - inertia[1]) * state[0] * state[1] / inertia[2];
        };
        System(const double *para_inertia) {
            for (int i = 0; i < 3; ++i) {
                inertia[i] = para_inertia[i];
            }
        };
    };

    const Vector3D inertia = {asteroid.Inertia(0), asteroid.Inertia(1), asteroid.Inertia(2)};
    System sys(inertia);

    const int num_test_cases = 10000;
    double min_error = 1e20;
    double max_error = 1e-20;
    double avg_error = 0.0;
    for (int i = 0; i < num_test_cases; ++i) {
        Vector3D omega_analytical;
        Vector3D d_omega_dt_analytical;
        const double time = distribution(generator);
        asteroid.AngularVelocityAndAccelerationAtTime(time, omega_analytical, d_omega_dt_analytical);

        State omega_numerical = {angular_velocity[0], 0.0, angular_velocity[2]};

        odeint::runge_kutta4<State> integrator;
        const double dt = 0.1;
        integrate_const(integrator, sys, omega_numerical, 0.0, time, dt);

        double error = 0.0;
        for (int i = 0; i < 3; ++i) {
            error += (omega_analytical[i] - omega_numerical[i]) * (omega_analytical[i] - omega_numerical[i]);
        }
        error = sqrt(error);

        if (error < min_error) {
            min_error = error;
        } else {
            max_error = error;
        }
        avg_error += error;
    }

    std::cout << std::setprecision(10) << "min: " << min_error << std::endl << "max: " << max_error << std::endl << "avg: " << avg_error/(double)num_test_cases << std::endl;
}

void UnitTestTrajectory() {
    std::cout << "looking for strange physics..." << std::setprecision(10) << std::endl;

    const double time = 1.0 * 60.0 * 60.0;

    const Vector3D semi_axis = {10000.0, 6000.0, 4000.0};
    const double density = 2215.0;
    const Vector3D angular_velocity = {-0.0002, 0.0, 0.0008};
    const double time_bias = 0.0;

    const double band_width_scaling = 4.0;

    const double spacecraft_specific_impulse = 200.0;
    const double spacecraft_mass = 1000.0;

    const double control_frequency = 10.0;

    const double sensor_noise = 0.05;
    const double perturbation_noise = 0.0;

    Asteroid asteroid(semi_axis,density, angular_velocity, time_bias);
    SensorSimulator sensor_simulator(asteroid, sensor_noise);
    SpacecraftController spacecraft_controller;
    Simulator simulator(asteroid, sensor_simulator, spacecraft_controller, control_frequency, perturbation_noise);
    simulator.InitSpacecraftSpecificImpulse(spacecraft_specific_impulse);

    const int num_test_cases = 1000000;
    for (int i = 0; i < num_test_cases; ++i) {
        Vector3D position;
        SamplePointOutSideEllipsoid(semi_axis, band_width_scaling, position);

        Vector3D velocity;
        CrossProduct(angular_velocity, position, velocity);
        velocity[0] *= -1;
        velocity[1] *= -1;
        velocity[2] *= -1;

        State state;
        for(int i = 0; i < 3; ++i) {
            state[i] = position[i];
            state[3+i] = velocity[i];
        }
        state[6] = spacecraft_mass;

        State next_state;
        const Vector3D thrust = {0.0, 0.0, 0.0};
        simulator.NextState(state, thrust, 0.0, next_state);

        const Vector3D next_position = {next_state[0], next_state[1], next_state[2]};
        double norm_pos = 0.0;
        double norm_next_pos = 0.0;
        for (int i = 0; i < 3; ++i) {
            norm_pos += position[i] * position[i];
            norm_next_pos += next_position[i] * next_position[i];
        }
        norm_pos = sqrt(norm_pos);
        norm_next_pos = sqrt(norm_next_pos);
        if(norm_pos < norm_next_pos) {
            Vector3D gravity;
            asteroid.GravityAtPosition(position, gravity);
            gravity[0] /= state[6];
            gravity[1] /= state[6];
            gravity[2] /= state[6];

            Vector3D angular_velocity;
            Vector3D angular_acceleration;
            asteroid.AngularVelocityAndAccelerationAtTime(0.0, angular_velocity, angular_acceleration);

            Vector3D centrifugal_acceleration;
            Vector3D tmp;
            CrossProduct(angular_velocity, position, tmp);
            CrossProduct(angular_velocity, tmp, centrifugal_acceleration);

            std::cout << "gravity at position: (" << gravity[0] << "," << gravity[1] << "," << gravity[2] << ")" << std::endl;
            std::cout << "centrifugal force at position: (" << centrifugal_acceleration[0] << "," << centrifugal_acceleration[1] << "," << centrifugal_acceleration[2] << ")" << std::endl;

            simulator.InitSpacecraft(position, velocity, spacecraft_mass, spacecraft_specific_impulse);
            std::vector<std::vector<double> > *positions = NULL;
            std::vector<std::vector<double> > *heights = NULL;
            const int iterations = simulator.Run(time, true, &positions, &heights);

            std::ofstream result_file;
            result_file.open(PATH_TO_FILE);
            result_file << std::setprecision(10);
            result_file << asteroid.SemiAxis(0) << "," << asteroid.SemiAxis(1) << "," << asteroid.SemiAxis(2) << "," << control_frequency << std::endl;

            for (int j = 0; j < iterations; ++j) {
                std::vector<double> *position = &positions->at(j);
                std::vector<double> *height = &heights->at(j);
                result_file << position->at(0) << "," << position->at(1) << "," << position->at(2) << "," << height->at(0) << "," << height->at(1) << "," << height->at(2) << "\n";
            }
            delete(heights);
            delete(positions);
            result_file.close();
            std::cout << "-> check the result.txt file..." << std::endl;
            break;
        }
    }
    std::cout << "done." << std::endl;
}
