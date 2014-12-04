
#include "simulator.h"
#include "test.h"
#include "vector.h"
#include "utility.h"

#include <iostream>
#include <ctime>
#include <iomanip>

#define COLLECT_DATA   1
#define PATH_TO_FILE    "../result.txt"

int main(int argc, char *argv[])
{
    srand(time(NULL));

    //UnitTestTrajectory();
    //UnitTestAngularVelocity();
    //return 0;

    std::ofstream result_file;
    if (COLLECT_DATA) {
        result_file.open(PATH_TO_FILE);
        result_file << std::setprecision(10);
    }

    const double time = 24.0 * 60.0 * 60.0;

    const Vector3D semi_axis = {10000.0, 6000.0, 4000.0};
    const double density = 2215.0;
    const Vector3D angular_velocity = {0.0002, 0.0, 0.0008};
    const double time_bias = 0.0;

    Vector3D spacecraft_position;
    SamplePointOutSideEllipsoid(semi_axis, 4.0, spacecraft_position);
    Vector3D spacecraft_velocity;

    CrossProduct(angular_velocity, spacecraft_position, spacecraft_velocity);
    spacecraft_velocity[0] *= -1; spacecraft_velocity[1] *= -1; spacecraft_velocity[2] *= -1;
    const double spacecraft_specific_impulse = 200.0;
    const double spacecraft_mass = 1000.0;

    const double control_frequency = 10.0;

    const double sensor_noise = 0.05;
    const double perturbation_noise = 1e-7;

    Asteroid asteroid(semi_axis, density, angular_velocity, time_bias);
    SensorSimulator sensor_simulator(asteroid, sensor_noise);
    SpacecraftController spacecraft_controller;

    std::vector<std::vector<double> > *positions = NULL;
    std::vector<std::vector<double> > *heights = NULL;

    std::cout << "running simulation ..." << std::endl;
    Simulator simulator(asteroid, sensor_simulator, spacecraft_controller, control_frequency, perturbation_noise);
    simulator.InitSpacecraft(spacecraft_position, spacecraft_velocity, spacecraft_mass, spacecraft_specific_impulse);
    clock_t begin = clock();
    const int iterations = simulator.Run(time, COLLECT_DATA, &positions, &heights);
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << time << " seconds simulation time took " << elapsed_secs << " real time to compute (x" << time/elapsed_secs << ")." << std::endl;

    if(COLLECT_DATA) {
        std::cout << "writing results to file ... ";
        result_file << asteroid.SemiAxis(0) << "," << asteroid.SemiAxis(1) << "," << asteroid.SemiAxis(2) << "," << control_frequency << std::endl;
        for (int i = 0; i < iterations; ++i) {
            std::vector<double> *position = &positions->at(i);
            std::vector<double> *height = &heights->at(i);
            result_file << position->at(0) << "," << position->at(1) << "," << position->at(2) << "," << height->at(0) << "," << height->at(1) << "," << height->at(2) << "\n";
        }
        delete(heights);
        delete(positions);
        result_file.close();
        std::cout << "done." << std::endl;
    }

    return 0;
}

