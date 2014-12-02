#include "simulator.h"
#include <iostream>
#include <ctime>
#include <iomanip>

#define WRITE_TO_FILE   1
#define PATH_TO_FILE    "./result.txt"
int main()
{
    std::ofstream *result_file = NULL;
    if (WRITE_TO_FILE) {
        result_file = new std::ofstream();
        result_file->open(PATH_TO_FILE);
        *result_file << std::setprecision(10);
    }

    const double time = 5.0 * 60.0 * 60.0;
    const double control_frequency = 10.0;
    const double spacecraft_position[3] = {6000.0, 3000.0, 2345.0};
    const double spacecraft_velocity[3] = {0.0, 0.0, 0.0};
    const double spacecraft_specific_impulse = 200.0;
    const double spacecraft_mass = 1000.0;
    const double semi_axis[3] = {5000.0, 2567.0, 1235.0};
    const double density = 2000.0;
    const double angular_velocity[3] = {0.0001,0.0, 0.0001};
    const double time_bias = 0.0;

    const Asteroid asteroid(semi_axis, density, angular_velocity, time_bias);
    const SensorSimulator sensor_simulator(asteroid);
    const SpacecraftController spacecraft_controller;

    std::cout << "running simulation ..." << std::endl;
    Simulator simulator(asteroid, spacecraft_position, spacecraft_velocity, spacecraft_mass, spacecraft_specific_impulse, sensor_simulator, spacecraft_controller, control_frequency, result_file);
    clock_t begin = clock();
    simulator.Run(time);
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << time << " seconds simulation time took " << elapsed_secs << " real time to compute (x" << time/elapsed_secs << ")." << std::endl;

    result_file->close();
    delete(result_file);

/*
    const double position[3] = {0.0, 106412.121, 0.0};
    double surface_point[3];
    double distance;
    asteroid.NearestPointOnSurface(position, surface_point, &distance);
    */
    return 0;
}

