#include "simulator.h"
#include "fullstatesensorsimulator.h"
#include "fullstatecontroller.h"
#include "test.h"
#include "vector.h"
#include "utility.h"

#include <iostream>
#include <ctime>

#define WRITE_TO_FILE   1
#define PATH_TO_FILE    "../../../results/states.txt"

int main(int argc, char *argv[]) {
    srand(time(NULL));

    //UnitTestTrajectory();
    //UnitTestAngularVelocity();
    //UnitTestAny();
    //UnitTestGravity();
    //return 0;

    const double time = 24.0 * 60.0 * 60.0;

    const Vector3D semi_axis = {SampleUniform(8000.0,12000.0), SampleUniform(4000.0, 7500.0), SampleUniform(1000.0, 3500.0)};
    const double density = SampleUniform(1500.0,3000.0);
    const Vector3D angular_velocity = {SampleSign() * SampleUniform(0.0002, 0.0008), 0.0, SampleSign() * SampleUniform(0.0002, 0.0008)};
    const double time_bias = 0.0;

    const Vector3D spacecraft_position = SamplePointOutSideEllipsoid(semi_axis, 4.0);

    Vector3D spacecraft_velocity = CrossProduct(angular_velocity, spacecraft_position);
    spacecraft_velocity[0] *= -1; spacecraft_velocity[1] *= -1; spacecraft_velocity[2] *= -1;

    const double spacecraft_specific_impulse = 200.0;
    const double spacecraft_mass = 1000.0;

    const double control_frequency = 10.0;

    Vector3D target_position;
    for (int i = 0; i < 3; ++i) {
        target_position[i] = SampleUniform(spacecraft_position[i] - 500.0, spacecraft_position[i] + 500.0);
    }

    const double sensor_noise = 0.05;
    const double perturbation_noise = 1e-7;

    Asteroid asteroid(semi_axis, density, angular_velocity, time_bias);
    FullStateSensorSimulator *sensor_simulator = new FullStateSensorSimulator(asteroid, sensor_noise);
    FullStateController *spacecraft_controller = new FullStateController(control_frequency, target_position);

    std::cout << "running simulation ..." << std::endl;
    Simulator simulator(asteroid, sensor_simulator, spacecraft_controller, control_frequency, perturbation_noise);
    simulator.InitSpacecraft(spacecraft_position, spacecraft_velocity, spacecraft_mass, spacecraft_specific_impulse);
    clock_t begin = clock();
    const double simulated_time = simulator.Run(time, WRITE_TO_FILE);
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << simulated_time << " seconds simulation time took " << elapsed_secs << " real time to compute (x" << simulated_time/elapsed_secs << ")." << std::endl;

    if(WRITE_TO_FILE) {
        std::cout << "writing results to file ... ";
        simulator.FlushLogToFile(PATH_TO_FILE);
        std::cout << "done." << std::endl;
    }

    return 0;
}

