#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "asteroid.h"
#include "sensorsimulator.h"
#include "spacecraftcontroller.h"
#include "odesystem.h"

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <vector>
#include <fstream>

class Simulator {
    /*
     * This class implements the interaction between stepwise integration of the ode system, artificial sensor data generation, control polling, and result logging.
     */
public:
    Simulator(Asteroid &asteroid, SensorSimulator *sensor_simulator, SpacecraftController *spacecraft_controller, const double &control_frequency, const double &perturbation_noise);
    ~Simulator();

    // Before running the simulator, it has to be initialized with a spacecraft configuration
    void InitSpacecraft(const Vector3D &position, const Vector3D &velocity, const double &mass, const double &specific_impulse);

    // Initialize spacecraft only by specifying the spacecraft's specific impulse (useful for the function NextState)
    void InitSpacecraftSpecificImpulse(const double &specific_impulse);

    // Implements F: S x A x T -> S : F(s,a,t) = s' (Useful for RL?)
    void NextState(const State &state, const Vector3D &thrust, const double &time, State &next_state);

    // Simulates the system for time "time". Logs the states if "log_data" is enabled, Returns the number of iterations the simulator made to get to the specified time.
    double Run(const double &time, const bool &log_data);

    // Writes the logged data from Run to the file given by "path_to_file"
    void FlushLogToFile(const std::string &path_to_file);


private:
    // Simulates random noise in the dynamical system (which is fed into the ODE system "system_")
    void SimulatePerturbations(Vector3D &perturbations);

    // Ordinary differential system which implements the spacecraft's dynamics
    ODESystem system_;

    // Simulator which generates artificial sensor data for the controller
    SensorSimulator *sensor_simulator_;

    // Controller which generates control thrust given sensor data
    SpacecraftController *spacecraft_controller_;

    // normal_distribution_ ~ N(0, perturbation_noise), whereas perturbation_noise is given in the constructor
    boost::variate_generator<boost::mt19937, boost::normal_distribution<> > normal_distribution_;

    // Every control_interval_ seconds we poll the controller for control thrust
    double control_interval_;

    // 1/control_interval_
    double control_frequency_;

    // Logging stores these informations for every control point
    struct LogState {
        Vector3D trajectory_position;
        Vector3D height;
    };
    std::vector<LogState> log_states_;

    // File stream to write the log to
    std::ofstream log_file_;
};

#endif // SIMULATOR_H
