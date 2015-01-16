#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "sensorsimulator.h"
#include "controller.h"
#include "controllerfullstate.h"
#include "odesystem.h"
#include "asteroid.h"

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>


class Simulator {
    /*
     * This class implements the interaction between stepwise integration of the ode system, artificial sensor data generation, control polling, and result logging.
     */
public:
    Simulator(const double &control_frequency, const double &perturbation_noise, const double &control_noise, Asteroid &asteroid, SensorSimulator *sensor_simulator, Controller *spacecraft_controller=NULL, ControllerFullState *full_state_controller=NULL);
    ~Simulator();

    // Before running the simulator, it has to be initialized with a spacecraft configuration
    void InitSpacecraft(const Vector3D &position, const Vector3D &velocity, const double &mass, const double &specific_impulse);

    // Initialize spacecraft only by specifying the spacecraft's specific impulse (useful for the function NextState)
    void InitSpacecraftSpecificImpulse(const double &specific_impulse);

    // Implements F: S x A x T -> S : F(s,a,t) = s' (Useful for RL?)
    SystemState NextState(const SystemState &state, const Vector3D &thrust, const double &time);

    // Simulates the system for time "time". Logs the states if "log_data" is enabled, Returns the actual simulated time, positions, heights, and sensor data
    boost::tuple<double, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<SensorData> > Run(const double &time, const bool &log_sensor_data);

    // Simulates the system for time "time". Returns the actual simulated time, positions and heights
    boost::tuple<double, std::vector<Vector3D>, std::vector<Vector3D> > RunForVisualization(const double &time);

    // Simulates the system for time "time". Returns the final position of the spacecraft.
    boost::tuple<double, Vector3D> RunThrough(const double &time);

    // The asteroid the simulator's ode system uses
    Asteroid& AsteroidOfSystem();

    // Returns the frequency in which the simulator polls the controller
    double ControlFrequency() const;

    // 1/control_frequency_
    double ControlInterval() const;

private:
    // Simulates random noise in the dynamical system (which is fed into the ODE system "system_")
    void SimulatePerturbations();

    // Ordinary differential system which implements the spacecraft's dynamics
    ODESystem system_;

    // Simulator which generates artificial sensor data for the controller
    SensorSimulator *sensor_simulator_;

    // Controller which generates control thrust given sensor data
    Controller *spacecraft_controller_;

    // perturbation_distribution_ ~ N(0, perturbation_noise), whereas perturbation_noise is given in the constructor
    boost::variate_generator<boost::mt19937, boost::normal_distribution<> > perturbation_distribution_;

    // Every control_interval_ seconds we poll the controller for control thrust
    double control_interval_;

    // 1/control_interval_
    double control_frequency_;

    // The controller for the simple hovering control
    ControllerFullState *full_state_controller_;
};

#endif // SIMULATOR_H
