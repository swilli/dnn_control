#ifndef PAGMOSIMULATION_H
#define PAGMOSIMULATION_H

#include "vector.h"
#include "asteroid.h"
#include "systemstate.h"
#include "sensorsimulator.h"

#include <boost/tuple/tuple.hpp>

class PaGMOSimulation {
    /*
    * This abstract class represents a full simulation of a spacecraft placed next to an asteroid.
    */
public:
    PaGMOSimulation(const unsigned int &random_seed, const std::set<SensorSimulator::SensorType> &control_sensor_types={}, const bool &control_with_noise=false, const std::set<SensorSimulator::SensorType> &recording_sensor_types={}, const bool &recording_with_noise=false, const bool &fuel_usage_enabled=true);
    PaGMOSimulation(const unsigned int &random_seed, const double &simulation_time, const std::set<SensorSimulator::SensorType> &control_sensor_types={}, const bool &control_with_noise=false, const std::set<SensorSimulator::SensorType> &recording_sensor_types={}, const bool &recording_with_noise=false, const bool &fuel_usage_enabled=true);

    virtual ~PaGMOSimulation();

    // Simulates the configured simulation, used an adaptive integrator
    virtual boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<std::vector<double> > > EvaluateAdaptive() = 0;

    // Simulates the configured simulation, used a fixed integrator
    virtual boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<std::vector<double> > > EvaluateFixed() = 0;

    // Returns the number of parameters the controller has.
    virtual unsigned int ChromosomeSize() const = 0;

    // Returns the target position, the spacecraft should stay at
    Vector3D TargetPosition() const;

    // Returns the initial spacecraft state
    SystemState InitialSystemState() const;

    // Returns the asteroid the simulation is working with
    Asteroid& AsteroidOfSystem();

    // Returns the random seed the simulation is configured with
    unsigned int RandomSeed() const;

    // Returns the simulation time the simulation will run for
    double SimulationTime() const;

    // Returns the fixed step size the fixed step integrator will use
    double FixedStepSize() const;

    // Returns the minimum step size the adaptive integrator will use
    double MinimumStepSize() const;

    // Returns the frequency the controller will be triggered during the simulation
    double ControlFrequency() const;

    // Returns the spacecraft's maximum mass
    double SpacecraftMaximumMass() const;

    // Returns the spacecraft's minimum mass
    double SpacecraftMinimumMass() const;

    // Returns the spacecraft's Isp
    double SpacecraftSpecificImpulse() const;

    // Returns the spacecraft's maximum thrust
    double SpacecraftMaximumThrust() const;

    // Change the simulation time manually
    void SetSimulationTime(const double &simulation_time);

    // PaGMOSimulation can throw the following exceptions
    class Exception {};

protected:
    // This class is used to observe the actual simulated time in case of an exception in the adaptive integration (out of fuel, crash)
    class Observer {
    public:
        Observer(double &time) : time_(time){}
        void operator () (const SystemState &, const double &current_time) {
            time_ = current_time;
        }
    private:
        double &time_;
    };

    // Configures the simulation based on the random seed
    void Init();

    // The simulation is determined by this seed
    unsigned int random_seed_;

    // The simulation will run for this time
    double simulation_time_;

    // The simulation's control frequency with which the controller gets triggered
    double control_frequency_;

    // Used by the adaptive ingetrator
    double minimum_step_size_;

    // Used by the fixed step integrator
    double fixed_step_size_;

    // Spacecraft's Isp noise
    double spacecraft_engine_noise_;

    // Spacecraft's Isp
    double spacecraft_specific_impulse_;

    // Spacecraft's maximum mass
    double spacecraft_maximum_mass_;

    // Spacecraft's minimum mass
    double spacecraft_minimum_mass_;

    // Spaceraft's maximum thrust
    double spacecraft_maximum_thrust_;

    // Random perturbation mean during the simulation
    double perturbation_mean_;

    // Random perturbation standard deviation during the simulation
    double perturbation_noise_;

    // The asteroid the simulation works with
    Asteroid asteroid_;

    // The initial spacecraft system state
    SystemState initial_system_state_;

    // The target position the spacecraft is supposed to hover on
    Vector3D target_position_;

    // The parameters for the controller
    std::vector<double> simulation_parameters_;

    // The sensor types used in the simulation for control
    std::set<SensorSimulator::SensorType> control_sensor_types_;

    // Is noise enabled during the simulation
    bool control_with_noise_;

    // The sensor types used in the simulation for recording
    std::set<SensorSimulator::SensorType> recording_sensor_types_;

    // Is noise enabled during the simulation
    bool recording_with_noise_;

    // Is the spacecraft actually consuming fuel for the taken thrust actions
    bool fuel_usage_enabled_;

    // Sensor values can be transformed using these two parameters
    std::map<SensorSimulator::SensorType, std::vector<std::pair<double, double> > > sensor_value_transformations_;
};

#endif // PAGMOSIMULATION_H
