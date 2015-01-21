#include "fixedsimulation.h"
#include "datacollector.h"
#include "odeint.h"
#include "odesystem.h"

FixedSimulation::FixedSimulation(const unsigned int &random_seed) : Simulation(random_seed) {
    fixed_step_size_ = 0.1;
}

FixedSimulation::FixedSimulation(const FixedSimulation &other) : Simulation(other) {
    fixed_step_size_ = other.fixed_step_size_;
}

FixedSimulation::~FixedSimulation() {
}

FixedSimulation& FixedSimulation::operator=(const FixedSimulation &other) {
    if (&other != this) {
        random_seed_ = other.random_seed_;
        simulation_time_ = other.simulation_time_;
        engine_noise_ = other.engine_noise_;
        perturbation_noise_ = other.perturbation_noise_;
        spacecraft_specific_impulse_ = other.spacecraft_specific_impulse_;
        sample_factory_ = other.sample_factory_;
        asteroid_ = other.asteroid_;
        fixed_step_size_ = other.fixed_step_size_;
        if (sensor_simulator_ != NULL) {
            delete sensor_simulator_;
        }
        if (other.sensor_simulator_ != NULL) {
            sensor_simulator_ = other.sensor_simulator_->Clone(sample_factory_);
        } else {
            sensor_simulator_ = NULL;
        }
        if (controller_ != NULL) {
            delete controller_;
        }
        if (other.controller_ != NULL) {
            controller_ = other.controller_->Clone();
        } else {
            controller_ = NULL;
        }
        initial_system_state_ = other.initial_system_state_;
    }
    return *this;
}

boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > FixedSimulation::Evaluate() {
    sample_factory_.SetSeed(random_seed_);

    std::vector<double> time_points;
    std::vector<double> evaluated_masses;
    std::vector<Vector3D> evaluated_positions;
    std::vector<Vector3D> evaluated_velocities;
    std::vector<Vector3D> evaluated_heights;
    std::vector<Vector3D> evaluated_angular_velocities;

    DataCollector collector(asteroid_, time_points, evaluated_masses, evaluated_positions, evaluated_heights, evaluated_velocities, evaluated_angular_velocities);
    SystemState system_state(initial_system_state_);

    ODESystem sys(sample_factory_, asteroid_, sensor_simulator_, controller_, spacecraft_specific_impulse_, perturbation_noise_, engine_noise_);

    odeint::runge_kutta4<SystemState> stepper;
    try {
         integrate_const(stepper , sys, system_state, 0.0, simulation_time_, fixed_step_size_, collector);
    } catch (const Asteroid::Exception &exception) {
        std::cout << "The spacecraft crashed into the asteroid's surface." << std::endl;
    } catch (const ODESystem::Exception &exception) {
         std::cout << "The spacecraft is out of fuel." << std::endl;
    }

    return boost::make_tuple(time_points, evaluated_masses, evaluated_positions, evaluated_heights, evaluated_velocities, evaluated_angular_velocities);
}

double FixedSimulation::FixedStepSize() const {
    return fixed_step_size_;
}


