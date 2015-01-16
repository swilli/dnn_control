#include "simulator.h"
#include "odeint.h"
#include "samplefactory.h"

Simulator::Simulator(const double &control_frequency, const double &perturbation_noise, const double &control_noise, Asteroid &asteroid, SensorSimulator *sensor_simulator, Controller *spacecraft_controller, ControllerFullState *full_state_controller) :
    system_(ODESystem(asteroid, control_noise)), perturbation_distribution_(boost::variate_generator<boost::mt19937, boost::normal_distribution<> >(SampleFactory::RandomNumberGenerator(), boost::normal_distribution<>(0.0, perturbation_noise))) {

    control_frequency_ = control_frequency;
    control_interval_ = 1.0 / control_frequency;

    sensor_simulator_ = sensor_simulator;
    spacecraft_controller_ = spacecraft_controller;
    full_state_controller_ = full_state_controller;
}

Simulator::~Simulator() {
    delete sensor_simulator_;
    if (spacecraft_controller_) {
        delete spacecraft_controller_;
    }
    if (full_state_controller_) {
        delete full_state_controller_;
    }
}

void Simulator::InitSpacecraft(const Vector3D &position, const Vector3D &velocity, const double &mass, const double &specific_impulse) {
    // Transform position, velocity and mass to state
    for (unsigned int i = 0; i < 3; ++i) {
        system_.state_[i] = position[i];
        system_.state_[3+i] = velocity[i];
    }
    system_.state_[6] = mass;

    // Cache Isp
    system_.spacecraft_specific_impulse_ = specific_impulse;
}

void Simulator::InitSpacecraftSpecificImpulse(const double &specific_impulse) {
    // Cache Isp
    system_.spacecraft_specific_impulse_ = specific_impulse;
}

State Simulator::NextState(const State &state, const Vector3D &thrust, const double &time) {
    using namespace boost::numeric::odeint;

    State next_state;

    // Assign state
    for (unsigned int i = 0; i < 3; ++i) {
        system_.state_[i] = state[i];
        system_.state_[3+i] = state[3+i];
        system_.thrust_[i] = thrust[i];
    }
    system_.state_[6] = state[6];

    // Simulate perturbations, directly write it to the ode system
    SimulatePerturbations();

    // Integrate for one time step control_interval_
    runge_kutta4<State> integrator;
    integrator.do_step(system_, system_.state_, time, control_interval_);

    // Extract new state out of system
    for (unsigned int i = 0; i < 7; ++i) {
        next_state[i] = system_.state_[i];
    }

    return next_state;
}

boost::tuple<double, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<SensorData> > Simulator::Run(const double &time, const bool &log_sensor_data) {
    using namespace boost::numeric::odeint;

    std::vector<SensorData> logged_sensor_data;
    std::vector<Vector3D> logged_positions;
    std::vector<Vector3D> logged_heights;

    const unsigned int sensor_dimensions = sensor_simulator_->Dimensions();
    if ((spacecraft_controller_ != NULL) && (sensor_dimensions != spacecraft_controller_->Dimensions())) {
        std::cout << "warning: sensor simulator and spacecraft controller have different output/input dimensions." << std::endl;
    }
    const double dt = control_interval_;
    const unsigned int iterations = time / dt;
    unsigned int performed_iterations = 0;

    logged_positions = std::vector<Vector3D>(iterations);
    logged_heights = std::vector<Vector3D>(iterations);

    if (log_sensor_data) {
        logged_sensor_data = std::vector<SensorData>(iterations, std::vector<double>(sensor_dimensions,0));
    }

    runge_kutta4<State> integrator;
    double current_time = 0.0;

    // Stepwise integration for "iterations" iterations
    try {
        for(unsigned int iteration = 0; iteration < iterations; ++iteration) {
            // Compute spacecraft position
            const Vector3D position = {system_.state_[0], system_.state_[1], system_.state_[2]};

            // Compute height
            const Vector3D surf_pos = boost::get<0>(system_.asteroid_.NearestPointOnSurfaceToPosition(position));
            const Vector3D height = {position[0] - surf_pos[0], position[1] - surf_pos[1], position[2] - surf_pos[2],};

            // Log position and height
            Vector3D &iter_pos = logged_positions.at(iteration);
            iter_pos[0] = position[0];
            iter_pos[1] = position[1];
            iter_pos[2] = position[2];

            Vector3D &iter_height = logged_heights.at(iteration);
            iter_height[0] = height[0];
            iter_height[1] = height[1];
            iter_height[2] = height[2];

            // Simulate perturbations, directly write it to the ode system
            SimulatePerturbations();

            // Simulate sensor data
            const SensorData sensor_data = sensor_simulator_->Simulate(system_.state_, height, system_.perturbations_acceleration_, current_time);

            if (log_sensor_data) {
                // Log sensor data, if enabled
                SensorData &data = logged_sensor_data.at(iteration);
                for (unsigned int i = 0; i < sensor_dimensions; ++i) {
                    data[i] = sensor_data[i];
                }
            }

            //std::cout << "real position:" << std::endl;
            //std::cout << "[" << system_.state_[0] << ";" << std::endl << system_.state_[1] << ";" << std::endl << system_.state_[2] <<  "]" << std::endl << std::endl;

            // If default controller is available, use it
            if (full_state_controller_) {
                SensorData full_state(7, 0.0);
                for (unsigned int i = 0; i < 7; ++i) {
                    full_state[i] = system_.state_[i];
                }
                system_.thrust_ = full_state_controller_->GetThrustForSensorData(full_state);

            } else if (spacecraft_controller_){
                // Poll controller for control thrust, write it to the ode system
                system_.thrust_ = spacecraft_controller_->GetThrustForSensorData(sensor_data);

            } else {
                // No controller -> no thrust
                system_.thrust_[0] = 0.0;
                system_.thrust_[1] = 0.0;
                system_.thrust_[2] = 0.0;
            }

            // Integrate the system for control_interval_ time
            integrator.do_step(system_, system_.state_, current_time, dt);

            current_time += dt;
            performed_iterations++;

            // Check if spacecraft is out of fuel
            if (system_.state_[6] <= 0.0) {
                std::cout << "The spacecraft is out of fuel." << std::endl;

                logged_positions.resize(performed_iterations);
                logged_heights.resize(performed_iterations);

                if (log_sensor_data) {
                    logged_sensor_data.resize(performed_iterations);
                }

                break;
            }
        }
    } catch (const Asteroid::Exception &exception) {
        std::cout << "The spacecraft crashed into the asteroid's surface." << std::endl;

        logged_positions.resize(performed_iterations);
        logged_heights.resize(performed_iterations);

        if (log_sensor_data) {
            logged_sensor_data.resize(performed_iterations);
        }
    }

    return boost::make_tuple(current_time, logged_positions, logged_heights, logged_sensor_data);
}

boost::tuple<double, std::vector<Vector3D>, std::vector<Vector3D> > Simulator::RunForVisualization(const double &time) {
    using namespace boost::numeric::odeint;

    std::vector<Vector3D> logged_positions;
    std::vector<Vector3D> logged_heights;

    const unsigned int sensor_dimensions = sensor_simulator_->Dimensions();
    if ((spacecraft_controller_ != NULL) && (sensor_dimensions != spacecraft_controller_->Dimensions())) {
        std::cout << "warning: sensor simulator and spacecraft controller have different output/input dimensions." << std::endl;
    }
    const double dt = control_interval_;
    const unsigned int iterations = time / dt;
    unsigned int performed_iterations = 0;

    logged_positions = std::vector<Vector3D>(iterations);
    logged_heights = std::vector<Vector3D>(iterations);

    runge_kutta4<State> integrator;
    double current_time = 0.0;

    // Stepwise integration for "iterations" iterations
    try {
        for(unsigned int iteration = 0; iteration < iterations; ++iteration) {
            // Compute spacecraft position
            const Vector3D position = {system_.state_[0], system_.state_[1], system_.state_[2]};

            // Compute height
            const Vector3D surf_pos = boost::get<0>(system_.asteroid_.NearestPointOnSurfaceToPosition(position));
            const Vector3D height = {position[0] - surf_pos[0], position[1] - surf_pos[1], position[2] - surf_pos[2],};

            // Log position and height
            Vector3D &iter_pos = logged_positions.at(iteration);
            iter_pos[0] = position[0];
            iter_pos[1] = position[1];
            iter_pos[2] = position[2];

            Vector3D &iter_height = logged_heights.at(iteration);
            iter_height[0] = height[0];
            iter_height[1] = height[1];
            iter_height[2] = height[2];

            // Simulate perturbations, directly write it to the ode system
            SimulatePerturbations();

            // Simulate sensor data
            const SensorData sensor_data = sensor_simulator_->Simulate(system_.state_, height, system_.perturbations_acceleration_, current_time);

            // If default controller is available, use it
            if (full_state_controller_) {
                SensorData full_state(7, 0.0);
                for (unsigned int i = 0; i < 7; ++i) {
                    full_state[i] = system_.state_[i];
                }
                system_.thrust_ = full_state_controller_->GetThrustForSensorData(full_state);

            } else if (spacecraft_controller_){
                // Poll controller for control thrust, write it to the ode system
                system_.thrust_ = spacecraft_controller_->GetThrustForSensorData(sensor_data);

            } else {
                // No controller -> no thrust
                system_.thrust_[0] = 0.0;
                system_.thrust_[1] = 0.0;
                system_.thrust_[2] = 0.0;
            }

            // Integrate the system for control_interval_ time
            integrator.do_step(system_, system_.state_, current_time, dt);

            current_time += dt;
            performed_iterations++;

            // Check if spacecraft is out of fuel
            if (system_.state_[6] <= 0.0) {
                std::cout << "The spacecraft is out of fuel." << std::endl;

                logged_positions.resize(performed_iterations);
                logged_heights.resize(performed_iterations);

                break;
            }
        }
    } catch (const Asteroid::Exception &exception) {
        std::cout << "The spacecraft crashed into the asteroid's surface." << std::endl;

        logged_positions.resize(performed_iterations);
        logged_heights.resize(performed_iterations);
    }

    return boost::make_tuple(current_time, logged_positions, logged_heights);
}

static struct IntegrationPointAction {
    double *time;
    Simulator *simulator;
    IntegrationPointAction() : time(NULL), simulator(NULL) {}

    void operator () ( const State &state , const double &param_time) {
        *time = param_time;
    }
} integration_point_action;

boost::tuple<double, Vector3D> Simulator::RunThrough(const double &time) {
    using namespace boost::numeric::odeint;

    double simulated_time = 0.0;
    integration_point_action.time = &simulated_time;
    integration_point_action.simulator = this;

    // Simulate perturbations, directly write it to the ode system
    SimulatePerturbations();

    try {
        const double &min_step_size = 0.1;
        integrate(system_, system_.state_, 0.0, time, min_step_size, integration_point_action);
    } catch (const Asteroid::Exception &exception) {
        std::cout << "The spacecraft crashed into the asteroid's surface." << std::endl;
    }

    Vector3D final_position = {system_.state_[0], system_.state_[1], system_.state_[2]};

    return boost::make_tuple(simulated_time, final_position);
}

Asteroid& Simulator::AsteroidOfSystem() {
    return system_.asteroid_;
}

double Simulator::ControlFrequency() const {
    return control_frequency_;
}

double Simulator::ControlInterval() const {
    return control_interval_;
}

void Simulator::SimulatePerturbations() {
    double mass = system_.state_[6];
    for(unsigned int i = 0; i < 3; ++i) {
        system_.perturbations_acceleration_[i] = mass * perturbation_distribution_();
    }
}
