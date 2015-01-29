#include "pagmosimulationneuralnetwork.h"
#include "odeint.h"
#include "modifiedcontrolledrungekutta.h"
#include "samplefactory.h"
#include "sensorsimulatorneuralnetwork.h"
#include "controllerneuralnetwork.h"
#include "odesystem.h"


PaGMOSimulationNeuralNetwork::PaGMOSimulationNeuralNetwork(const unsigned int &random_seed, const double &simulation_time)
    : random_seed_(random_seed),  simulation_time_(simulation_time) {
    neural_network_hidden_nodes_ = 10;
    Init();
}

PaGMOSimulationNeuralNetwork::PaGMOSimulationNeuralNetwork(const unsigned int &random_seed,  const double &simulation_time, const unsigned int &hidden_nodes)
    : random_seed_(random_seed), neural_network_hidden_nodes_(hidden_nodes), simulation_time_(simulation_time) {
    Init();
}

PaGMOSimulationNeuralNetwork::PaGMOSimulationNeuralNetwork(const unsigned int &random_seed, const double &simulation_time, const std::vector<double> &neural_network_weights)
    : random_seed_(random_seed), simulation_time_(simulation_time) {
    neural_network_hidden_nodes_ = 10;
#if PSNN_TEST_FOR_ORBIT == 0
    neural_network_weights_ = neural_network_weights;
#endif
    Init();
}

PaGMOSimulationNeuralNetwork::PaGMOSimulationNeuralNetwork(const unsigned int &random_seed, const double &simulation_time, const unsigned int &hidden_nodes, const std::vector<double> &neural_network_weights)
    : random_seed_(random_seed), neural_network_hidden_nodes_(hidden_nodes), simulation_time_(simulation_time) {
#if PSNN_TEST_FOR_ORBIT == 0
    neural_network_weights_ = neural_network_weights;
#endif
    Init();
}

PaGMOSimulationNeuralNetwork::~PaGMOSimulationNeuralNetwork() {

}

boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > PaGMOSimulationNeuralNetwork::Evaluate() {
    typedef odeint::runge_kutta_cash_karp54<SystemState> ErrorStepper;
    typedef odeint::modified_controlled_runge_kutta<ErrorStepper> ControlledStepper;

    SampleFactory sample_factory(random_seed_);
    SampleFactory sf_sensor_simulator(sample_factory.SampleRandomInteger());

    SensorSimulatorNeuralNetwork sensor_simulator(sf_sensor_simulator, asteroid_, target_position_);
    ControllerNeuralNetwork controller(spacecraft_maximum_thrust_, neural_network_hidden_nodes_);
    if (neural_network_weights_.size()) {
        controller.SetWeights(neural_network_weights_);
    }

    if (sensor_simulator.Dimensions() != controller.Dimensions()) {
        throw SizeMismatchException();
    }

    const unsigned int num_iterations = simulation_time_ / interaction_interval_;

    std::vector<double> evaluated_times(num_iterations + 1);
    std::vector<double> evaluated_masses(num_iterations + 1);
    std::vector<Vector3D> evaluated_positions(num_iterations + 1);
    std::vector<Vector3D> evaluated_velocities(num_iterations + 1);
    std::vector<Vector3D> evaluated_heights(num_iterations + 1);

    SystemState system_state(initial_system_state_);

    Vector3D perturbations_acceleration;
    for (unsigned int i = 0; i < 3; ++i) {
        perturbations_acceleration[i] = sample_factory.SampleNormal(0.0, perturbation_noise_);
    }

    double current_time = 0.0;
    double current_time_observer = 0.0;
    Observer observer(current_time_observer);
    unsigned int iteration = 0;
    bool exception_thrown = false;
    try {
        for (iteration = 0; iteration < num_iterations; ++iteration) {
            const Vector3D &position = {system_state[0], system_state[1], system_state[2]};
            const Vector3D &velocity = {system_state[3], system_state[4], system_state[5]};
            const double &mass = system_state[6];

            const Vector3D &surf_pos = boost::get<0>(asteroid_.NearestPointOnSurfaceToPosition(position));
            const Vector3D &height = {position[0] - surf_pos[0], position[1] - surf_pos[1], position[2] - surf_pos[2]};

            evaluated_times[iteration] = current_time;
            evaluated_masses[iteration] = mass;
            evaluated_positions[iteration] = position;
            evaluated_velocities[iteration] = velocity;
            evaluated_heights[iteration] = height;

            const SensorData sensor_data = sensor_simulator.Simulate(system_state, height, perturbations_acceleration, current_time);
            const Vector3D thrust = controller.GetThrustForSensorData(sensor_data);
            const double engine_noise = sample_factory.SampleNormal(0.0, engine_noise_);

            ODESystem ode_system(asteroid_, perturbations_acceleration, thrust, spacecraft_specific_impulse_, spacecraft_minimum_mass_, engine_noise);

            ControlledStepper controlled_stepper;
            integrate_adaptive(controlled_stepper, ode_system, system_state, current_time, current_time + interaction_interval_, minimum_step_size_, observer);

            current_time += interaction_interval_;
        }
    } catch (const Asteroid::Exception &exception) {
        //std::cout << "The spacecraft crashed into the asteroid's surface." << std::endl;
        exception_thrown = true;
    } catch (const ODESystem::Exception &exception) {
        //std::cout << "The spacecraft is out of fuel." << std::endl;
        exception_thrown = true;
    }
    if (exception_thrown) {
        const unsigned int new_size = iteration + 2;
        evaluated_times.resize(new_size);
        evaluated_masses.resize(new_size);
        evaluated_positions.resize(new_size);
        evaluated_velocities.resize(new_size);
        evaluated_heights.resize(new_size);
    }

    const Vector3D &position = {system_state[0], system_state[1], system_state[2]};
    const Vector3D &velocity = {system_state[3], system_state[4], system_state[5]};
    const double &mass = system_state[6];

    const Vector3D &surf_pos = boost::get<0>(asteroid_.NearestPointOnSurfaceToPosition(position));
    const Vector3D &height = {position[0] - surf_pos[0], position[1] - surf_pos[1], position[2] - surf_pos[2]};

    evaluated_times.back() = current_time_observer;
    evaluated_masses.back() = mass;
    evaluated_positions.back() = position;
    evaluated_velocities.back() = velocity;
    evaluated_heights.back() = height;

    return boost::make_tuple(evaluated_times, evaluated_masses, evaluated_positions, evaluated_heights, evaluated_velocities);
}

boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > PaGMOSimulationNeuralNetwork::EvaluateDetailed() {
    SampleFactory sample_factory(random_seed_);
    SampleFactory sf_sensor_simulator(sample_factory.SampleRandomInteger());

    SensorSimulatorNeuralNetwork sensor_simulator(sf_sensor_simulator, asteroid_, target_position_);
    ControllerNeuralNetwork controller(spacecraft_maximum_thrust_, neural_network_hidden_nodes_);
    if (neural_network_weights_.size()) {
        controller.SetWeights(neural_network_weights_);
    }

    if (sensor_simulator.Dimensions() != controller.Dimensions()) {
        throw SizeMismatchException();
    }

    std::vector<double> evaluated_times;
    std::vector<double> evaluated_masses;
    std::vector<Vector3D> evaluated_positions;
    std::vector<Vector3D> evaluated_velocities;
    std::vector<Vector3D> evaluated_heights;

    SystemState system_state(initial_system_state_);

    Vector3D perturbations_acceleration;
    for (unsigned int i = 0; i < 3; ++i) {
        perturbations_acceleration[i] = sample_factory.SampleNormal(0.0, perturbation_noise_);
    }

    double current_time = 0.0;
    double engine_noise = 0.0;
    Vector3D thrust = {0.0, 0.0, 0.0};
    const unsigned int num_steps = interaction_interval_ / fixed_step_size_;
    odeint::runge_kutta4<SystemState> stepper;
    try {
        while (current_time < simulation_time_) {
            const Vector3D &position = {system_state[0], system_state[1], system_state[2]};
            const Vector3D &velocity = {system_state[3], system_state[4], system_state[5]};
            const double &mass = system_state[6];

            const Vector3D &surf_pos = boost::get<0>(asteroid_.NearestPointOnSurfaceToPosition(position));
            const Vector3D &height = {position[0] - surf_pos[0], position[1] - surf_pos[1], position[2] - surf_pos[2]};

            evaluated_times.push_back(current_time);
            evaluated_masses.push_back(mass);
            evaluated_positions.push_back(position);
            evaluated_velocities.push_back(velocity);
            evaluated_heights.push_back(height);

            const SensorData sensor_data = sensor_simulator.Simulate(system_state, height, perturbations_acceleration, current_time);
            thrust = controller.GetThrustForSensorData(sensor_data);
            engine_noise = sample_factory.SampleNormal(0.0, engine_noise_);
            ODESystem ode_system(asteroid_, perturbations_acceleration, thrust, spacecraft_specific_impulse_, spacecraft_minimum_mass_, engine_noise);
            for (unsigned int i = 0; i < num_steps; ++i) {
                stepper.do_step(ode_system, system_state, current_time, fixed_step_size_);
                current_time += fixed_step_size_;
            }
        }
    } catch (const Asteroid::Exception &exception) {
        //std::cout << "The spacecraft crashed into the asteroid's surface." << std::endl;
    } catch (const ODESystem::Exception &exception) {
        //std::cout << "The spacecraft is out of fuel." << std::endl;
    }

    return boost::make_tuple(evaluated_times, evaluated_masses, evaluated_positions, evaluated_heights, evaluated_velocities);
}

double PaGMOSimulationNeuralNetwork::FixedStepSize() const {
    return fixed_step_size_;
}

double PaGMOSimulationNeuralNetwork::MinimumStepSize() const {
    return minimum_step_size_;
}

double PaGMOSimulationNeuralNetwork::InteractionInterval() const {
    return interaction_interval_;
}

unsigned int PaGMOSimulationNeuralNetwork::ControllerNeuralNetworkSize() const {
    return ControllerNeuralNetwork(spacecraft_maximum_thrust_, neural_network_hidden_nodes_).NeuralNetworkSize();
}

Asteroid& PaGMOSimulationNeuralNetwork::AsteroidOfSystem() {
    return asteroid_;
}

void PaGMOSimulationNeuralNetwork::Init() {
    minimum_step_size_ = 0.1;
    fixed_step_size_ = 0.1;
    interaction_interval_ = 1.0;

    SampleFactory sample_factory(random_seed_);

    const Vector3D semi_axis = {sample_factory.SampleUniform(8000.0, 12000.0), sample_factory.SampleUniform(4000.0, 7500.0), sample_factory.SampleUniform(1000.0, 3500.0)};
    const double density = sample_factory.SampleUniform(1500.0, 3000.0);
    Vector2D angular_velocity_xz = {sample_factory.SampleSign() * sample_factory.SampleUniform(0.0002, 0.0008), sample_factory.SampleSign() * sample_factory.SampleUniform(0.0002, 0.0008)};
    const double time_bias = sample_factory.SampleUniform(0.0, 12.0 * 60 * 60);
    asteroid_ = Asteroid(semi_axis, density, angular_velocity_xz, time_bias);

    const double spacecraft_mass = sample_factory.SampleUniform(450.0, 500.0);
    spacecraft_minimum_mass_ = spacecraft_mass * 0.5;
    spacecraft_maximum_thrust_ = 21.0;
    spacecraft_specific_impulse_ = 200.0;


#if PSNN_TEST_FOR_ORBIT == 1
    // orbit
    const Vector3D spacecraft_position = sample_factory.SamplePointOutSideEllipsoid(semi_axis, 2.0, 4.0);
#else
    // random
    const Vector3D spacecraft_position = sample_factory.SamplePointOutSideEllipsoid(semi_axis, 1.1, 4.0);
#endif

    target_position_ = spacecraft_position;

    const Vector3D angular_velocity = boost::get<0>(asteroid_.AngularVelocityAndAccelerationAtTime(0.0));
    Vector3D spacecraft_velocity = VectorCrossProduct(angular_velocity, spacecraft_position);

#if PSNN_TEST_FOR_ORBIT == 1
    // orbital velocity
    const double norm_position = VectorNorm(spacecraft_position);
    const double magn_orbital_vel = sqrt(asteroid_.MassGravitationalConstant() / norm_position);
    Vector3D orth_pos = {sample_factory.SampleSign(), sample_factory.SampleSign(),  sample_factory.SampleSign()};
    if (spacecraft_position[2] > 1.0 || spacecraft_position[2] < -1.0) {
        orth_pos[2] = -(orth_pos[0]*spacecraft_position[0] + orth_pos[1]*spacecraft_position[1]) / spacecraft_position[2];
    } else if (spacecraft_position[1] > 1.0 || spacecraft_position[1] < -1.0) {
        orth_pos[1] = -(orth_pos[0]*spacecraft_position[0] + orth_pos[2]*spacecraft_position[2]) / spacecraft_position[1];
    } else {
        orth_pos[0] = -(orth_pos[1]*spacecraft_position[1] + orth_pos[2]*spacecraft_position[2]) / spacecraft_position[0];
    }
    const double coef_norm_orth_pos = 1.0 / VectorNorm(orth_pos);
    for (unsigned int i = 0; i < 3; ++i) {
        orth_pos[i] *= coef_norm_orth_pos;
    }
    spacecraft_velocity[0] = -spacecraft_velocity[0] + orth_pos[0] * magn_orbital_vel;
    spacecraft_velocity[1] = -spacecraft_velocity[1] + orth_pos[1] * magn_orbital_vel;
    spacecraft_velocity[2] = -spacecraft_velocity[2] + orth_pos[2] * magn_orbital_vel;
#else
    // no velocity
    spacecraft_velocity[0] *= -1; spacecraft_velocity[1] *= -1; spacecraft_velocity[2] *= -1;
#endif


    perturbation_noise_ = 1e-7;
    engine_noise_ = 0.05;

    for (unsigned int i = 0; i < 3; ++i) {
        initial_system_state_[i] = spacecraft_position[i];
        initial_system_state_[3+i] = spacecraft_velocity[i];
    }
    initial_system_state_[6] = spacecraft_mass;
}
