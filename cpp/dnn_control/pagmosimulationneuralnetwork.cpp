#include "pagmosimulationneuralnetwork.h"
#include "odeint.h"
#include "modifiedcontrolledrungekutta.h"
#include "odesystem.h"
#include "samplefactory.h"
#include "sensorsimulator.h"
#include "controllerneuralnetwork.h"
#include "controllerdeepneuralnetwork.h"
#include "configuration.h"

PaGMOSimulationNeuralNetwork::PaGMOSimulationNeuralNetwork(const unsigned int &random_seed, const std::set<SensorSimulator::SensorType> &control_sensor_types, const bool &control_with_noise, const std::set<SensorSimulator::SensorType> &recording_sensor_types, const bool &recording_with_noise)
    : PaGMOSimulation(random_seed, control_sensor_types, control_with_noise, recording_sensor_types, recording_with_noise) {
    neural_network_hidden_nodes_ = kHiddenNodes;
}

PaGMOSimulationNeuralNetwork::PaGMOSimulationNeuralNetwork(const unsigned int &random_seed, const unsigned int &hidden_nodes, const std::set<SensorSimulator::SensorType> &control_sensor_types, const bool &control_with_noise, const std::set<SensorSimulator::SensorType> &recording_sensor_types, const bool &recording_with_noise)
    : PaGMOSimulation(random_seed, control_sensor_types, control_with_noise, recording_sensor_types, recording_with_noise) {
    neural_network_hidden_nodes_ = hidden_nodes;
}

PaGMOSimulationNeuralNetwork::PaGMOSimulationNeuralNetwork(const unsigned int &random_seed, const std::vector<double> &neural_network_weights, const std::set<SensorSimulator::SensorType> &control_sensor_types, const bool &control_with_noise, const std::set<SensorSimulator::SensorType> &recording_sensor_types, const bool &recording_with_noise)
    : PaGMOSimulation(random_seed, control_sensor_types, control_with_noise, recording_sensor_types, recording_with_noise) {
    neural_network_hidden_nodes_ = kHiddenNodes;
    simulation_parameters_ = neural_network_weights;
}

PaGMOSimulationNeuralNetwork::PaGMOSimulationNeuralNetwork(const unsigned int &random_seed, const unsigned int &hidden_nodes, const std::vector<double> &neural_network_weights, const std::set<SensorSimulator::SensorType> &control_sensor_types, const bool &control_with_noise, const std::set<SensorSimulator::SensorType> &recording_sensor_types, const bool &recording_with_noise)
    : PaGMOSimulation(random_seed, control_sensor_types, control_with_noise, recording_sensor_types, recording_with_noise) {
    neural_network_hidden_nodes_ = hidden_nodes;
    simulation_parameters_ = neural_network_weights;
}

boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<std::vector<double> > > PaGMOSimulationNeuralNetwork::EvaluateAdaptive() {
    typedef odeint::runge_kutta_cash_karp54<SystemState> ErrorStepper;
    typedef odeint::modified_controlled_runge_kutta<ErrorStepper> ControlledStepper;

    ControlledStepper controlled_stepper;

    SampleFactory sample_factory(random_seed_);
    SampleFactory sf_sensor_simulator(sample_factory.SampleRandomNatural());
    SampleFactory sf_sensor_recording(sf_sensor_simulator.Seed());

    SensorSimulator sensor_simulator(sf_sensor_simulator, asteroid_);
    sensor_simulator.SetNoiseEnabled(control_with_noise_);
    sensor_simulator.SetSensorTypes(control_sensor_types_);
    sensor_simulator.SetSensorValueTransformations(sensor_value_transformations_);
    sensor_simulator.SetTargetPosition(target_position_);

    SensorSimulator sensor_recorder(sf_sensor_recording, asteroid_);
    sensor_recorder.SetNoiseEnabled(recording_with_noise_);
    sensor_recorder.SetSensorTypes(recording_sensor_types_);
    sensor_recorder.SetSensorValueTransformations(sensor_value_transformations_);
    sensor_recorder.SetTargetPosition(target_position_);


#if CNN_ENABLE_STACKED_AUTOENCODER
    ControllerDeepNeuralNetwork controller(sensor_simulator.Dimensions(), spacecraft_maximum_thrust_, neural_network_hidden_nodes_);
#else
    ControllerNeuralNetwork controller(sensor_simulator.Dimensions(), spacecraft_maximum_thrust_, neural_network_hidden_nodes_);
#endif

    if (simulation_parameters_.size()) {
        controller.SetWeights(simulation_parameters_);
    }

    const unsigned int num_iterations = simulation_time_ * control_frequency_;

    std::vector<double> evaluated_times(num_iterations + 1);
    std::vector<double> evaluated_masses(num_iterations + 1);
    std::vector<Vector3D> evaluated_positions(num_iterations + 1);
    std::vector<Vector3D> evaluated_velocities(num_iterations + 1);
    std::vector<Vector3D> evaluated_heights(num_iterations + 1);
    std::vector<Vector3D> evaluated_thrusts(num_iterations +1);
    std::vector<std::vector<double> > evaluated_sensor_data(num_iterations + 1);

    SystemState system_state(initial_system_state_);

    Vector3D perturbations_acceleration;
    Vector3D thrust;
    std::vector<double> sensor_data;
    std::vector<double> sensor_recording;

    double current_time = 0.0;
    double current_time_observer = 0.0;
    const double dt = 1.0 / control_frequency_;
    Observer observer(current_time_observer);
    unsigned int iteration = 0;
    bool exception_thrown = false;
    try {
        for (iteration = 0; iteration < num_iterations; ++iteration) {
            const Vector3D &position = {system_state[0], system_state[1], system_state[2]};
            const Vector3D &velocity = {system_state[3], system_state[4], system_state[5]};
            const double &mass = system_state[6];

            const Vector3D surf_pos = boost::get<0>(asteroid_.NearestPointOnSurfaceToPosition(position));
            const Vector3D height = VectorSub(position, surf_pos);

            evaluated_times.at(iteration) = current_time;
            evaluated_masses.at(iteration) = mass;
            evaluated_positions.at(iteration) = position;
            evaluated_velocities.at(iteration) = velocity;
            evaluated_heights.at(iteration) = height;

            for (unsigned int i = 0; i < 3; ++i) {
                perturbations_acceleration[i] = sample_factory.SampleNormal(perturbation_mean_, perturbation_noise_);
            }

            sensor_data = sensor_simulator.Simulate(system_state, height, perturbations_acceleration, current_time, thrust);

            sensor_recording = sensor_recorder.Simulate(system_state, height, perturbations_acceleration, current_time, thrust);
            evaluated_sensor_data.at(iteration) = sensor_recording;


            thrust = controller.GetThrustForSensorData(sensor_data);
            evaluated_thrusts.at(iteration) = thrust;

            const double engine_noise = sample_factory.SampleNormal(0.0, spacecraft_engine_noise_);

            ODESystem ode_system(asteroid_, perturbations_acceleration, thrust, spacecraft_specific_impulse_, spacecraft_minimum_mass_, engine_noise);

            integrate_adaptive(controlled_stepper, ode_system, system_state, current_time, current_time + dt, minimum_step_size_, observer);

            current_time += dt;
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
        evaluated_sensor_data.resize(new_size);
        evaluated_thrusts.resize(new_size);
    }

    const Vector3D &position = {system_state[0], system_state[1], system_state[2]};
    const Vector3D &velocity = {system_state[3], system_state[4], system_state[5]};
    const double &mass = system_state[6];

    const Vector3D surf_pos = boost::get<0>(asteroid_.NearestPointOnSurfaceToPosition(position));
    const Vector3D height = VectorSub(position, surf_pos);

    evaluated_times.back() = current_time_observer;
    evaluated_masses.back() = mass;
    evaluated_positions.back() = position;
    evaluated_velocities.back() = velocity;
    evaluated_heights.back() = height;
    evaluated_sensor_data.back() = sensor_recording;
    evaluated_thrusts.back() = thrust;

    return boost::make_tuple(evaluated_times, evaluated_masses, evaluated_positions, evaluated_heights, evaluated_velocities, evaluated_thrusts, evaluated_sensor_data);
}

boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<std::vector<double> > > PaGMOSimulationNeuralNetwork::EvaluateFixed() {
    odeint::runge_kutta4<SystemState> stepper;

    SampleFactory sample_factory(random_seed_);
    SampleFactory sf_sensor_simulator(sample_factory.SampleRandomNatural());
    SampleFactory sf_sensor_recording(sf_sensor_simulator.Seed());

    SensorSimulator sensor_simulator(sf_sensor_simulator, asteroid_);
    sensor_simulator.SetNoiseEnabled(control_with_noise_);
    sensor_simulator.SetSensorTypes(control_sensor_types_);
    sensor_simulator.SetSensorValueTransformations(sensor_value_transformations_);
    sensor_simulator.SetTargetPosition(target_position_);

    SensorSimulator sensor_recorder(sf_sensor_recording, asteroid_);
    sensor_recorder.SetNoiseEnabled(recording_with_noise_);
    sensor_recorder.SetSensorTypes(recording_sensor_types_);
    sensor_recorder.SetSensorValueTransformations(sensor_value_transformations_);
    sensor_recorder.SetTargetPosition(target_position_);

    ControllerNeuralNetwork controller(sensor_simulator.Dimensions(), spacecraft_maximum_thrust_, neural_network_hidden_nodes_);

    if (simulation_parameters_.size()) {
        controller.SetWeights(simulation_parameters_);
    }

    std::vector<double> evaluated_times;
    std::vector<double> evaluated_masses;
    std::vector<Vector3D> evaluated_positions;
    std::vector<Vector3D> evaluated_velocities;
    std::vector<Vector3D> evaluated_heights;
    std::vector<Vector3D> evaluated_thrusts;
    std::vector<std::vector<double> > evaluated_sensor_data;

    SystemState system_state(initial_system_state_);

    Vector3D perturbations_acceleration;
    Vector3D thrust;
    std::vector<double> sensor_data;

    double current_time = 0.0;
    double engine_noise = 0.0;
    const unsigned int num_steps = 1.0 / (fixed_step_size_ * control_frequency_);
    try {
        while (current_time < simulation_time_) {
            const Vector3D &position = {system_state[0], system_state[1], system_state[2]};
            const Vector3D &velocity = {system_state[3], system_state[4], system_state[5]};
            const double &mass = system_state[6];

            const Vector3D surf_pos = boost::get<0>(asteroid_.NearestPointOnSurfaceToPosition(position));
            const Vector3D height = VectorSub(position, surf_pos);

            evaluated_times.push_back(current_time);
            evaluated_masses.push_back(mass);
            evaluated_positions.push_back(position);
            evaluated_velocities.push_back(velocity);
            evaluated_heights.push_back(height);

            for (unsigned int i = 0; i < 3; ++i) {
                perturbations_acceleration[i] = sample_factory.SampleNormal(perturbation_mean_, perturbation_noise_);
            }

            sensor_data = sensor_simulator.Simulate(system_state, height, perturbations_acceleration, current_time, thrust);

            const std::vector<double> sensor_recording = sensor_recorder.Simulate(system_state, height, perturbations_acceleration, current_time, thrust);
            evaluated_sensor_data.push_back(sensor_recording);

            thrust = controller.GetThrustForSensorData(sensor_data);
            evaluated_thrusts.push_back(thrust);

            engine_noise = sample_factory.SampleNormal(0.0, spacecraft_engine_noise_);

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

    return boost::make_tuple(evaluated_times, evaluated_masses, evaluated_positions, evaluated_heights, evaluated_velocities,evaluated_thrusts, evaluated_sensor_data);
}

unsigned int PaGMOSimulationNeuralNetwork::ChromosomeSize() const {
    SampleFactory sf;
    SensorSimulator sens_sim(sf, asteroid_);
    sens_sim.SetSensorTypes(control_sensor_types_);
    const unsigned int dimensions = sens_sim.Dimensions();

#if CNN_ENABLE_STACKED_AUTOENCODER
    return ControllerDeepNeuralNetwork(dimensions, spacecraft_maximum_thrust_, neural_network_hidden_nodes_).NumberOfParameters();
#else
    return ControllerNeuralNetwork(dimensions, spacecraft_maximum_thrust_, neural_network_hidden_nodes_).NumberOfParameters();
#endif
}

