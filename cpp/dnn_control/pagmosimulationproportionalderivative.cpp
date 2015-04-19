#include "pagmosimulationproportionalderivative.h"
#include "odeint.h"
#include "modifiedcontrolledrungekutta.h"
#include "odesystem.h"
#include "samplefactory.h"
#include "controllerproportionalderivative.h"
#include "configuration.h"

PaGMOSimulationProportionalDerivative::PaGMOSimulationProportionalDerivative(const unsigned int &random_seed)
    : PaGMOSimulation(random_seed) {
}

PaGMOSimulationProportionalDerivative::PaGMOSimulationProportionalDerivative(const unsigned int &random_seed, const std::vector<double> &pd_coefficients)
    : PaGMOSimulation(random_seed) {
    simulation_parameters_ = pd_coefficients;
}

boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<std::vector<double> > > PaGMOSimulationProportionalDerivative::EvaluateAdaptive() {
    typedef odeint::runge_kutta_cash_karp54<SystemState> ErrorStepper;
    typedef odeint::modified_controlled_runge_kutta<ErrorStepper> ControlledStepper;

    ControlledStepper controlled_stepper;

    SampleFactory sample_factory(random_seed_);
    SampleFactory sf_sensor_simulator(sample_factory.SampleRandomNatural());

    SensorSimulator sensor_simulator(sf_sensor_simulator, asteroid_, sensor_types_, enable_sensor_noise_, target_position_, sensor_value_transformations_);

#if PGMOS_ENABLE_SENSOR_DATA_RECORDING
    SampleFactory sf_sensor_recording(sf_sensor_simulator.Seed());
    SensorSimulator sensor_recorder(sf_sensor_recording, asteroid_, {SensorSimulator::SensorType::RelativePosition, SensorSimulator::SensorType::Velocity}, false);
#endif

    ControllerProportionalDerivative controller(sensor_simulator.Dimensions(), spacecraft_maximum_thrust_);

    if (simulation_parameters_.size()) {
        controller.SetCoefficients(simulation_parameters_);
    }

    const unsigned int num_iterations = simulation_time_ * control_frequency_;

    std::vector<double> evaluated_times(num_iterations + 1);
    std::vector<double> evaluated_masses(num_iterations + 1);
    std::vector<Vector3D> evaluated_positions(num_iterations + 1);
    std::vector<Vector3D> evaluated_velocities(num_iterations + 1);
    std::vector<Vector3D> evaluated_heights(num_iterations + 1);
    std::vector<Vector3D> evaluated_thrusts(num_iterations + 1);
    std::vector<std::vector<double> > evaluated_sensor_data(num_iterations + 1);

    SystemState system_state(initial_system_state_);

    Vector3D perturbations_acceleration;
    Vector3D thrust;
    std::vector<double> sensor_data;
#if PGMOS_ENABLE_SENSOR_DATA_RECORDING
    std::vector<double> sensor_recording;
#endif

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
            const Vector3D &height = {position[0] - surf_pos[0], position[1] - surf_pos[1], position[2] - surf_pos[2]};

            evaluated_times.at(iteration) = current_time;
            evaluated_masses.at(iteration) = mass;
            evaluated_positions.at(iteration) = position;
            evaluated_velocities.at(iteration) = velocity;
            evaluated_heights.at(iteration) = height;

            for (unsigned int i = 0; i < 3; ++i) {
                perturbations_acceleration[i] = sample_factory.SampleNormal(perturbation_mean_, perturbation_noise_);
            }

            sensor_data = sensor_simulator.Simulate(system_state, height, perturbations_acceleration, current_time, thrust);

#if PGMOS_ENABLE_SENSOR_DATA_RECORDING
            sensor_recording = sensor_recorder.Simulate(system_state, height, perturbations_acceleration, current_time);
            evaluated_sensor_data.at(iteration) = sensor_recording;
#else
            evaluated_sensor_data.at(iteration) = sensor_data;
#endif

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
    const Vector3D &height = {position[0] - surf_pos[0], position[1] - surf_pos[1], position[2] - surf_pos[2]};

    evaluated_times.back() = current_time_observer;
    evaluated_masses.back() = mass;
    evaluated_positions.back() = position;
    evaluated_velocities.back() = velocity;
    evaluated_heights.back() = height;
#if PGMOS_ENABLE_SENSOR_DATA_RECORDING
    evaluated_sensor_data.back() = sensor_recording;
#else
    evaluated_sensor_data.back() = sensor_data;
#endif
    evaluated_thrusts.back() = thrust;

    return boost::make_tuple(evaluated_times, evaluated_masses, evaluated_positions, evaluated_heights, evaluated_velocities, evaluated_thrusts, evaluated_sensor_data);
}

boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<std::vector<double> > > PaGMOSimulationProportionalDerivative::EvaluateFixed() {
    odeint::runge_kutta4<SystemState> stepper;

    SampleFactory sample_factory(random_seed_);
    SampleFactory sf_sensor_simulator(sample_factory.SampleRandomNatural());

    SensorSimulator sensor_simulator(sf_sensor_simulator, asteroid_, sensor_types_, enable_sensor_noise_, target_position_, sensor_value_transformations_);

#if PGMOS_ENABLE_SENSOR_DATA_RECORDING
    SampleFactory sf_sensor_recording(sf_sensor_simulator.Seed());
    SensorSimulator sensor_recorder(sf_sensor_recording, asteroid_, {SensorSimulator::SensorType::RelativePosition, SensorSimulator::SensorType::Velocity}, false);
#endif

    ControllerProportionalDerivative controller(sensor_simulator.Dimensions(), spacecraft_maximum_thrust_);

    if (simulation_parameters_.size()) {
        controller.SetCoefficients(simulation_parameters_);
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
    const unsigned int num_steps = 1.0 / (fixed_step_size_ * control_frequency_);
    try {
        while (current_time < simulation_time_) {
            const Vector3D &position = {system_state[0], system_state[1], system_state[2]};
            const Vector3D &velocity = {system_state[3], system_state[4], system_state[5]};
            const double &mass = system_state[6];

            const Vector3D surf_pos = boost::get<0>(asteroid_.NearestPointOnSurfaceToPosition(position));
            const Vector3D &height = {position[0] - surf_pos[0], position[1] - surf_pos[1], position[2] - surf_pos[2]};

            evaluated_times.push_back(current_time);
            evaluated_masses.push_back(mass);
            evaluated_positions.push_back(position);
            evaluated_velocities.push_back(velocity);
            evaluated_heights.push_back(height);

            for (unsigned int i = 0; i < 3; ++i) {
                perturbations_acceleration[i] = sample_factory.SampleNormal(perturbation_mean_, perturbation_noise_);
            }

            sensor_data = sensor_simulator.Simulate(system_state, height, perturbations_acceleration, current_time, thrust);

#if PGMOS_ENABLE_SENSOR_DATA_RECORDING
            const std::vector<double> sensor_recording = sensor_recorder.Simulate(system_state, height, perturbations_acceleration, current_time);
            evaluated_sensor_data.push_back(sensor_recording);
#else
            evaluated_sensor_data.push_back(sensor_data);
#endif

            thrust = controller.GetThrustForSensorData(sensor_data);
            evaluated_thrusts.push_back(thrust);

            const double engine_noise = sample_factory.SampleNormal(0.0, spacecraft_engine_noise_);

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

    return boost::make_tuple(evaluated_times, evaluated_masses, evaluated_positions, evaluated_heights, evaluated_velocities, evaluated_thrusts, evaluated_sensor_data);
}

unsigned int PaGMOSimulationProportionalDerivative::ChromosomeSize() const {
    SampleFactory sf;
    const unsigned int dimensions = SensorSimulator(sf, asteroid_, sensor_types_, enable_sensor_noise_, target_position_, sensor_value_transformations_).Dimensions();
    return ControllerProportionalDerivative(dimensions, spacecraft_maximum_thrust_).NumberOfParameters();
}
