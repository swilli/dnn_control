#include "pagmosimulationfullstate.h"
#include "utility.h"
#include "datacollector.h"
#include "odeint.h"
#include "modifiedcontrolledrungekutta.h"
#include "samplefactory.h"
#include "sensorsimulatorfullstate.h"
#include "controllerfullstate.h"
#include "odesystem.h"
#include "odesystemimpl2.h"

PaGMOSimulationFullState::PaGMOSimulationFullState(const unsigned int &random_seed, const double &simulation_time) : random_seed_(random_seed),  simulation_time_(simulation_time) {
    Init();
}

PaGMOSimulationFullState::PaGMOSimulationFullState(const unsigned int &random_seed, const double &simulation_time, const std::vector<double> &pid_coefficients) : random_seed_(random_seed),  simulation_time_(simulation_time) {
    full_state_coefficients_ = pid_coefficients;
    Init();
}

PaGMOSimulationFullState::~PaGMOSimulationFullState() {

}

boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > PaGMOSimulationFullState::Evaluate() {
    SampleFactory sample_factory(random_seed_);
    SampleFactory sf_sensor_simulator(sample_factory.SampleRandomInteger());
    SampleFactory sf_odesystem(sample_factory.SampleRandomInteger());

    SensorSimulatorFullState sensor_simulator(sf_sensor_simulator, asteroid_);
    ControllerFullState controller(spacecraft_maximum_thrust_, target_position_);
    if (full_state_coefficients_.size()) {
        controller.SetCoefficients(full_state_coefficients_[0], full_state_coefficients_[1], full_state_coefficients_[2]);
    }

    if (sensor_simulator.Dimensions() != controller.Dimensions()) {
        throw SizeMismatchException();
    }

    std::vector<double> time_points;
    std::vector<double> evaluated_masses;
    std::vector<Vector3D> evaluated_positions;
    std::vector<Vector3D> evaluated_velocities;
    std::vector<Vector3D> evaluated_heights;

    DataCollector collector(asteroid_, time_points, evaluated_masses, evaluated_positions, evaluated_heights, evaluated_velocities);
    SystemState system_state(initial_system_state_);

    typedef odeint::runge_kutta_cash_karp54<SystemState> ErrorStepper;
    typedef odeint::modified_controlled_runge_kutta<ErrorStepper> ControlledStepper;
    ControlledStepper controlled_stepper;

    ODESystem sys(sf_odesystem, asteroid_, sensor_simulator, controller, spacecraft_specific_impulse_, perturbation_noise_, engine_noise_);

    try {
        integrate_adaptive(controlled_stepper , sys, system_state, 0.0, simulation_time_, minimum_step_size_, collector);
    } catch (const Asteroid::Exception &exception) {
        //std::cout << "The spacecraft crashed into the asteroid's surface." << std::endl;
    } catch (const ODESystem::Exception &exception) {
        //std::cout << "The spacecraft is out of fuel." << std::endl;
    }

    return boost::make_tuple(time_points, evaluated_masses, evaluated_positions, evaluated_heights, evaluated_velocities);
}

boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > PaGMOSimulationFullState::EvaluateDetailed() {
    SampleFactory sample_factory(random_seed_);
    SampleFactory sf_sensor_simulator(sample_factory.SampleRandomInteger());
    SampleFactory sf_odesystem(sample_factory.SampleRandomInteger());

    SensorSimulatorFullState sensor_simulator(sf_sensor_simulator, asteroid_);
    ControllerFullState controller(spacecraft_maximum_thrust_, target_position_);
    if (full_state_coefficients_.size()) {
        controller.SetCoefficients(full_state_coefficients_[0], full_state_coefficients_[1], full_state_coefficients_[2]);
    }

    if (sensor_simulator.Dimensions() != controller.Dimensions()) {
        throw SizeMismatchException();
    }

    std::vector<double> time_points;
    std::vector<double> evaluated_masses;
    std::vector<Vector3D> evaluated_positions;
    std::vector<Vector3D> evaluated_velocities;
    std::vector<Vector3D> evaluated_heights;

    DataCollector collector(asteroid_, time_points, evaluated_masses, evaluated_positions, evaluated_heights, evaluated_velocities);
    SystemState system_state(initial_system_state_);

    ODESystem sys(sf_odesystem, asteroid_, sensor_simulator, controller, spacecraft_specific_impulse_, perturbation_noise_, engine_noise_);

    odeint::runge_kutta4<SystemState> stepper;
    try {
        integrate_const(stepper , sys, system_state, 0.0, simulation_time_, fixed_step_size_, collector);
    } catch (const Asteroid::Exception &exception) {
        //std::cout << "The spacecraft crashed into the asteroid's surface." << std::endl;
    } catch (const ODESystem::Exception &exception) {
        //std::cout << "The spacecraft is out of fuel." << std::endl;
    }

    return boost::make_tuple(time_points, evaluated_masses, evaluated_positions, evaluated_heights, evaluated_velocities);
}

boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > PaGMOSimulationFullState::EvaluateImpl2() {
    typedef odeint::runge_kutta_cash_karp54<SystemState> ErrorStepper;
    typedef odeint::modified_controlled_runge_kutta<ErrorStepper> ControlledStepper;

    SampleFactory sample_factory(random_seed_);
    SampleFactory sf_sensor_simulator(sample_factory.SampleRandomInteger());
    SampleFactory sf_odesystem(sample_factory.SampleRandomInteger());

    SensorSimulatorFullState sensor_simulator(sf_sensor_simulator, asteroid_);
    ControllerFullState controller(spacecraft_maximum_thrust_, target_position_);
    if (full_state_coefficients_.size()) {
        controller.SetCoefficients(full_state_coefficients_[0], full_state_coefficients_[1], full_state_coefficients_[2]);
    }

    if (sensor_simulator.Dimensions() != controller.Dimensions()) {
        throw SizeMismatchException();
    }

    std::vector<double> time_points;
    std::vector<double> evaluated_masses;
    std::vector<Vector3D> evaluated_positions;
    std::vector<Vector3D> evaluated_velocities;
    std::vector<Vector3D> evaluated_heights;

    DataCollector collector(asteroid_, time_points, evaluated_masses, evaluated_positions, evaluated_heights, evaluated_velocities);
    SystemState system_state(initial_system_state_);

    Vector3D perturbations_acceleration;
    for (unsigned int i = 0; i < 3; ++i) {
        perturbations_acceleration[i] = sample_factory.SampleNormal(0.0,perturbation_noise_);
    }

    double current_time = 0.0;
    try {
        while (current_time + interaction_interval_ <= simulation_time_) {
            ControlledStepper controlled_stepper;
            const Vector3D position = {system_state[0], system_state[1], system_state[2]};
            const Vector3D surf_pos = boost::get<0>(asteroid_.NearestPointOnSurfaceToPosition(position));
            const Vector3D height = {position[0] - surf_pos[0], position[1] - surf_pos[1], position[2] - surf_pos[2]};
            const SensorData sensor_data = sensor_simulator.Simulate(system_state, height, perturbations_acceleration, current_time);
            const Vector3D thrust = controller.GetThrustForSensorData(sensor_data);
            const double engine_noise = sample_factory.SampleNormal(0.0, engine_noise_);

            ODESystemImpl2 sys(asteroid_, perturbations_acceleration, thrust, spacecraft_specific_impulse_, engine_noise);

            integrate_adaptive(controlled_stepper, sys, system_state, current_time, current_time + interaction_interval_, minimum_step_size_, collector);

            current_time += interaction_interval_;
        }
        if (current_time < simulation_time_) {
            const double remaining_time = simulation_time_ - current_time;
            ControlledStepper controlled_stepper;
            const Vector3D position = {system_state[0], system_state[1], system_state[2]};
            const Vector3D surf_pos = boost::get<0>(asteroid_.NearestPointOnSurfaceToPosition(position));
            const Vector3D height = {position[0] - surf_pos[0], position[1] - surf_pos[1], position[2] - surf_pos[2]};
            const SensorData sensor_data = sensor_simulator.Simulate(system_state, height, perturbations_acceleration, current_time);
            const Vector3D thrust = controller.GetThrustForSensorData(sensor_data);
            const double engine_noise = sample_factory.SampleNormal(0.0, engine_noise_);

            ODESystemImpl2 sys(asteroid_, perturbations_acceleration, thrust, spacecraft_specific_impulse_, engine_noise);

            integrate_adaptive(controlled_stepper, sys, system_state, current_time, current_time + remaining_time, minimum_step_size_, collector);

            current_time += interaction_interval_;
        }

    } catch (const Asteroid::Exception &exception) {
        //std::cout << "The spacecraft crashed into the asteroid's surface." << std::endl;
    } catch (const ODESystemImpl2::Exception &exception) {
        //std::cout << "The spacecraft is out of fuel." << std::endl;
    }

    return boost::make_tuple(time_points, evaluated_masses, evaluated_positions, evaluated_heights, evaluated_velocities);
}

double PaGMOSimulationFullState::FixedStepSize() const {
    return fixed_step_size_;
}

double PaGMOSimulationFullState::MinimumStepSize() const {
    return minimum_step_size_;
}

Asteroid& PaGMOSimulationFullState::AsteroidOfSystem() {
    return asteroid_;
}

void PaGMOSimulationFullState::Init() {
    minimum_step_size_ = 0.1;
    fixed_step_size_ = 0.1;
    interaction_interval_ = 10.0;

    SampleFactory sample_factory(random_seed_);

    const Vector3D semi_axis = {sample_factory.SampleUniform(8000.0, 12000.0), sample_factory.SampleUniform(4000.0, 7500.0), sample_factory.SampleUniform(1000.0, 3500.0)};
    const double density = sample_factory.SampleUniform(1500.0, 3000.0);
    Vector2D angular_velocity_xz = {sample_factory.SampleSign() * sample_factory.SampleUniform(0.0002, 0.0008), sample_factory.SampleSign() * sample_factory.SampleUniform(0.0002, 0.0008)};
    const double time_bias = sample_factory.SampleUniform(0.0, 12.0 * 60 * 60);
    asteroid_ = Asteroid(semi_axis, density, angular_velocity_xz, time_bias);

    const double spacecraft_mass = sample_factory.SampleUniform(450.0, 500.0);
    spacecraft_maximum_thrust_ = 21.0;
    spacecraft_specific_impulse_ = 200.0;


    // orbit
    const Vector3D spacecraft_position = {sample_factory.SampleUniform(4.0, 6.0) * semi_axis[0], 0.0, 0.0};

    // random
    //const Vector3D spacecraft_position = sample_factory.SamplePointOutSideEllipsoid(semi_axis, 1.1, 4.0);

    target_position_ = spacecraft_position;

    const Vector3D angular_velocity = boost::get<0>(asteroid_.AngularVelocityAndAccelerationAtTime(0.0));
    Vector3D spacecraft_velocity = VectorCrossProduct(angular_velocity, spacecraft_position);

    // orbit
    const double norm_position = VectorNorm(spacecraft_position);
    spacecraft_velocity[0] = -spacecraft_velocity[0]; spacecraft_velocity[1] = -spacecraft_velocity[1] + sqrt(asteroid_.MassGravitationalConstant() / norm_position); spacecraft_velocity[2] = -spacecraft_velocity[2];

    // no velocity
    //spacecraft_velocity[0] *= -1; spacecraft_velocity[1] *= -1; spacecraft_velocity[2] *= -1;

    perturbation_noise_ = 1e-7;
    engine_noise_ = 0.05;

    for (unsigned int i = 0; i < 3; ++i) {
        initial_system_state_[i] = spacecraft_position[i];
        initial_system_state_[3+i] = spacecraft_velocity[i];
    }
    initial_system_state_[6] = spacecraft_mass;
}
