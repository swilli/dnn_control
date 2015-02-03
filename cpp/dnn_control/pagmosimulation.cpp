#include "pagmosimulation.h"
#include "samplefactory.h"
#include "odeint.h"
#include "modifiedcontrolledrungekutta.h"
#include "odesystem.h"

// Include here sensor simulator for which you want to produce sensor data
#include "sensorsimulatorautoencoder.h"

PaGMOSimulation::PaGMOSimulation(const unsigned int &random_seed, const double &simulation_time)
    : random_seed_(random_seed), simulation_time_(simulation_time) {
    Init();
}

PaGMOSimulation::~PaGMOSimulation() {

}

double PaGMOSimulation::FixedStepSize() const {
    return fixed_step_size_;
}

double PaGMOSimulation::MinimumStepSize() const {
    return minimum_step_size_;
}

double PaGMOSimulation::InteractionInterval() const {
    return interaction_interval_;
}

Asteroid &PaGMOSimulation::AsteroidOfSystem() {
    return asteroid_;
}

double PaGMOSimulation::SpacecraftMaximumMass() const {
    return spacecraft_maximum_mass_;
}

double PaGMOSimulation::SpacecraftMinimumMass() const {
    return spacecraft_minimum_mass_;
}

unsigned int PaGMOSimulation::RandomSeed() const {
    return random_seed_;
}

SystemState PaGMOSimulation::InitialSystemState() const {
    return initial_system_state_;
}

double PaGMOSimulation::SimulationTime() const {
    return simulation_time_;
}

std::vector<SensorData> PaGMOSimulation::GenerateSensorDataSet() {
    typedef odeint::runge_kutta_cash_karp54<SystemState> ErrorStepper;
    typedef odeint::modified_controlled_runge_kutta<ErrorStepper> ControlledStepper;

    SampleFactory sample_factory(random_seed_);
    SampleFactory sf_sensor_simulator(sample_factory.SampleRandomInteger());


    SensorSimulatorAutoencoder sensor_simulator(sf_sensor_simulator, asteroid_);


    const unsigned int num_iterations = simulation_time_ / interaction_interval_;
    std::vector<SensorData> evaluated_sensor_values(num_iterations + 1);


    SystemState system_state(initial_system_state_);

    Vector3D perturbations_acceleration;
    const Vector3D thrust = {0.0, 0.0, 0.0};

    double current_time = 0.0;
    double current_time_observer = 0.0;
    Observer observer(current_time_observer);
    unsigned int iteration = 0;
    bool exception_thrown = false;
    try {
        for (iteration = 0; iteration < num_iterations; ++iteration) {
            const Vector3D &position = {system_state[0], system_state[1], system_state[2]};
            const double &mass = system_state[6];

            const Vector3D &surf_pos = boost::get<0>(asteroid_.NearestPointOnSurfaceToPosition(position));
            const Vector3D &height = {position[0] - surf_pos[0], position[1] - surf_pos[1], position[2] - surf_pos[2]};

            for (unsigned int i = 0; i < 3; ++i) {
                perturbations_acceleration[i] = mass * sample_factory.SampleNormal(perturbation_mean_, perturbation_noise_);
            }

            evaluated_sensor_values[iteration] = sensor_simulator.Simulate(system_state, height, perturbations_acceleration, current_time);

            const double engine_noise = sample_factory.SampleNormal(0.0, spacecraft_engine_noise_);

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
        evaluated_sensor_values.resize(iteration + 2);
    }

    const Vector3D &position = {system_state[0], system_state[1], system_state[2]};

    const Vector3D &surf_pos = boost::get<0>(asteroid_.NearestPointOnSurfaceToPosition(position));
    const Vector3D &height = {position[0] - surf_pos[0], position[1] - surf_pos[1], position[2] - surf_pos[2]};


    evaluated_sensor_values.back() = sensor_simulator.Simulate(system_state, height, perturbations_acceleration, current_time_observer);

    return evaluated_sensor_values;

}

void PaGMOSimulation::Init() {
    minimum_step_size_ = 0.1;
    fixed_step_size_ = 0.1;
    interaction_interval_ = 1.0;

    SampleFactory sample_factory(random_seed_);

    const Vector3D semi_axis = {sample_factory.SampleUniform(8000.0, 12000.0), sample_factory.SampleUniform(4000.0, 7500.0), sample_factory.SampleUniform(1000.0, 3500.0)};
    const double density = sample_factory.SampleUniform(1500.0, 3000.0);
    Vector2D angular_velocity_xz = {sample_factory.SampleSign() * sample_factory.SampleUniform(0.0002, 0.0008), sample_factory.SampleSign() * sample_factory.SampleUniform(0.0002, 0.0008)};
    const double time_bias = sample_factory.SampleUniform(0.0, 12.0 * 60 * 60);
    asteroid_ = Asteroid(semi_axis, density, angular_velocity_xz, time_bias);

    spacecraft_maximum_mass_ = sample_factory.SampleUniform(450.0, 500.0);
    spacecraft_minimum_mass_ = spacecraft_maximum_mass_ * 0.5;
    spacecraft_maximum_thrust_ = 21.0;
    spacecraft_specific_impulse_ = 200.0;
    spacecraft_engine_noise_ = 0.05;

    perturbation_mean_ = 1e-9;
    perturbation_noise_ = 1e-11;

#if PS_ORBITAL_INITIAL_CONDITIONS == true
    // orbit
    const Vector3D spacecraft_position = sample_factory.SamplePointOutSideEllipsoid(semi_axis, 2.0, 4.0);
#else
    // random
    const Vector3D spacecraft_position = sample_factory.SamplePointOutSideEllipsoid(semi_axis, 1.1, 4.0);
#endif

    target_position_ = spacecraft_position;

    const Vector3D angular_velocity = boost::get<0>(asteroid_.AngularVelocityAndAccelerationAtTime(0.0));
    Vector3D spacecraft_velocity = VectorCrossProduct(angular_velocity, spacecraft_position);

#if PS_ORBITAL_INITIAL_CONDITIONS == true
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

    for (unsigned int i = 0; i < 3; ++i) {
        initial_system_state_[i] = spacecraft_position[i];
        initial_system_state_[3+i] = spacecraft_velocity[i];
    }
    initial_system_state_[6] = spacecraft_maximum_mass_;
}

