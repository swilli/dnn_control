#include "simulation.h"
#include "utility.h"

// DEFINE CONTROLLER AND SENSORSIMULATOR USED IN THE SIMULATION
#include "controllerfullstate.h"
#include "sensorsimulatorfullstate.h"

Simulation::Simulation(const unsigned int &random_seed) : sample_factory_(SampleFactory(random_seed)) {
    simulation_time_ = 24.0 * 60.0 * 60.0;

    const Vector3D semi_axis = {sample_factory_.SampleUniform(8000.0, 12000.0), sample_factory_.SampleUniform(4000.0, 7500.0), sample_factory_.SampleUniform(1000.0, 3500.0)};
    const double density = sample_factory_.SampleUniform(1500.0, 3000.0);
    Vector2D angular_velocity_xz = {sample_factory_.SampleSign() * sample_factory_.SampleUniform(0.0002, 0.0008), sample_factory_.SampleSign() * sample_factory_.SampleUniform(0.0002, 0.0008)};
    const double time_bias = sample_factory_.SampleUniform(0.0, 12.0 * 60 * 60);
    asteroid_ = Asteroid(semi_axis, density, angular_velocity_xz, time_bias);

    const double spacecraft_mass = sample_factory_.SampleUniform(450.0, 500.0);
    const double spacecraft_specific_impulse = 200.0;
    const double spacecraft_maximum_thrust = 21.0;

    const Vector3D spacecraft_position = sample_factory_.SamplePointOutSideEllipsoid(semi_axis, 4.0);
    const double norm_position = VectorNorm(spacecraft_position);
    const Vector3D angular_velocity = boost::get<0>(asteroid_.AngularVelocityAndAccelerationAtTime(0.0));
    Vector3D spacecraft_velocity = CrossProduct(angular_velocity, spacecraft_position);

    // orbit
    //spacecraft_velocity[0] = -spacecraft_velocity[0]; spacecraft_velocity[1] = -spacecraft_velocity[1] + sqrt(asteroid_.MassGravitationalConstant() / norm_position); spacecraft_velocity[2] = -spacecraft_velocity[2];

    // no velocity
    spacecraft_velocity[0] *= -1; spacecraft_velocity[1] *= -1; spacecraft_velocity[2] *= -1;

    Vector3D target_position;
    for (unsigned int i = 0; i < 3; ++i) {
        target_position[i] = sample_factory_.SampleUniform(spacecraft_position[i] - 3.0, spacecraft_position[i] + 3.0);
    }

    // ADAPT CONTROLLER AND SENSOR SIMULATOR HERE
    controller_ = new ControllerFullState(spacecraft_maximum_thrust, target_position);

    SensorSimulatorFullState::SensorNoiseConfiguration sensor_noise;
    for (unsigned int i = 0; i < sensor_noise.size(); ++i) {
        sensor_noise[i] = 0.05;
    }
    sensor_simulator_ = new SensorSimulatorFullState(sample_factory_, asteroid_, sensor_noise);


    const double perturbation_noise = 1e-7;
    const double engine_noise = 0.05;

    for (unsigned int i = 0; i < 3; ++i) {
        initial_system_state_[i] = spacecraft_position[i];
        initial_system_state_[3+i] = spacecraft_velocity[i];
    }
    initial_system_state_[6] = spacecraft_mass;

    system_ = ODESystem(&sample_factory_, asteroid_, sensor_simulator_, controller_, spacecraft_specific_impulse, perturbation_noise, engine_noise);
}

Simulation::~Simulation() {
    if (controller_ != NULL) {
        delete controller_;
    }
    if (sensor_simulator_ != NULL) {
        delete sensor_simulator_;
    }
}

Asteroid &Simulation::AsteroidOfSystem() {
    return asteroid_;
}

