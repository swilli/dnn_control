#include "simulation.h"
#include "utility.h"

Simulation::Simulation(const unsigned int &random_seed) : random_seed_(random_seed) {
    simulation_time_ = 24.0 * 60.0 * 60.0;

    SampleFactory sample_factory(random_seed);

    const Vector3D semi_axis = {sample_factory.SampleUniform(8000.0, 12000.0), sample_factory.SampleUniform(4000.0, 7500.0), sample_factory.SampleUniform(1000.0, 3500.0)};
    const double density = sample_factory.SampleUniform(1500.0, 3000.0);
    const Vector2D angular_velocity_xz = {sample_factory.SampleSign() * sample_factory.SampleUniform(0.0002, 0.0008), sample_factory.SampleSign() * sample_factory.SampleUniform(0.0002, 0.0008)};
    const double time_bias = sample_factory.SampleUniform(0.0, 12.0 * 60 * 60);
    asteroid_ = Asteroid(semi_axis, density, angular_velocity_xz, time_bias);

    const double spacecraft_mass = sample_factory.SampleUniform(450.0, 500.0);
    spacecraft_maximum_thrust_ = 21.0;
    spacecraft_specific_impulse_ = 200.0;

    const Vector3D spacecraft_position = {sample_factory.SampleUniform(4.0, 6.0) * semi_axis[0], 0.0, 0.0}; //sample_factory.SamplePointOutSideEllipsoid(semi_axis, 1.1, 4.0);
    const Vector3D angular_velocity = boost::get<0>(asteroid_.AngularVelocityAndAccelerationAtTime(0.0));
    Vector3D spacecraft_velocity = VectorCrossProduct(angular_velocity, spacecraft_position);

    // orbit
    const double norm_position = VectorNorm(spacecraft_position);
    spacecraft_velocity[0] = -spacecraft_velocity[0]; spacecraft_velocity[1] = -spacecraft_velocity[1] + sqrt(asteroid_.MassGravitationalConstant() / norm_position); spacecraft_velocity[2] = -spacecraft_velocity[2];

    // no velocity
    //spacecraft_velocity[0] *= -1; spacecraft_velocity[1] *= -1; spacecraft_velocity[2] *= -1;

    for (unsigned int i = 0; i < 3; ++i) {
        target_position_[i] = sample_factory.SampleUniform(spacecraft_position[i] - 3.0, spacecraft_position[i] + 3.0);
    }

    perturbation_noise_ = 1e-7;
    engine_noise_ = 0.05;

    for (unsigned int i = 0; i < 3; ++i) {
        initial_system_state_[i] = spacecraft_position[i];
        initial_system_state_[3+i] = spacecraft_velocity[i];
    }
    initial_system_state_[6] = spacecraft_mass;
}

Simulation::~Simulation() {

}

Asteroid &Simulation::AsteroidOfSystem() {
    return asteroid_;
}
