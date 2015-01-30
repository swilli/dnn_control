#include "pagmosimulation.h"
#include "samplefactory.h"

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

double PaGMOSimulation::SpacecraftMass() const {
    return spacecraft_mass_;
}

double PaGMOSimulation::SpacecraftMinimumMass() const {
    return spacecraft_minimum_mass_;
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

    spacecraft_mass_ = sample_factory.SampleUniform(450.0, 500.0);
    spacecraft_minimum_mass_ = spacecraft_mass_ * 0.5;
    spacecraft_maximum_thrust_ = 21.0;
    spacecraft_specific_impulse_ = 200.0;
    spacecraft_engine_noise_ = 0.05;

    perturbation_mean_ = 1e-9;
    perturbation_noise_ = 1e-11;

#if PS_TEST_FOR_ORBIT == 1
    // orbit
    const Vector3D spacecraft_position = sample_factory.SamplePointOutSideEllipsoid(semi_axis, 2.0, 4.0);
#else
    // random
    const Vector3D spacecraft_position = sample_factory.SamplePointOutSideEllipsoid(semi_axis, 1.1, 4.0);
#endif

    target_position_ = spacecraft_position;

    const Vector3D angular_velocity = boost::get<0>(asteroid_.AngularVelocityAndAccelerationAtTime(0.0));
    Vector3D spacecraft_velocity = VectorCrossProduct(angular_velocity, spacecraft_position);

#if PS_TEST_FOR_ORBIT == 1
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
    initial_system_state_[6] = spacecraft_mass_;
}

