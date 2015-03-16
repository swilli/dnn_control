#include "pagmosimulation.h"
#include "samplefactory.h"
#include "odeint.h"
#include "modifiedcontrolledrungekutta.h"
#include "odesystem.h"
#include "constants.h"
#include "configuration.h"

PaGMOSimulation::PaGMOSimulation(const unsigned int &random_seed)
    : random_seed_(random_seed), simulation_time_(0.0) {
    Init();
    simulation_time_ = (int) (asteroid_.RotationalPeriod() * 0.5);
}

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

double PaGMOSimulation::ControlFrequency() const {
    return control_frequency_;
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

void PaGMOSimulation::SetSimulationTime(const double &simulation_time) {
    simulation_time_ = simulation_time;
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

Vector3D PaGMOSimulation::TargetPosition() const {
    return target_position_;
}

void PaGMOSimulation::Init() {
    minimum_step_size_ = 0.1;
    fixed_step_size_ = 0.1;
    control_frequency_ = 1.0;

    SampleFactory sample_factory(random_seed_);

    const double c_semi_axis = sample_factory.SampleUniform(100.0, 8000.0);
    const double b_semi_axis_n = sample_factory.SampleUniform(1.1, 2.0);
    const double a_semi_axis_n = sample_factory.SampleUniform(1.1 * b_semi_axis_n, 4.0);
    const Vector3D semi_axis = {a_semi_axis_n * c_semi_axis, b_semi_axis_n * c_semi_axis, c_semi_axis};
    const double density = sample_factory.SampleUniform(1500.0, 3000.0);
    const double magn_angular_velocity = 0.85 * sqrt((kGravitationalConstant * 4.0/3.0 * kPi * semi_axis[0] * semi_axis[1] * semi_axis[2] * density) / (semi_axis[0] * semi_axis[0] * semi_axis[0]));
    const Vector2D angular_velocity_xz = {sample_factory.SampleSign() * sample_factory.SampleUniform(magn_angular_velocity * 0.5, magn_angular_velocity), sample_factory.SampleSign() * sample_factory.SampleUniform(magn_angular_velocity * 0.5, magn_angular_velocity)};
    const double time_bias = sample_factory.SampleUniform(0.0, 12.0 * 60 * 60);
    asteroid_ = Asteroid(semi_axis, density, angular_velocity_xz, time_bias);

    spacecraft_maximum_mass_ = sample_factory.SampleUniform(450.0, 500.0);
    spacecraft_minimum_mass_ = spacecraft_maximum_mass_ * 0.5;
    spacecraft_maximum_thrust_ = 21.0;
    spacecraft_specific_impulse_ = 200.0;
    spacecraft_engine_noise_ = 0.05;

    perturbation_mean_ = 1e-6;
    perturbation_noise_ = 1e-7;

#if PGMOS_IC_VELOCITY_TYPE == PGMOS_IC_INERTIAL_ORBITAL_VELOCITY
    // higher for orbit, so we don't crash into the asteroid
    const boost::tuple<Vector3D, double, double, double> sampled_point = sample_factory.SamplePointOutSideEllipsoid(semi_axis, 2.0, 4.0);
    target_position_ = boost::get<0>(sampled_point);
#else
    // random
    const boost::tuple<Vector3D, double, double, double> sampled_point = sample_factory.SamplePointOutSideEllipsoid(semi_axis, 1.1, 4.0);
    target_position_ = boost::get<0>(sampled_point);
#endif

#if PGMOS_IC_POSITION_OFFSET_ENABLED
    // spacecraft has uniformly distributed offset to target position
    Vector3D spacecraft_position;
    for (unsigned int i = 0 ; i < 3; ++i) {
        spacecraft_position[i] = target_position_[i] + sample_factory.SampleUniform(-3.0, 3.0);
    }
#else
    // spacecraft starts at target position
    const Vector3D spacecraft_position = target_position_;
#endif


#if PGMOS_IC_VELOCITY_TYPE == PGMOS_IC_INERTIAL_ORBITAL_VELOCITY
    // orbital velocity in inertial frame
    const Vector3D angular_velocity = boost::get<0>(asteroid_.AngularVelocityAndAccelerationAtTime(0.0));
    Vector3D spacecraft_velocity = VectorCrossProduct(angular_velocity, spacecraft_position);

    const double norm_position = VectorNorm(spacecraft_position);
    const double magn_orbital_vel = sqrt(asteroid_.MassGravitationalConstant() / norm_position);
    Vector3D orth_pos = {sample_factory.SampleSign() * sample_factory.SampleUniform(1e-10, 1.0), sample_factory.SampleSign() * sample_factory.SampleUniform(1e-10, 1.0), sample_factory.SampleSign() * sample_factory.SampleUniform(1e-10, 1.0)};

    std::vector<unsigned int> choices;
    if (spacecraft_position[2] > 1.0 || spacecraft_position[2] < -1.0) {
        choices.push_back(0);
    }
    if (spacecraft_position[1] > 1.0 || spacecraft_position[1] < -1.0) {
        choices.push_back(1);
    }
    if (spacecraft_position[0] > 1.0 || spacecraft_position[0] < -1.0) {
        choices.push_back(2);
    }
    const unsigned int choice = choices.at(sample_factory.SampleRandomInteger() % choices.size());
    switch (choice) {
        case 0:
        orth_pos[2] = -(orth_pos[0]*spacecraft_position[0] + orth_pos[1]*spacecraft_position[1]) / spacecraft_position[2];
        break;
    case 1:
        orth_pos[1] = -(orth_pos[0]*spacecraft_position[0] + orth_pos[2]*spacecraft_position[2]) / spacecraft_position[1];
        break;
    case 2:
        orth_pos[0] = -(orth_pos[1]*spacecraft_position[1] + orth_pos[2]*spacecraft_position[2]) / spacecraft_position[0];
        break;
    }
    const double coef_norm_orth_pos = 1.0 / VectorNorm(orth_pos);
    for (unsigned int i = 0; i < 3; ++i) {
        orth_pos[i] *= coef_norm_orth_pos;
    }
    spacecraft_velocity[0] = -spacecraft_velocity[0] + orth_pos[0] * magn_orbital_vel;
    spacecraft_velocity[1] = -spacecraft_velocity[1] + orth_pos[1] * magn_orbital_vel;
    spacecraft_velocity[2] = -spacecraft_velocity[2] + orth_pos[2] * magn_orbital_vel;

#elif PGMOS_IC_VELOCITY_TYPE == PGMOS_IC_INERTIAL_ZERO_VELOCITY
    // zero velocity in inertial frame
    const Vector3D angular_velocity = boost::get<0>(asteroid_.AngularVelocityAndAccelerationAtTime(0.0));
    Vector3D spacecraft_velocity = VectorCrossProduct(angular_velocity, spacecraft_position);

    spacecraft_velocity[0] *= -1; spacecraft_velocity[1] *= -1; spacecraft_velocity[2] *= -1;

#elif PGMOS_IC_VELOCITY_TYPE == PGMOS_IC_BODY_ZERO_VELOCITY
    // zero velocity in body frame
    const Vector3D spacecraft_velocity = {0.0, 0.0, 0.0};

#elif PGMOS_IC_VELOCITY_TYPE == PGMOS_IC_BODY_RANDOM_VELOCITY
    // uniformly distributed random velocity in body frame
    Vector3D spacecraft_velocity;
    for (unsigned int i = 0 ; i < 3; ++i) {
        spacecraft_velocity[i] = sample_factory.SampleUniform(-0.3, 0.3);
    }
#endif

    for (unsigned int i = 0; i < 3; ++i) {
        initial_system_state_[i] = spacecraft_position[i];
        initial_system_state_[3+i] = spacecraft_velocity[i];
    }

    initial_system_state_[6] = spacecraft_maximum_mass_;
}
