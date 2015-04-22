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
    simulation_time_ = 10.0; //(int) (asteroid_.EstimatedMainMotionPeriod() * 0.5);
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

double PaGMOSimulation::SpacecraftMaximumThrust() const {
    return spacecraft_maximum_thrust_;
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

void PaGMOSimulation::SetSimulationTime(const double &simulation_time) {
    simulation_time_ = simulation_time;
}

void PaGMOSimulation::Init() {
    minimum_step_size_ = 0.1;
    fixed_step_size_ = 0.1;
    control_frequency_ = 1.0;

    SampleFactory asteroid_sf(random_seed_);
    SampleFactory spacecraft_sf(random_seed_);

    const double c_semi_axis = asteroid_sf.SampleUniformReal(100.0, 8000.0);
    const double b_semi_axis_n = asteroid_sf.SampleUniformReal(1.1, 2.0);
    const double a_semi_axis_n = asteroid_sf.SampleUniformReal(1.1 * b_semi_axis_n, 4.0);
    const Vector3D &semi_axis = {a_semi_axis_n * c_semi_axis, b_semi_axis_n * c_semi_axis, c_semi_axis};
    const double density = asteroid_sf.SampleUniformReal(1500.0, 3000.0);
    const double magn_angular_velocity = 0.85 * sqrt((kGravitationalConstant * 4.0/3.0 * kPi * semi_axis[0] * semi_axis[1] * semi_axis[2] * density) / (semi_axis[0] * semi_axis[0] * semi_axis[0]));
    const Vector2D &angular_velocity_xz = {asteroid_sf.SampleSign() * asteroid_sf.SampleUniformReal(magn_angular_velocity * 0.5, magn_angular_velocity), asteroid_sf.SampleSign() * asteroid_sf.SampleUniformReal(magn_angular_velocity * 0.5, magn_angular_velocity)};
    const double time_bias = asteroid_sf.SampleUniformReal(0.0, 12.0 * 60 * 60);
    asteroid_ = Asteroid(semi_axis, density, angular_velocity_xz, time_bias);

    spacecraft_maximum_mass_ = spacecraft_sf.SampleUniformReal(450.0, 500.0);
    spacecraft_minimum_mass_ = spacecraft_maximum_mass_ * 0.5;
    spacecraft_maximum_thrust_ = 21.0;
    spacecraft_specific_impulse_ = 200.0;
    spacecraft_engine_noise_ = 0.05;

    perturbation_mean_ = 1e-6;
    perturbation_noise_ = 1e-7;

#if PGMOS_IC_VELOCITY_TYPE == PGMOS_IC_INERTIAL_ORBITAL_VELOCITY
    // higher for orbit, so we don't crash into the asteroid
    const boost::tuple<Vector3D, double, double, double> sampled_point = spacecraft_sf.SamplePointOutSideEllipsoid(semi_axis, 2.0, 4.0);
    target_position_ = boost::get<0>(sampled_point);
#else
    // random
    const boost::tuple<Vector3D, double, double, double> sampled_point = spacecraft_sf.SamplePointOutSideEllipsoid(semi_axis, 1.1, 4.0);
    target_position_ = boost::get<0>(sampled_point);
#endif

#if PGMOS_IC_ENABLE_POSITION_OFFSET
    // spacecraft has uniformly distributed offset to target position
    Vector3D spacecraft_position;
    for (unsigned int i = 0 ; i < 3; ++i) {
        spacecraft_position[i] = target_position_[i] + spacecraft_sf.SampleUniformReal(-3.0, 3.0);
    }
#else
    // spacecraft starts at target position
    const Vector3D &spacecraft_position = target_position_;
#endif


#if PGMOS_IC_VELOCITY_TYPE == PGMOS_IC_INERTIAL_ORBITAL_VELOCITY
    // orbital velocity in inertial frame
    const Vector3D angular_velocity = boost::get<0>(asteroid_.AngularVelocityAndAccelerationAtTime(0.0));
    Vector3D spacecraft_velocity = VectorCrossProduct(angular_velocity, spacecraft_position);

    const double norm_position = VectorNorm(spacecraft_position);
    const double magn_orbital_vel = sqrt(asteroid_.MassGravitationalConstant() / norm_position);
    Vector3D orth_pos = {spacecraft_sf.SampleSign() * spacecraft_sf.SampleUniform(1e-10, 1.0), spacecraft_sf.SampleSign() * spacecraft_sf.SampleUniform(1e-10, 1.0), spacecraft_sf.SampleSign() * spacecraft_sf.SampleUniform(1e-10, 1.0)};

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
    const unsigned int choice = choices.at(spacecraft_sf.SampleRandomInteger() % choices.size());
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
    orth_pos = VectorNormalized(orth_pos);
    spacecraft_velocity = VectorSub(VectorMul(magn_orbital_vel, orth_pos), spacecraft_velocity);


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
        spacecraft_velocity[i] = spacecraft_sf.SampleUniformReal(-0.3, 0.3);
    }

#elif PGMOS_IC_VELOCITY_TYPE == PGMOS_IC_BODY_PROPORTIONAL_VELOCITY
    // velocity proportional to height
    const double up_scale = 1000000.0;
    const double max_initital_sensor_value = 10.0;
    const double norm_height = boost::get<1>(asteroid_.NearestPointOnSurfaceToPosition(spacecraft_position));
    const double magn_velocity = max_initital_sensor_value * norm_height / up_scale;
    Vector3D spacecraft_velocity;
    for (unsigned int i = 0 ; i < 3; ++i) {
        spacecraft_velocity[i] = spacecraft_sf.SampleUniform(-magn_velocity, magn_velocity);
    }

#endif

    for (unsigned int i = 0; i < 3; ++i) {
        initial_system_state_[i] = spacecraft_position[i];
        initial_system_state_[3+i] = spacecraft_velocity[i];
    }

    initial_system_state_[6] = spacecraft_maximum_mass_;

    enable_sensor_noise_ = PGMOS_ENABLE_NOISE;

#if PGMOS_ENABLE_RELATIVE_POSITION
    sensor_types_.insert(SensorSimulator::SensorType::RelativePosition);
#endif
#if PGMOS_ENABLE_VELOCITY
    sensor_types_.insert(SensorSimulator::SensorType::Velocity);
#endif
#if PGMOS_ENABLE_OPTICAL_FLOW
    sensor_types_.insert(SensorSimulator::SensorType::OpticalFlow);
#endif
#if PGMOS_ENABLE_ACCELEROMETER
    sensor_types_.insert(SensorSimulator::SensorType::Acceleration);
#endif

    // Means: [-2.34315594537, -2.27956514526, -21.2874117161, -2.3236143943, -3.2269601852, 13.7950111531]
    //Stdevs: [5454.37836105, 7966.07979339, 21295.1854193, 10658.0857674, 18415.6359962, 5702.25191872]

#if PGMOS_STANDARDIZE_SENSOR_VALUES
    sensor_value_transformations_[SensorSimulator::SensorType::OpticalFlow] = {
        {-2.34315594537, 5454.37836105},
        {-2.27956514526, 7966.07979339},
        {-21.2874117161, 21295.1854193},
        {-2.3236143943, 10658.0857674},
        {-3.2269601852, 18415.6359962},
        {13.7950111531, 5702.25191872}
    };
#endif
}
