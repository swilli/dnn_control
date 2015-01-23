#include "odesystem.h"
#include "utility.h"
#include "constants.h"

ODESystem::ODESystem(SampleFactory &sample_factory, const Asteroid &asteroid, SensorSimulator *sensor_simulator, Controller *controller, const double &spacecraft_specific_impulse, const double &perturbation_noise, const double &engine_noise) : sample_factory_(sample_factory), asteroid_(asteroid) {
    engine_noise_ = engine_noise;
    spacecraft_specific_impulse_ = spacecraft_specific_impulse;
    sensor_simulator_ = sensor_simulator;
    controller_ = controller;
    latest_control_input_time_ = 0.0;
    min_control_interval_ = 0.2;

    for (unsigned int i = 0; i < 3; ++i) {
        perturbations_acceleration_[i] = sample_factory_.SampleNormal(0.0, perturbation_noise);
        thrust_[i] = 0.0;
    }
}

ODESystem::~ODESystem() {

}

void ODESystem::operator ()(const SystemState &state, SystemState &d_state_dt, const double &time) {
    const double mass = state[6];
    // check if spacecraft is out of fuel
    if (mass <= 0.0) {
        throw OutOfFuelException();
    }

    // 1/m
    const double coef_mass = 1.0 / mass;

    Vector3D position;
    Vector3D velocity;
    for (unsigned int i = 0; i < 3; ++i) {
        position[i] = state[i];
        velocity[i] = state[3+i];
    }

    // Compute height
    //const Vector3D surf_pos = boost::get<0>(asteroid_.NearestPointOnSurfaceToPosition(position));
    //const Vector3D height = {position[0] - surf_pos[0], position[1] - surf_pos[1], position[2] - surf_pos[2]};

    // Fg
    const Vector3D gravity_acceleration = asteroid_.GravityAccelerationAtPosition(position);

    // w, w'
    const boost::tuple<Vector3D, Vector3D> result = asteroid_.AngularVelocityAndAccelerationAtTime(time);
    const Vector3D angular_velocity = boost::get<0>(result);
    const Vector3D angular_acceleration = boost::get<1>(result);

    // Fc
    Vector3D thrust_acceleration;
    if (sensor_simulator_ != NULL && controller_ != NULL) {
        if (time - latest_control_input_time_ >= min_control_interval_) {
            latest_control_input_time_ = time;
            const SensorData sensor_data = sensor_simulator_->Simulate(state, {0.0, 0.0, 0.0}, perturbations_acceleration_, time);
            thrust_ = controller_->GetThrustForSensorData(sensor_data);
            for (unsigned int i = 0; i < 3; ++i) {
                thrust_acceleration[i] = thrust_[i] * coef_mass;
            }
        } else {
            for (unsigned int i = 0; i < 3; ++i) {
                thrust_acceleration[i] = thrust_[i] * coef_mass;
            }
        }
    } else {
        for (unsigned int i = 0; i < 3; ++i) {
            thrust_acceleration[i] = 0.0;
        }
    }

    // w' x r
    const Vector3D euler_acceleration = VectorCrossProduct(angular_acceleration, position);

    // w x (w x r)
    const Vector3D centrifugal_acceleration = VectorCrossProduct(angular_velocity, VectorCrossProduct(angular_velocity, position));

    Vector3D tmp;
    for(unsigned int i = 0; i < 3; ++i) {
        tmp[i] = angular_velocity[i] * 2.0;
    }

    // 2w x r'
    const Vector3D coriolis_acceleration = VectorCrossProduct(tmp, velocity);

    for (unsigned int i = 0; i < 3 ;++i) {
        d_state_dt[i] = state[3+i];
        d_state_dt[3+i] = perturbations_acceleration_[i] + gravity_acceleration[i] + thrust_acceleration[i]
                - coriolis_acceleration[i] - euler_acceleration[i] - centrifugal_acceleration[i];
    }

    d_state_dt[6] = -sqrt(thrust_[0] * thrust_[0] + thrust_[1] * thrust_[1] + thrust_[2] * thrust_[2]) / ((spacecraft_specific_impulse_ + spacecraft_specific_impulse_ * sample_factory_.SampleNormal(0.0, engine_noise_)) * kEarthAcceleration);
}

void ODESystem::SetThrust(const Vector3D &thrust) {
    VectorCopy3D(thrust, thrust_);
}

Vector3D ODESystem::PerturbationsAcceleration() const {
    return perturbations_acceleration_;
}

