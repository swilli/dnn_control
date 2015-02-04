#include "odesystem.h"
#include "constants.h"
#include "configuration.h"

ODESystem::ODESystem(const Asteroid &asteroid, const Vector3D &perturbations_acceleration, const Vector3D &thrust, const double &spacecraft_specific_impulse, const double &spacecraft_minimum_mass, const double &engine_noise)
    : asteroid_(asteroid) {
    perturbations_acceleration_ = perturbations_acceleration;
    thrust_ = thrust;
    spacecraft_specific_impulse_ = spacecraft_specific_impulse;
    spacecraft_minimum_mass_ = spacecraft_minimum_mass;
    engine_noise_ = engine_noise;
}

ODESystem::ODESystem(const ODESystem &other)
    : asteroid_(other.asteroid_) {
    perturbations_acceleration_ = other.perturbations_acceleration_;
    thrust_ = other.thrust_;
    spacecraft_specific_impulse_ = other.spacecraft_specific_impulse_;
    spacecraft_minimum_mass_ = other.spacecraft_minimum_mass_;
    engine_noise_ = other.engine_noise_;
}

ODESystem::~ODESystem() {

}

void ODESystem::operator ()(const SystemState &state, SystemState &d_state_dt, const double &time) {
    const double mass = state[6];
    // check if spacecraft is out of fuel
    if (mass <= spacecraft_minimum_mass_) {
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

    // Fg
    const Vector3D gravity_acceleration = asteroid_.GravityAccelerationAtPosition(position);

    // w, w'
    const boost::tuple<Vector3D, Vector3D> result = asteroid_.AngularVelocityAndAccelerationAtTime(time);
    const Vector3D angular_velocity = boost::get<0>(result);
    const Vector3D angular_acceleration = boost::get<1>(result);

    // Fc
    Vector3D thrust_acceleration;
    for (unsigned int i = 0; i < 3; ++i) {
        thrust_acceleration[i] = thrust_[i] * coef_mass;
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

#if ODES_FUEL_ENABLED
    d_state_dt[6] = -sqrt(thrust_[0] * thrust_[0] + thrust_[1] * thrust_[1] + thrust_[2] * thrust_[2]) / ((spacecraft_specific_impulse_ + spacecraft_specific_impulse_ * engine_noise_) * kEarthAcceleration);
#else
    d_state_dt[6] = 0.0;
#endif
}

