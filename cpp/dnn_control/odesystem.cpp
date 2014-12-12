#include "odesystem.h"
#include "utility.h"
#include "constants.h"

#include <math.h>
#include <iostream>

ODESystem::ODESystem(Asteroid asteroid, const double &control_noise) : asteroid_(asteroid), control_distribution_(boost::mt19937(time(0)),boost::normal_distribution<>(0.0, control_noise)) {
    spacecraft_specific_impulse_ = 0.0;
    for (int i = 0; i < 3; ++i) {
        thrust_[i] = 0.0;
        perturbations_acceleration_[i] = 0.0;
        state_[i] = 0.0;
        state_[3+i] = 0.0;
    }
}

void ODESystem::operator()(const State &state, State &d_state_dt, const double &time) {
    // This operator implements eq(1) of "Control of Hovering Spacecraft Using Altimetry" by S. Sawai et. al.
    // r'' + 2w x r' + w' x r + w x (w x r) = Fc + Fg, whereas Fc = control acceleration and Fg = gravity acceleration


    // 1/m
    const double coef_mass = 1.0 / state[6];

    Vector3D position;
    Vector3D velocity;
    for(int i = 0; i < 3; ++i) {
        position[i] = state[i];
        velocity[i] = state[3+i];
    }

    // g
    Vector3D gravity_acceleration = asteroid_.GravityAtPosition(position);

    // w, w'
    const boost::tuple<Vector3D, Vector3D> result = asteroid_.AngularVelocityAndAccelerationAtTime(time);
    Vector3D angular_velocity = boost::get<0>(result);
    Vector3D angular_acceleration = boost::get<1>(result);

    // Fg, Fc
    Vector3D thrust_acceleration;
    for(int i = 0; i < 3; ++i) {
        gravity_acceleration[i] *= coef_mass;
        thrust_acceleration[i] = thrust_[i] * coef_mass;
    }

    // w' x r
    const Vector3D euler_acceleration = CrossProduct(angular_acceleration, position);

    // w x (w x r)
    const Vector3D centrifugal_acceleration = CrossProduct(angular_velocity, CrossProduct(angular_velocity, position));

    Vector3D tmp;
    for(int i = 0; i < 3; ++i) {
        tmp[i] = angular_velocity[i] * 2.0;
    }

    // 2w x r'
    const Vector3D coriolis_acceleration = CrossProduct(tmp, velocity);

    for (int i = 0; i < 3 ;++i) {
        d_state_dt[i] = state[3+i];
        d_state_dt[3+i] = perturbations_acceleration_[i] + gravity_acceleration[i] + thrust_acceleration[i]
                - coriolis_acceleration[i] - euler_acceleration[i] - centrifugal_acceleration[i];
    }

    d_state_dt[6] = -sqrt(thrust_[0] * thrust_[0] + thrust_[1] * thrust_[1] + thrust_[2] * thrust_[2]) / ((spacecraft_specific_impulse_ + spacecraft_specific_impulse_ * control_distribution_()) * k_earth_acceleration);
}
