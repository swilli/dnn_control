#include "odesystem.h"
#include "utility.h"
#include <math.h>

ODESystem::ODESystem(Asteroid &asteroid) : asteroid_(asteroid) {
    coef_earth_acceleration_mul_specific_impulse_ = 0.0;
    for (int i = 0; i < 3; ++i) {
        thrust_[i] = 0.0;
        perturbations_acceleration_[i] = 0.0;
        state_[i] = 0.0;
        state_[3+i] = 0.0;
    }
    coef_earth_acceleration_mul_specific_impulse_ = 0.0;
}

void ODESystem::operator()(const State &state, State &d_state_dt, const double &time) const {
    // This operator implements eq(1) of "Control of Hovering Spacecraft Using Altimetry" by S. Sawai et. al.
    // r'' + 2w x r' + w' x r + w x (w x r) = Fc + Fg, whereas Fc = control acceleration and Fg = gravity acceleration

    Vector3D gravity_acceleration;
    Vector3D thrust_acceleration;
    Vector3D euler_acceleration;
    Vector3D centrifugal_acceleration;
    Vector3D coriolis_acceleration;
    Vector3D angular_velocity;
    Vector3D angular_acceleration;
    Vector3D tmp;

    // 1/m
    const double coef_mass = 1.0 / state[6];

    Vector3D position;
    Vector3D velocity;
    for(int i = 0; i < 3; ++i) {
        position[i] = state[i];
        velocity[i] = state[3+i];
    }

    // g
    asteroid_.GravityAtPosition(position,gravity_acceleration);

    // w, w'
    asteroid_.AngularVelocityAndAccelerationAtTime(time, angular_velocity, angular_acceleration);

    // Fg, Fc
    for(int i = 0; i < 3; ++i) {
        gravity_acceleration[i] *= coef_mass;
        thrust_acceleration[i] = thrust_[i] * coef_mass;
    }

    // w' x r
    CrossProduct(angular_acceleration, position, euler_acceleration);

    // w x (w x r)
    CrossProduct(angular_velocity, position, tmp);
    CrossProduct(angular_velocity, tmp, centrifugal_acceleration);

    for(int i = 0; i < 3; ++i) {
        tmp[i] = angular_velocity[i] * 2.0;
    }

    // 2w x r'
    CrossProduct(tmp, velocity, coriolis_acceleration);

    for (int i = 0; i < 3 ;++i) {
        d_state_dt[i] = state[3+i];
        d_state_dt[3+i] = perturbations_acceleration_[i] + gravity_acceleration[i] + thrust_acceleration[i]
                - coriolis_acceleration[i] - euler_acceleration[i] - centrifugal_acceleration[i];
    }

    d_state_dt[6] = sqrt(thrust_[0] * thrust_[0] + thrust_[1] * thrust_[1] + thrust_[2] * thrust_[2]) * coef_earth_acceleration_mul_specific_impulse_;
}
