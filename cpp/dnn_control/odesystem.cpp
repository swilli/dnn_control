#include "odesystem.h"
#include "constants.h"

ODESystem::ODESystem(SampleFactory &sample_factory, const Asteroid &asteroid, SensorSimulator &sensor_simulator, Controller &controller, const double &spacecraft_specific_impulse, const double &perturbation_noise, const double &engine_noise)
    : sample_factory_(sample_factory), asteroid_(asteroid), sensor_simulator_(sensor_simulator), controller_(controller) {

    engine_noise_ = engine_noise;
    spacecraft_specific_impulse_ = spacecraft_specific_impulse;
    latest_control_input_time_ = 0.0;
    min_control_interval_ = 0.2;

    for (unsigned int i = 0; i < 3; ++i) {
        thrust_[i] = 0.0;
        perturbations_acceleration_[i] = 0.0; //sample_factory_.SampleNormal(0.0, perturbation_noise);
    }
}

ODESystem::ODESystem(const ODESystem &other)
    : sample_factory_(other.sample_factory_), asteroid_(other.asteroid_), sensor_simulator_(other.sensor_simulator_), controller_(other.controller_) {

    engine_noise_ = other.engine_noise_;
    spacecraft_specific_impulse_ = other.spacecraft_specific_impulse_;
    latest_control_input_time_ = other.latest_control_input_time_;
    min_control_interval_ = other.min_control_interval_;
    thrust_ = other.thrust_;
    perturbations_acceleration_ = other.perturbations_acceleration_;
}

ODESystem::~ODESystem() {

}

void ODESystem::operator ()(const SystemState &state, SystemState &d_state_dt, const double &time) {
    //std::cout << time << std::endl;

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
    const Vector3D surf_pos = boost::get<0>(asteroid_.NearestPointOnSurfaceToPosition(position));
    const Vector3D height = {position[0] - surf_pos[0], position[1] - surf_pos[1], position[2] - surf_pos[2]};

    // Fg
    const Vector3D gravity_acceleration = asteroid_.GravityAccelerationAtPosition(position);

    // w, w'
    const boost::tuple<Vector3D, Vector3D> result = asteroid_.AngularVelocityAndAccelerationAtTime(time);
    const Vector3D angular_velocity = boost::get<0>(result);
    const Vector3D angular_acceleration = boost::get<1>(result);

    // Fc
    Vector3D thrust_acceleration;
    if (time - latest_control_input_time_ >= min_control_interval_) {
        latest_control_input_time_ = time;
        const SensorData sensor_data = sensor_simulator_.Simulate(state, height, perturbations_acceleration_, time);
        thrust_ = controller_.GetThrustForSensorData(sensor_data);
        for (unsigned int i = 0; i < 3; ++i) {
            thrust_acceleration[i] = thrust_[i] * coef_mass;
        }
    } else {
        for (unsigned int i = 0; i < 3; ++i) {
            thrust_acceleration[i] = thrust_[i] * coef_mass;
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

    d_state_dt[6] = -sqrt(thrust_[0] * thrust_[0] + thrust_[1] * thrust_[1] + thrust_[2] * thrust_[2]) / ((spacecraft_specific_impulse_ + spacecraft_specific_impulse_ * sample_factory_.SampleNormal(time, 0.0, engine_noise_)) * kEarthAcceleration);
}
