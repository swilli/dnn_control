#include "lspisimulator.h"
#include "samplefactory.h"
#include "odesystem.h"
#include "odeint.h"
#include "modifiedcontrolledrungekutta.h"
#include "constants.h"

LSPISimulator::LSPISimulator(const unsigned int &random_seed)
    : random_seed_(random_seed), sample_factory_(random_seed) {

    minimum_step_size_ = 0.1;
    control_frequency_ = 1.0;

    SampleFactory asteroid_sf(random_seed);

    const double c_semi_axis = asteroid_sf.SampleUniformReal(100.0, 8000.0);
    const double b_semi_axis_n = asteroid_sf.SampleUniformReal(1.1, 2.0);
    const double a_semi_axis_n = asteroid_sf.SampleUniformReal(1.1 * b_semi_axis_n, 4.0);
    const Vector3D &semi_axis = {a_semi_axis_n * c_semi_axis, b_semi_axis_n * c_semi_axis, c_semi_axis};
    const double density = asteroid_sf.SampleUniformReal(1500.0, 3000.0);
    const double magn_angular_velocity = 0.85 * sqrt((kGravitationalConstant * 4.0/3.0 * kPi * semi_axis[0] * semi_axis[1] * semi_axis[2] * density) / (semi_axis[0] * semi_axis[0] * semi_axis[0]));
    const Vector2D &angular_velocity_xz = {asteroid_sf.SampleSign() * asteroid_sf.SampleUniformReal(magn_angular_velocity * 0.5, magn_angular_velocity), asteroid_sf.SampleSign() * asteroid_sf.SampleUniformReal(magn_angular_velocity * 0.5, magn_angular_velocity)};
    const double time_bias = asteroid_sf.SampleUniformReal(0.0, 12.0 * 60 * 60);
    asteroid_ = Asteroid(semi_axis, density, angular_velocity_xz, time_bias);

    spacecraft_maximum_mass_ = sample_factory_.SampleUniformReal(450.0, 500.0);
    spacecraft_minimum_mass_ = spacecraft_maximum_mass_ * 0.5;
    spacecraft_maximum_thrust_ = 21.0;
    spacecraft_specific_impulse_ = 200.0;
    spacecraft_engine_noise_ = 0.05;

    perturbation_mean_ = 1e-6;
    perturbation_noise_ = 1e-7;
}

boost::tuple<SystemState, double, bool> LSPISimulator::NextState(const SystemState &state, const double &time, const Vector3D &thrust) {
    typedef odeint::runge_kutta_cash_karp54<SystemState> ErrorStepper;
    typedef odeint::modified_controlled_runge_kutta<ErrorStepper> ControlledStepper;

    SystemState state_copy(state);

    Vector3D perturbations_acceleration;
    for (unsigned int i = 0; i < 3; ++i) {
        perturbations_acceleration[i] = sample_factory_.SampleNormal(perturbation_mean_, perturbation_noise_);
    }

    const double engine_noise = sample_factory_.SampleNormal(0.0, spacecraft_engine_noise_);

    ODESystem ode_system(asteroid_, perturbations_acceleration, thrust, spacecraft_specific_impulse_, spacecraft_minimum_mass_, engine_noise);

    ControlledStepper controlled_stepper;
    double current_time_observer = 0.0;
    Observer observer(current_time_observer);
    bool exception_thrown = false;
    const double dt = 1.0 / control_frequency_;
    try {
        integrate_adaptive(controlled_stepper, ode_system, state_copy, time, time + dt, minimum_step_size_, observer);
    } catch (const Asteroid::Exception &exception) {
        //std::cout << "The spacecraft crashed into the asteroid's surface." << std::endl;
        exception_thrown = true;
    } catch (const ODESystem::Exception &exception) {
        //std::cout << "The spacecraft is out of fuel." << std::endl;
        exception_thrown = true;
    }

    return boost::make_tuple(state_copy, current_time_observer, exception_thrown);
}

Asteroid &LSPISimulator::AsteroidOfSystem() {
    return asteroid_;
}

SampleFactory &LSPISimulator::SampleFactoryOfSystem() {
    return sample_factory_;
}

double LSPISimulator::ControlFrequency() const {
    return control_frequency_;
}

double LSPISimulator::SpacecraftMaximumMass() const {
    return spacecraft_maximum_mass_;
}

