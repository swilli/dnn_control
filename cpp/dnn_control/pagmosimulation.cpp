#include "pagmosimulation.h"
#include "samplefactory.h"
#include "odeint.h"
#include "modifiedcontrolledrungekutta.h"
#include "odesystem.h"
#include "trainingdatagenerator.h"
#include "constants.h"
#include "configuration.h"

PaGMOSimulation::PaGMOSimulation(const unsigned int &random_seed)
    : random_seed_(random_seed), simulation_time_(0.0) {
    Init();
    simulation_time_ = asteroid_.RotationalPeriod() * 0.5;
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

unsigned int PaGMOSimulation::RandomSeed() const {
    return random_seed_;
}

SystemState PaGMOSimulation::InitialSystemState() const {
    return initial_system_state_;
}

double PaGMOSimulation::SimulationTime() const {
    return simulation_time_;
}

boost::tuple<std::vector<std::vector<double> >, std::vector<std::vector<double> >, std::vector<Vector3D>, std::vector<Vector3D> > PaGMOSimulation::GenerateSensorDataSet() {
    typedef odeint::runge_kutta_cash_karp54<SystemState> ErrorStepper;
    typedef odeint::modified_controlled_runge_kutta<ErrorStepper> ControlledStepper;

    SampleFactory sample_factory(random_seed_);
    SampleFactory sf_sensor_simulator(sample_factory.SampleRandomInteger());

    TrainingDataGenerator training_data_generator(sf_sensor_simulator, asteroid_);

    const unsigned int num_iterations = simulation_time_ * control_frequency_;

    std::vector<std::vector<double> > evaluated_sensor_values(num_iterations + 1);
    std::vector<std::vector<double> > evaluated_labels(num_iterations + 1);
    std::vector<Vector3D> evaluated_positions(num_iterations + 1);
    std::vector<Vector3D> evaluated_heights(num_iterations + 1);

    SystemState system_state(initial_system_state_);

    Vector3D perturbations_acceleration;
    const Vector3D thrust = {0.0, 0.0, 0.0};

    double current_time = 0.0;
    double current_time_observer = 0.0;
    const double dt = 1.0 / control_frequency_;
    Observer observer(current_time_observer);
    unsigned int iteration = 0;
    bool exception_thrown = false;
    try {
        for (iteration = 0; iteration < num_iterations; ++iteration) {
            const Vector3D &position = {system_state[0], system_state[1], system_state[2]};

            const Vector3D surf_pos = boost::get<0>(asteroid_.NearestPointOnSurfaceToPosition(position));
            const Vector3D &height = {position[0] - surf_pos[0], position[1] - surf_pos[1], position[2] - surf_pos[2]};

            for (unsigned int i = 0; i < 3; ++i) {
                perturbations_acceleration[i] = sample_factory.SampleNormal(perturbation_mean_, perturbation_noise_);
            }

            boost::tuple<std::vector<double>, std::vector<double> > result = training_data_generator.Generate(system_state, height, perturbations_acceleration, current_time);
            evaluated_sensor_values.at(iteration) = boost::get<0>(result);
            evaluated_labels.at(iteration) = boost::get<1>(result);
            evaluated_positions.at(iteration) = position;
            evaluated_heights.at(iteration) = height;

            const double engine_noise = sample_factory.SampleNormal(0.0, spacecraft_engine_noise_);

            ODESystem ode_system(asteroid_, perturbations_acceleration, thrust, spacecraft_specific_impulse_, spacecraft_minimum_mass_, engine_noise);

            ControlledStepper controlled_stepper;
            integrate_adaptive(controlled_stepper, ode_system, system_state, current_time, current_time + dt, minimum_step_size_, observer);

            current_time += dt;
        }
    } catch (const Asteroid::Exception &exception) {
        std::cout << "The spacecraft crashed into the asteroid's surface." << std::endl;
        exception_thrown = true;
    } catch (const ODESystem::Exception &exception) {
        std::cout << "The spacecraft is out of fuel." << std::endl;
        exception_thrown = true;
    }
    if (exception_thrown) {
        const unsigned int new_size = iteration + 2;
        evaluated_sensor_values.resize(new_size);
        evaluated_labels.resize(new_size);
        evaluated_positions.resize(new_size);
        evaluated_heights.resize(new_size);
    }
    const Vector3D &position = {system_state[0], system_state[1], system_state[2]};

    const Vector3D surf_pos = boost::get<0>(asteroid_.NearestPointOnSurfaceToPosition(position));
    const Vector3D &height = {position[0] - surf_pos[0], position[1] - surf_pos[1], position[2] - surf_pos[2]};

    boost::tuple<std::vector<double>, std::vector<double> > result = training_data_generator.Generate(system_state, height, perturbations_acceleration, current_time_observer);
    evaluated_sensor_values.back() = boost::get<0>(result);
    evaluated_labels.back() = boost::get<1>(result);
    evaluated_positions.back() = position;
    evaluated_heights.back() = height;

    return boost::make_tuple(evaluated_sensor_values, evaluated_labels, evaluated_positions, evaluated_heights);
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
    const double angular_velocity = 0.85 * sqrt((kGravitationalConstant * 4.0/3.0 * kPi * semi_axis[0] * semi_axis[1] * semi_axis[2] * density) / (semi_axis[0] * semi_axis[0] * semi_axis[0]));
    const Vector2D angular_velocity_xz = {sample_factory.SampleSign() * sample_factory.SampleUniform(angular_velocity * 0.5, angular_velocity), sample_factory.SampleSign() * sample_factory.SampleUniform(angular_velocity * 0.5, angular_velocity)};
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
    if (choices.size() == 0) {
        std::cout << "WOW... WTF... " << random_seed_ << std::endl;
        exit(1);
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
