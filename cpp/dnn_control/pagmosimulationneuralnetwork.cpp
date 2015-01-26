#include "pagmosimulationneuralnetwork.h"
#include "utility.h"
#include "datacollector.h"
#include "odeint.h"
#include "modifiedcontrolledrungekutta.h"
#include "samplefactory.h"
#include "sensorsimulatorneuralnetwork.h"
#include "controllerneuralnetwork.h"
#include "odesystem.h"

PaGMOSimulationNeuralNetwork::PaGMOSimulationNeuralNetwork(const unsigned int &random_seed, const double &simulation_time)
    : random_seed_(random_seed),  simulation_time_(simulation_time) {
    hidden_nodes_ = 10;
    Init();
}

PaGMOSimulationNeuralNetwork::PaGMOSimulationNeuralNetwork(const unsigned int &random_seed,  const double &simulation_time, const unsigned int &hidden_nodes)
    : random_seed_(random_seed), hidden_nodes_(hidden_nodes), simulation_time_(simulation_time) {
    Init();
}

PaGMOSimulationNeuralNetwork::PaGMOSimulationNeuralNetwork(const unsigned int &random_seed, const double &simulation_time, const std::vector<double> &neural_network_weights)
    : random_seed_(random_seed),  simulation_time_(simulation_time) {
    hidden_nodes_ = 10;
    neural_network_weights_ = neural_network_weights;
    Init();
}

PaGMOSimulationNeuralNetwork::PaGMOSimulationNeuralNetwork(const unsigned int &random_seed, const double &simulation_time, const std::vector<double> &neural_network_weights, const unsigned int &hidden_nodes)
    : random_seed_(random_seed),  hidden_nodes_(hidden_nodes), simulation_time_(simulation_time) {
    neural_network_weights_ = neural_network_weights;
    Init();
}

PaGMOSimulationNeuralNetwork::~PaGMOSimulationNeuralNetwork() {

}

boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > PaGMOSimulationNeuralNetwork::Evaluate() {
    SampleFactory sample_factory(random_seed_);
    SampleFactory sf_sensor_simulator(sample_factory.SampleRandomInteger());
    SampleFactory sf_odesystem(sample_factory.SampleRandomInteger());

    SensorSimulatorNeuralNetwork sensor_simulator(sf_sensor_simulator, asteroid_, target_position_);
    ControllerNeuralNetwork controller(spacecraft_maximum_thrust_, hidden_nodes_);
    if (neural_network_weights_.size()) {
        controller.SetWeights(neural_network_weights_);
    }

    if (sensor_simulator.Dimensions() != controller.Dimensions()) {
        throw SizeMismatchException();
    }

    std::vector<double> time_points;
    std::vector<double> evaluated_masses;
    std::vector<Vector3D> evaluated_positions;
    std::vector<Vector3D> evaluated_velocities;
    std::vector<Vector3D> evaluated_heights;

    DataCollector collector(asteroid_, time_points, evaluated_masses, evaluated_positions, evaluated_heights, evaluated_velocities);
    SystemState system_state(initial_system_state_);

    typedef odeint::runge_kutta_cash_karp54<SystemState> ErrorStepper;
    typedef odeint::modified_controlled_runge_kutta<ErrorStepper> ControlledStepper;
    ControlledStepper controlled_stepper;

    ODESystem sys(sf_odesystem, asteroid_, sensor_simulator, controller, spacecraft_specific_impulse_, perturbation_noise_, engine_noise_);

    try {
        integrate_adaptive(controlled_stepper , sys, system_state, 0.0, simulation_time_, minimum_step_size_, collector);
    } catch (const Asteroid::Exception &exception) {
        //std::cout << "The spacecraft crashed into the asteroid's surface." << std::endl;
    } catch (const ODESystem::Exception &exception) {
        //std::cout << "The spacecraft is out of fuel." << std::endl;
   }

    return boost::make_tuple(time_points, evaluated_masses, evaluated_positions, evaluated_heights, evaluated_velocities);
}

boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > PaGMOSimulationNeuralNetwork::EvaluateDetailed() {
    SampleFactory sample_factory(random_seed_);
    SampleFactory sf_sensor_simulator(sample_factory.SampleRandomInteger());
    SampleFactory sf_odesystem(sample_factory.SampleRandomInteger());

    SensorSimulatorNeuralNetwork sensor_simulator(sf_sensor_simulator, asteroid_, target_position_);
    ControllerNeuralNetwork controller(spacecraft_maximum_thrust_, hidden_nodes_);
    if (neural_network_weights_.size()) {
        controller.SetWeights(neural_network_weights_);
    }

    if (sensor_simulator.Dimensions() != controller.Dimensions()) {
        throw SizeMismatchException();
    }

    std::vector<double> time_points;
    std::vector<double> evaluated_masses;
    std::vector<Vector3D> evaluated_positions;
    std::vector<Vector3D> evaluated_velocities;
    std::vector<Vector3D> evaluated_heights;

    DataCollector collector(asteroid_, time_points, evaluated_masses, evaluated_positions, evaluated_heights, evaluated_velocities);
    SystemState system_state(initial_system_state_);

    ODESystem sys(sf_odesystem, asteroid_, sensor_simulator, controller, spacecraft_specific_impulse_, perturbation_noise_, engine_noise_);

    odeint::runge_kutta4<SystemState> stepper;
    try {
        integrate_const(stepper , sys, system_state, 0.0, simulation_time_, fixed_step_size_, collector);
    } catch (const Asteroid::Exception &exception) {
        //std::cout << "The spacecraft crashed into the asteroid's surface." << std::endl;
    } catch (const ODESystem::Exception &exception) {
        //std::cout << "The spacecraft is out of fuel." << std::endl;
    }

    return boost::make_tuple(time_points, evaluated_masses, evaluated_positions, evaluated_heights, evaluated_velocities);
}

double PaGMOSimulationNeuralNetwork::FixedStepSize() const {
    return fixed_step_size_;
}

double PaGMOSimulationNeuralNetwork::MinimumStepSize() const {
    return minimum_step_size_;
}

unsigned int PaGMOSimulationNeuralNetwork::ControllerNeuralNetworkSize() const {
    return ControllerNeuralNetwork(spacecraft_maximum_thrust_, hidden_nodes_).NeuralNetworkSize();
}

Asteroid& PaGMOSimulationNeuralNetwork::AsteroidOfSystem() {
    return asteroid_;
}

void PaGMOSimulationNeuralNetwork::Init() {
    minimum_step_size_ = 0.1;
    fixed_step_size_ = 0.1;

    const unsigned int fixed_seed = 123;
    SampleFactory sample_factory(fixed_seed);

    const Vector3D semi_axis = {sample_factory.SampleUniform(8000.0, 12000.0), sample_factory.SampleUniform(4000.0, 7500.0), sample_factory.SampleUniform(1000.0, 3500.0)};
    const double density = sample_factory.SampleUniform(1500.0, 3000.0);
    Vector2D angular_velocity_xz = {sample_factory.SampleSign() * sample_factory.SampleUniform(0.0002, 0.0008), sample_factory.SampleSign() * sample_factory.SampleUniform(0.0002, 0.0008)};
    const double time_bias = sample_factory.SampleUniform(0.0, 12.0 * 60 * 60);
    asteroid_ = Asteroid(semi_axis, density, angular_velocity_xz, time_bias);

    const double spacecraft_mass = sample_factory.SampleUniform(450.0, 500.0);
    spacecraft_maximum_thrust_ = 21.0;
    spacecraft_specific_impulse_ = 200.0;


    // orbit
    //const Vector3D spacecraft_position = {sample_factory.SampleUniform(4.0, 6.0) * semi_axis[0], 0.0, 0.0};

    // random
    const Vector3D spacecraft_position = sample_factory.SamplePointOutSideEllipsoid(semi_axis, 1.1, 4.0);

    target_position_ = spacecraft_position;

    const Vector3D angular_velocity = boost::get<0>(asteroid_.AngularVelocityAndAccelerationAtTime(0.0));
    Vector3D spacecraft_velocity = VectorCrossProduct(angular_velocity, spacecraft_position);

    // orbit
    //const double norm_position = VectorNorm(spacecraft_position);
    //spacecraft_velocity[0] = -spacecraft_velocity[0]; spacecraft_velocity[1] = -spacecraft_velocity[1] + sqrt(asteroid_.MassGravitationalConstant() / norm_position); spacecraft_velocity[2] = -spacecraft_velocity[2];

    // no velocity
    spacecraft_velocity[0] *= -1; spacecraft_velocity[1] *= -1; spacecraft_velocity[2] *= -1;

    perturbation_noise_ = 1e-7;
    engine_noise_ = 0.05;

    for (unsigned int i = 0; i < 3; ++i) {
        initial_system_state_[i] = spacecraft_position[i];
        initial_system_state_[3+i] = spacecraft_velocity[i];
    }
    initial_system_state_[6] = spacecraft_mass;
}
