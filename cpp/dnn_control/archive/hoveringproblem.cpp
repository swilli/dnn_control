#include "hoveringproblem.h"
#include "samplefactory.h"
#include "vector.h"
#include "asteroid.h"
#include "controllerfullstate.h"
#include "sensorsimulatoracceleration.h"
#include "controlleracceleration.h"
#include "utility.h"
#include "filewriter.h"
#include "constants.h"

HoveringProblem::HoveringProblem() : initialized_(false) {
    simulation_time_ = 24.0 * 60.0 * 60.0;
    simulator_ = NULL;
}

HoveringProblem::~HoveringProblem() {
    if (simulator_ != NULL) {
        delete simulator_;
        simulator_ = NULL;
    }
}

void HoveringProblem::Init(const unsigned int &random_seed, const bool &full_state_controlled) {
    SampleFactory::Init(random_seed);

    const Vector3D semi_axis = {SampleFactory::SampleUniform(8000.0, 12000.0), SampleFactory::SampleUniform(4000.0, 7500.0), SampleFactory::SampleUniform(1000.0, 3500.0)};
    const double density = SampleFactory::SampleUniform(1500.0, 3000.0);
    Vector2D angular_velocity_xz = {SampleFactory::SampleSign() * SampleFactory::SampleUniform(0.0002, 0.0008), SampleFactory::SampleSign() * SampleFactory::SampleUniform(0.0002, 0.0008)};
    const double time_bias = SampleFactory::SampleUniform(0.0, 12.0 * 60 * 60);
    Asteroid asteroid(semi_axis, density, angular_velocity_xz, time_bias);

    const double spacecraft_mass = SampleFactory::SampleUniform(450.0, 500.0);
    const double spacecraft_specific_impulse = 200.0;
    const double spacecraft_maximum_thrust = 21.0;

    const Vector3D spacecraft_position = {2.0 * semi_axis[0], 0.0, 0.0}; //SamplePointOutSideEllipsoid(semi_axis, 4.0);
    double norm_position = 0.0;
    for (unsigned int i = 0; i < 3; ++i) {
        norm_position += spacecraft_position[i] * spacecraft_position[i];
    }
    norm_position = sqrt(norm_position);
    const Vector3D angular_velocity = boost::get<0>(asteroid.AngularVelocityAndAccelerationAtTime(0.0));
    Vector3D spacecraft_velocity = CrossProduct(angular_velocity, spacecraft_position);
    spacecraft_velocity[0] = -spacecraft_velocity[0];
    spacecraft_velocity[1] = -spacecraft_velocity[1] + sqrt(asteroid.MassGravitationalConstant() / norm_position);
    spacecraft_velocity[2] = -spacecraft_velocity[2];
    //spacecraft_velocity[0] *= -1; spacecraft_velocity[1] *= -1; spacecraft_velocity[2] *= -1;

    Vector3D target_position;
    for (unsigned int i = 0; i < 3; ++i) {
        target_position[i] = SampleFactory::SampleUniform(spacecraft_position[i] - 3.0, spacecraft_position[i] + 3.0);
    }

    SensorSimulatorAcceleration::SensorNoiseConfiguration sensor_noise;
    for (unsigned int i = 0; i < sensor_noise.size(); ++i) {
        sensor_noise[i] = 0.05;
    }

    const double perturbation_noise = 0.0;

    const double control_noise = 0.05;
    const double control_frequency = 10;

    SensorSimulator *sensor_simulator = new SensorSimulatorAcceleration(asteroid, sensor_noise);
    Controller *spacecraft_controller = NULL; //new ControllerAcceleration(control_frequency, spacecraft_maximum_thrust, target_position, spacecraft_position, spacecraft_velocity);

    ControllerFullState *full_state_controller = NULL;
    if (full_state_controlled) {
        full_state_controller = new ControllerFullState(control_frequency, spacecraft_maximum_thrust, target_position);
    }

    simulator_ = new Simulator(control_frequency, perturbation_noise, control_noise, asteroid, sensor_simulator, spacecraft_controller, full_state_controller);
    simulator_->InitSpacecraft(spacecraft_position, spacecraft_velocity, spacecraft_mass, spacecraft_specific_impulse);

    initialized_ = true;
}

void HoveringProblem::CreateVisualizationFile(const std::string &path_to_visualization_file) {
    if (simulator_ == NULL || !initialized_) {
        exit(EXIT_FAILURE);
    }

    const boost::tuple<double, std::vector<Vector3D>, std::vector<Vector3D> > result = simulator_->RunForVisualization(simulation_time_);

    FileWriter writer;
    const std::vector<Vector3D> positions = boost::get<1>(result);
    const std::vector<Vector3D> heights = boost::get<2>(result);
    writer.CreateVisualizationFile(path_to_visualization_file, simulator_->ControlFrequency(), simulator_->AsteroidOfSystem(), positions, heights);
}

Vector3D HoveringProblem::PositionAtEnd() {
    if (simulator_ == NULL || !initialized_) {
        exit(EXIT_FAILURE);
    }
    return boost::get<1>(simulator_->RunThrough(simulation_time_));
}

Simulator* HoveringProblem::SimulatorOfProblem() {
    return simulator_;
}

double HoveringProblem::SimulationTime() const {
    return simulation_time_;
}


