#include "adaptivesimulation.h"
#include "fixedsimulation.h"
#include "filewriter.h"

#define FULL_STATE_CONTROLLED                                   0

#define GENERATE_SENSOR_DATA_FILES                              0
#define NUM_SENSOR_DATA_FILES                                   5
#define PATH_TO_SENSOR_DATA_FOLDER                              "../../../data/"

#define CREATE_RANDOM_VISUALIZATION_FILE                        1
#define PATH_TO_RANDOM_VISUALIZATION_FILE                          "../../../results/visualization.txt"

int main(int argc, char *argv[]) {
    srand(time(0));

    /*FixedSimulation f_sim( rand());
    const boost::tuple<std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D> > f_result = f_sim.Evaluate();
    const std::vector<Vector3D> &f_positions = boost::get<1>(f_result);
    const std::vector<Vector3D> &f_heights = boost::get<2>(f_result);

    FileWriter writer;
    writer.CreateVisualizationFile(PATH_TO_RANDOM_VISUALIZATION_FILE, 1.0 / f_sim.FixedStepSize(), f_sim.AsteroidOfSystem(), f_positions, f_heights);
    exit(0);
    */

    double error = 0.0;
    const unsigned int num_tests = 1000;
    for (unsigned int i = 0; i < num_tests; ++i) {
        const unsigned int random_seed = rand();
        std::cout << "test " << (i + 1) << ", current seed: " << random_seed << std::endl;

        AdaptiveSimulation a_sim(random_seed);
        FixedSimulation f_sim(random_seed);

        std::cout << "running adaptive ... " << std::endl;
        clock_t begin = clock();
        const boost::tuple<std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D> > a_result = a_sim.Evaluate();
        clock_t end = clock();
        double simulated_time = boost::get<0>(a_result).back();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        std::cout << simulated_time << " seconds simulation time took " << elapsed_secs << " real time to compute (x" << simulated_time/elapsed_secs << ")." << std::endl;

        const std::vector<Vector3D> &a_positions = boost::get<1>(a_result);
        const Vector3D a_pos = a_positions.back();


        std::cout << "running fixed ... " << std::endl;
        begin = clock();
        const boost::tuple<std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D> > f_result = f_sim.Evaluate();
        end = clock();
        simulated_time = boost::get<0>(f_result).back();
        elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        std::cout << simulated_time << " seconds simulation time took " << elapsed_secs << " real time to compute (x" << simulated_time/elapsed_secs << ")." << std::endl;

        const std::vector<Vector3D> &f_positions = boost::get<1>(f_result);
        const std::vector<Vector3D> &f_heights = boost::get<2>(f_result);
        const Vector3D f_pos = f_positions.back();

        error += VectorNorm(VectorSub(a_pos, f_pos));
    }
    std::cout << "error: " << error / (double) num_tests << std::endl;
    std::cout << "done" << std::endl;

    /*UnitTestRunThrough();
    return 0;

    if (CREATE_RANDOM_VISUALIZATION_FILE) {
        HoveringProblem problem;
        std::cout << "creating visualization file ... " << std::endl;
        problem.Init(random_seed, FULL_STATE_CONTROLLED);
        problem.CreateVisualizationFile(PATH_TO_RANDOM_TRAJECTORY_FILE);
        std::cout << "done." << std::endl;

    }
    return 0;

    /*const double time = SIMULATION_TIME;
    const double control_frequency = 10.0;

    if (GENERATE_SENSOR_DATA_FILES) {
        SensorDataGenerator generator(PATH_TO_SENSOR_DATA_FOLDER, control_frequency, time / 6.0);
        generator.Generate(NUM_SENSOR_DATA_FILES,false);
        generator.Generate(NUM_SENSOR_DATA_FILES,true);
        return 0;
    }

    const Vector3D spacecraft_position = SamplePointOutSideEllipsoid(semi_axis, 4.0);
    const Vector3D angular_velocity = boost::get<0>(asteroid.AngularVelocityAndAccelerationAtTime(0.0));
    Vector3D spacecraft_velocity = CrossProduct(angular_velocity, spacecraft_position);
    spacecraft_velocity[0] *= -1; spacecraft_velocity[1] *= -1; spacecraft_velocity[2] *= -1;
    const double spacecraft_specific_impulse = 200.0;
    const double spacecraft_mass = 1000.0;
    const double spacecraft_maximum_thrust = 21.0;

    Vector3D target_position;
    for (unsigned int i = 0; i < 3; ++i) {
        target_position[i] = SampleUniform(spacecraft_position[i] - 3.0, spacecraft_position[i] + 3.0);
    }

    SensorSimulatorAcceleration::SensorNoiseConfiguration sensor_noise;
    for (unsigned int i = 0; i < sensor_noise.size(); ++i) {
        sensor_noise[i] = 0.05;
    }

    const double perturbation_noise = 1e-7;
    const double control_noise = 0.05;

    SensorSimulator *sensor_simulator = new SensorSimulatorAcceleration(asteroid, sensor_noise);
    Controller *spacecraft_controller = NULL; //new ControllerAcceleration(control_frequency, spacecraft_maximum_thrust, target_position, spacecraft_position, spacecraft_velocity);

    ControllerFullState *full_state_controller = NULL;
    if (FULL_STATE_CONTROLLED) {
        full_state_controller = new ControllerFullState(control_frequency, spacecraft_maximum_thrust, target_position);
    }

    Simulator simulator(control_frequency, perturbation_noise, control_noise, asteroid, sensor_simulator, spacecraft_controller, full_state_controller);
    simulator.InitSpacecraft(spacecraft_position, spacecraft_velocity, spacecraft_mass, spacecraft_specific_impulse);

    std::cout << "running simulation ..." << std::endl;
    const clock_t begin = clock();
    const boost::tuple<double, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<SensorData> > result = simulator.Run(time, false);
    const clock_t end = clock();
    const double simulated_time = boost::get<0>(result);
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << simulated_time << " seconds simulation time took " << elapsed_secs << " real time to compute (x" << simulated_time/elapsed_secs << ")." << std::endl;

    if(GENERATE_RANDOM_TRAJECTORY_FILE) {
        std::cout << "writing states to file ... ";
        FileWriter writer;
        const std::vector<Vector3D> positions = boost::get<1>(result);
        const std::vector<Vector3D> heights = boost::get<2>(result);
        writer.CreateVisualizationFile(PATH_TO_RANDOM_TRAJECTORY_FILE, control_frequency, asteroid, positions, heights);
        std::cout << "done." << std::endl;
    }
    */
}

