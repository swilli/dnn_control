#include "evolutionaryrobotics.h"
#include "hoveringproblemneuralnetwork.h"
#include "samplefactory.h"
#include "filewriter.h"
#include "configuration.h"
#include <sstream>

// Training configuration
static const unsigned int kNumGenerations = ER_NUM_GENERATIONS;
static const unsigned int kPopulationSize = ER_POPULATION_SIZE;
static const unsigned int kNumIslands = ER_NUM_ISLANDS;
#ifdef ER_SIMULATION_TIME
static const double kSimulationTime = ER_SIMULATION_TIME;
#else
static const double kSimulationTime = 0.0;
#endif
static const unsigned int kNumEvaluations = ER_EVALUATIONS;
static const unsigned int kNumHiddenNeurons = ER_NUM_HIDDEN_NODES;
static const unsigned int kEarlyStoppingTestInterval = 10;
static const unsigned int kNumEarlyStoppingTests = 100;
static const unsigned int kNumEarlyStoppingDelay = 10;
static const bool kEnableSensorNoise = ER_ENABLE_SENSOR_NOISE;

static std::set<SensorSimulator::SensorType> kSensorTypes;

static void Init() {
#if ER_ENABLE_RELATIVE_POSITION
    kSensorTypes.insert(SensorSimulator::SensorType::RelativePosition);
#endif
#if ER_ENABLE_VELOCITY
    kSensorTypes.insert(SensorSimulator::SensorType::Velocity);
#endif
#if ER_ENABLE_OPTICAL_FLOW
    kSensorTypes.insert(SensorSimulator::SensorType::OpticalFlow);
#endif
#if ER_ENABLE_ACCELEROMETER
    kSensorTypes.insert(SensorSimulator::SensorType::ExternalAcceleration);
#endif
}

static unsigned int ArchipelagoChampionID(pagmo::archipelago archi) {
    double min = archi.get_island(0)->get_population().champion().f[0];
    unsigned int idx = 0;
    for (unsigned int i = 1; i < archi.get_size(); ++i) {
        double cur = archi.get_island(i)->get_population().champion().f[0];
        if (cur < min) {
            min = cur;
            idx = i;
        }
    }
    return idx;
}

static void ArchipelagoEvolve(pagmo::archipelago &archi, const pagmo::problem::hovering_problem_neural_network &prob, const unsigned int &num_generations, const unsigned int &early_stopping_test_interval, const unsigned int &num_early_stopping_tests, const unsigned int &early_stopping_delay) {
    // Buffer
    std::vector<double> buff;

    //Evolution is here started on the archipelago

    unsigned int generations = 0;
    double avg_error = std::numeric_limits<double>::max();
    unsigned int worse = 0;
    pagmo::decision_vector champion;
    for (unsigned int i = 0; i< num_generations; ++i){
        const unsigned int idx = ArchipelagoChampionID(archi);
        double best_f = archi.get_island(idx)->get_population().champion().f[0];

        if (i<50) {
            buff.push_back(best_f);
        }
        else {
            (buff[i%50] = best_f);
        }
        double mean = 0.0;
        mean = std::accumulate(buff.begin(),buff.end(),mean);
        mean /= (double)buff.size();

        time_t rawtime;
        struct tm *timeinfo;
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        std::cout << std::endl << asctime(timeinfo) << "generation: "<< std::setw(20) << i << std::setw(20) <<
                     best_f << std::setw(20) <<
                     archi.get_island(idx)->get_population().mean_velocity() << std::setw(20) <<
                     mean <<	 std::endl << "[";
        const pagmo::decision_vector x = archi.get_island(idx)->get_population().champion().x;
        for (unsigned int i = 0; i < x.size() -1; ++i) {
            std::cout << x[i] << ", ";
        }
        std::cout << x.back() << "]" << std::endl;
        fflush(stdout);
        archi.evolve(1);

        generations++;
        if (generations == early_stopping_test_interval) {
            generations = 0;
            std::cout << std::endl << ">> early stopping test ... ";
            fflush(stdout);
            std::vector<unsigned int> random_seeds;
            for (unsigned int j = 0; j < num_early_stopping_tests; ++j) {
                random_seeds.push_back(rand());
            }

            double cur_avg_error = 0.0;
            for (unsigned int j = 0; j < num_early_stopping_tests; ++j) {
                cur_avg_error += prob.objfun_seeded(random_seeds.at(j), x)[0];
            }
            cur_avg_error /= num_early_stopping_tests;
            std::cout << "average error: " << cur_avg_error << "/" << avg_error << " patience: ";
            fflush(stdout);
            if (cur_avg_error > avg_error) {
                worse++;
            } else {
                avg_error = cur_avg_error;
                worse = 0;
                champion = x;
            }

            std::cout << worse << "/" << early_stopping_delay << std::endl;
            if (worse == early_stopping_delay) {
                break;
            }
        }
    }

    std::cout << std::endl << "And the winner is ......" << std::endl << "[";
    for (unsigned int i = 0; i < champion.size() -1; ++i) {
        std::cout << champion[i] << ", ";
    }
    std::cout << champion.back() << "]" << std::endl;


    std::cout << std::endl << "Copy this into C++ code:" << std::endl << "{";
    for (unsigned int i = 0; i < champion.size() -1; ++i) {
        std::cout << champion[i] << ", ";
    }
    std::cout << champion.back() << "};" << std::endl;
}

void TrainNeuralNetworkController() {
    ConfigurationPaGMO();
    Init();

    srand(time(0));

    std::cout << std::setprecision(10);

    // We instantiate a PSO algorithm capable of coping with stochastic prolems
    pagmo::algorithm::pso_generational algo(1,0.7298,2.05,2.05,0.05);

    std::cout << "Initializing NN controller evolution ....";

    pagmo::archipelago archi = pagmo::archipelago(pagmo::topology::fully_connected());

    for (unsigned int j = 0;j < kNumIslands; ++j) {
        std::cout << " [" << j;
        fflush(stdout);
        pagmo::problem::hovering_problem_neural_network prob(rand(), kNumEvaluations, kSimulationTime, kNumHiddenNeurons, kSensorTypes, kEnableSensorNoise);

        // This instantiates a population within the original bounds (-1,1)
        pagmo::population pop_temp(prob, kPopulationSize);

        // We make the bounds larger to allow neurons weights to grow
        prob.set_bounds(-30.0,30.0);

        // We create an empty population on the new prolem (-10,10)
        pagmo::population pop(prob);

        // And we fill it up with (-1,1) individuals having zero velocities
        pagmo::decision_vector v(prob.get_dimension(),0);
        for (unsigned int i = 0; i < kPopulationSize; ++i) {
            pop.push_back(pop_temp.get_individual(i).cur_x);
            pop.set_v(i,v);
        }
        archi.push_back(pagmo::island(algo,pop));
        std::cout << "]";
        fflush(stdout);
    }

    std::cout << " done" << std::endl << "Evolving ..." << std::endl;
    fflush(stdout);

    pagmo::problem::hovering_problem_neural_network prob(rand(), kNumEvaluations, kSimulationTime, kNumHiddenNeurons);
    ArchipelagoEvolve(archi, prob, kNumGenerations, kEarlyStoppingTestInterval, kNumEarlyStoppingTests, kNumEarlyStoppingDelay);
}

static void ConvexityCheck(pagmo::problem::hovering_problem_neural_network &problem, const unsigned &random_seed, const pagmo::decision_vector &x) {
    const double d_range = 0.01;
    const double range = 5.0;

    std::cout << "Checking NN controller convexity... " << std::endl;

    for (unsigned int dimension = 0; dimension < x.size(); ++dimension) {
        std::vector<std::pair<double,double> > fitness;
        pagmo::decision_vector x_copy(x);
        double weight = -range;
        while (weight <= range) {
            std::cout << weight << std::endl;
            x_copy.at(dimension) = weight;
            fitness.push_back(std::make_pair(weight, problem.objfun_seeded(random_seed, x_copy)[0]));
            weight += d_range;
        }
        std::cout << "Writing convexity file ... ";
        std::string path(PATH_TO_NEURO_CONVEXITY_PATH);
        std::stringstream ss;
        ss << "dim_" << dimension << ".txt";
        path += ss.str();
        FileWriter writer(path);
        writer.CreateConvexityFile(random_seed, dimension, fitness);
        std::cout << "done." << std::endl;
    }

    std::cout << "done." << std::endl;
}

void TestNeuralNetworkController(const unsigned int &random_seed) {
    ConfigurationPaGMO();
    Init();

    const unsigned int worst_case_seed = random_seed;

    const pagmo::decision_vector &solution = {1.001103789, -8.161331234, 0.3520941629, 21.31224108, 2.993190596, 1.540083983, -4.960443718, 0.07848678025, 21.50888414, -1.016576372, 2.527411128, -10.16261973, -2.73463824, 0.5018890191, -0.9125227149, -0.5766106474, -2.635801162, -6.731137006, 5.495700868, 4.043348968, 12.0606057, 0.0373994039, -0.9530464447, 10.87969989, 8.12263668, 6.525536847, -8.688256578, -3.232172807, -0.8889480546, 0.9646354963, -13.9637541, 0.5056783048, 5.792543577, 16.97313262, -3.077920321, 0.1209991542, -1.021725346, 3.075370631, 0.5546551459, 6.795723845, -4.87550421, -3.542912142, 0.4711377657, -0.6430664077, 4.793894827, -3.242255107, -0.8484921515, -1.070496677, -1.592554738, 0.1722556504, -0.8767766873, 0.2031663292, -1.962621847, 1.909640343, -2.218611753, 1.124499377, -0.9393535706, 1.038633255, 0.7051762329, -3.698254931, 0.6465689793, 0.8694362508, 0.5576020991};

    std::cout << std::setprecision(10);

    pagmo::problem::hovering_problem_neural_network prob(random_seed, kNumEvaluations, kSimulationTime, kNumHiddenNeurons, kSensorTypes, kEnableSensorNoise);

    std::cout << "Checking NN controller fitness... ";
    const double fitness = prob.objfun_seeded(random_seed, solution)[0];
    std::cout << fitness << std::endl;

    std::cout << "Simulating NN controller ... ";
    PaGMOSimulationNeuralNetwork simulation(worst_case_seed, kNumHiddenNeurons, solution, kSensorTypes, kEnableSensorNoise);
    simulation.SetSimulationTime(2.0 * 86400.0);
    const boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<std::vector<double> > > result = simulation.EvaluateAdaptive();
    const std::vector<double> &times = boost::get<0>(result);
    const std::vector<Vector3D> &positions = boost::get<2>(result);
    const std::vector<Vector3D> &heights = boost::get<3>(result);
    const std::vector<Vector3D> &velocities = boost::get<4>(result);
    const std::vector<Vector3D> &thrusts = boost::get<5>(result);
    std::cout << "done." << std::endl;

    std::cout << "Writing visualization file ... ";
    FileWriter writer_visualization(PATH_TO_NEURO_TRAJECTORY_FILE);
    writer_visualization.CreateTrajectoryFile(simulation.ControlFrequency(), simulation.AsteroidOfSystem(), positions, heights);
    std::cout << "done." << std::endl;

    std::cout << "Writing evaluation file ... ";
    FileWriter writer_evaluation(PATH_TO_NEURO_EVALUATION_FILE);
    writer_evaluation.CreateEvaluationFile(random_seed, simulation.TargetPosition(), simulation.AsteroidOfSystem(), times, positions, velocities, thrusts);
    std::cout << "done." << std::endl;

    std::cout << "Performing post evaluation ... ";
    const boost::tuple<std::vector<unsigned int>, std::vector<double>, std::vector<std::pair<double, double> >, std::vector<std::pair<double, double> > > post_evaluation = prob.post_evaluate(solution, random_seed);
    const std::vector<unsigned int> &random_seeds = boost::get<0>(post_evaluation);
    const std::vector<double> &mean_errors = boost::get<1>(post_evaluation);
    const std::vector<std::pair<double, double> > &min_max_errors = boost::get<2>(post_evaluation);
    const std::vector<std::pair<double, double> > &fuel_consumptions = boost::get<3>(post_evaluation);

    std::cout << "done." << std::endl << "Writing post evaluation file ... ";
    FileWriter writer_post_evaluation(PATH_TO_NEURO_POST_EVALUATION_FILE);
    writer_post_evaluation.CreatePostEvaluationFile(random_seeds, mean_errors, min_max_errors, fuel_consumptions);
    std::cout << "done." << std::endl;
}
