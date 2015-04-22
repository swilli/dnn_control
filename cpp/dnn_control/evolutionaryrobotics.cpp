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

    srand(time(0));

    std::cout << std::setprecision(10);

    // We instantiate a PSO algorithm capable of coping with stochastic prolems
    pagmo::algorithm::pso_generational algo(1,0.7298,2.05,2.05,0.05);

    std::cout << "Initializing NN controller evolution ....";

    pagmo::archipelago archi = pagmo::archipelago(pagmo::topology::fully_connected());

    for (unsigned int j = 0;j < kNumIslands; ++j) {
        std::cout << " [" << j;
        fflush(stdout);
        pagmo::problem::hovering_problem_neural_network prob(rand(), kNumEvaluations, kSimulationTime, kNumHiddenNeurons);

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

    const unsigned int worst_case_seed = random_seed;

    const pagmo::decision_vector &solution = {-0.5519161942, 1.482310409, -0.5553234993, 5.860997513, -3.319229915, -2.705639197, 1.233132803, -1.226870093, -0.4318064625, 0.572722332, -4.220140028, -3.041953377, 1.242803613, -1.981815885, -3.044787807, 0.9730719303, -0.2928538063, 0.1196750983, 0.9552693459, 2.721820847, 0.3496558269, -1.942386561, 0.51770142, -3.753890724, 1.395752842, 1.515788584, -0.9577614273, -1.576397533, 1.405982091, -0.3787527549, 0.3603703363, -0.1749495327, 1.114310208, 0.1492604559, -2.048066788, -2.207784332, -0.4330978115, -3.036395678, -0.261054318, -0.08319209848, 2.20476404, 2.70853875, -1.510543576, -0.9336680232, -0.8555392033, 1.164857232, -0.4305392135, 1.196865835, 0.7605511167, 2.494452164, 2.212430658, -0.1517483275, -2.32037249, 1.86055639, 2.965200952, -0.5850148932, 1.037077116, -0.2976088803, -1.230729609, -0.5064036464, -1.823245909, -0.3958055462, -0.171909577, 1.209526875, 0.1226990167, 0.6182103361, -5.328462911, 0.4025540354, 1.892097023, -6.232201969, 1.868803384, -0.6774073812, 1.552509167, -1.346189826, -2.057285741, 3.41403338, 1.474427614, -2.818096494, -3.900769969, 4.900660307, -0.0613155739, 0.6434781547, -0.574412067, -0.05884536953, -3.324015104, 1.236654014, -0.7978557866, 3.44851521, 1.979399431, -0.3798207812, 0.4627470013, 0.1609883211, -0.4795866935, 0.189498394, -0.2935490475, 0.3601395287, -0.08220212112, -0.5797671538, -0.9206348157, -0.2603653809, 1.393066943, -0.280084909, -2.219072592, -2.37650468, -1.29282975, -0.4366180434, 0.5653748768, 2.986652115, -0.1359079699, -0.0719217029, 1.397887123, -0.07881457674, -0.7769461672};

    std::cout << std::setprecision(10);

    pagmo::problem::hovering_problem_neural_network prob(random_seed, kNumEvaluations, kSimulationTime, kNumHiddenNeurons);

    std::cout << "Checking NN controller fitness... ";
    const double fitness = prob.objfun_seeded(random_seed, solution)[0];
    std::cout << fitness << std::endl;

    std::cout << "Simulating NN controller ... ";
    PaGMOSimulationNeuralNetwork simulation(worst_case_seed, kNumHiddenNeurons, solution);
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
    const boost::tuple<std::vector<unsigned int>, std::vector<double>, std::vector<std::pair<double, double> > > post_evaluation = prob.post_evaluate(solution, random_seed);
    const std::vector<unsigned int> &random_seeds = boost::get<0>(post_evaluation);
    const std::vector<double> &mean_errors = boost::get<1>(post_evaluation);
    const std::vector<std::pair<double, double> > &min_max_errors = boost::get<2>(post_evaluation);

    std::cout << "done." << std::endl << "Writing post evaluation file ... ";
    FileWriter writer_post_evaluation(PATH_TO_NEURO_POST_EVALUATION_FILE);
    writer_post_evaluation.CreatePostEvaluationFile(random_seeds, mean_errors, min_max_errors);
    std::cout << "done." << std::endl;
}
