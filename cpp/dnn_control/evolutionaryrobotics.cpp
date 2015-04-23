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

    const pagmo::decision_vector &solution = {3.029506878, 5.920776096, -0.6954663801, -2.084201016, 2.901741114, -2.703666157, -0.8361321469, 4.217928173, 0.2271514851, 0.669312924, 3.430800097, -5.595167316, 1.068138745, 2.428427804, 3.153371716, -2.811535036, 1.817673226, -0.4303820009, -1.198913345, -1.090662357, -4.999863385, -3.396505621, -4.306196326, -0.4927390932, 1.704048763, 2.604055063, -0.6848533721, -1.65814225, 6.189078364, -0.3324037482, -4.422659017, 1.847192286, 7.273787622, 0.5029598237, 0.1691830311, -4.174990116, -0.4836497653, 2.630119328, -2.024932577, -4.665299388, -3.87314456, -1.920174896, -1.368259047, -1.29890367, -0.401414487, 1.700848623, 3.737086446, 0.06603159418, -0.3448020221, -2.974154104, -0.1007722235, 0.08102218147, -7.877349202, 4.065310135, 3.640264835, 5.637098866, 4.253135194, 3.588216298, -2.823381096, 2.103837222, -1.011566583, 1.146481284, -0.308559377, -0.8827949338, 5.224376016, -0.7316942109, 0.7380328444, -1.67613986, 4.139552687, -0.334528333, -0.0938551802, -0.02785121602, -3.388803266, 7.293616497, 1.431982217, -0.4003014024, 3.066233115, 4.808817249, 0.5856678579, -3.144395761, -4.638532473, -2.510970352, -2.421688762, 8.505497759, -5.687488995, 5.748679534, 4.782954794, 2.675972698, -1.89386327, -1.147524036, 2.070100368, -1.075176765, -4.141387318, 6.459400947, 3.152550381, -0.626856043, 3.480009318, -7.310873543, -0.1285314144, -2.137152259, -1.021720354, 3.04691681, -7.191341877, -0.04523143453, 1.817588148, 2.863667669, -2.643510964, -5.696053247, 1.910802364, 0.691540613, -1.296716516, 0.3474816654, -2.558885409, -6.202657995, 0.02170004213, 4.149512864, -2.367229887, -0.1978019427, -2.957177753, 1.137488425, 1.83820455, -2.742597632, -0.6963896656, 5.878953728, -3.045404634, -5.347228448, 1.651793383, 1.502283297, -3.835026293, -6.741500355, -0.1803661447, -0.6958428088, -5.6110423, -0.1754260492, 2.225832084, -0.3330844456, 0.8506149507, -1.880581888, 4.007018202, -4.514046149, 2.129995434, 9.601546111, 1.663068398, -0.5712892172, -7.223223972, -0.9862387142, -0.0697366674, -1.341183004, 0.9603821498, 5.294273675, -0.3594702676, 5.106002104, -2.000552457, 1.644907013, -2.246196339, 4.146585287, 5.418945789, -0.02931200837, 4.814033779, -0.2560448512, 1.283844649, 2.115889106, -3.971757632, 6.928459845, -7.401736882, 0.1512576105, 3.12982937, 1.535878802, -1.938551275, 1.717001092, -1.837239449, 1.043503695, 7.272505987, 2.652563518, -1.871849661, -5.074460891, 2.183274265, 1.183457037, -4.913993819, 0.09814671153, 2.041229433, 1.833492395, -0.5816691382, -2.437753273, 4.198870564, -1.288273767, -5.096786248, 3.848784306, 3.323075712, 3.76157903, -3.460765991, -4.162008862, -6.353870524, -9.63571011, 2.584822072, 6.694550299, -0.6963469366, -0.1002722554, -2.891641809, -8.424369036, 1.982158593, 2.527151758, 0.4093860327, 0.2863791534, 1.947148573, 6.321686234, -1.815570609, 0.9024711768, -2.657538932, 2.154624253, 0.4227921252, 1.539893695, -4.937716751, -0.1197216231, -0.3275662189, 5.757754714, 1.320694827, -2.995234482, -1.851742891, 3.20195685, -3.390158446, -1.002760607, 1.019160566};

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
