#include "evolutionaryrobotics.h"
#include "hoveringproblemneuralnetwork.h"
#include "hoveringproblemproportionalderivative.h"
#include "samplefactory.h"
#include "filewriter.h"
#include "configuration.h"
#include <sstream>

// Training configuration
static const unsigned int kNumGenerations = ER_NUM_GENERATIONS;
static const unsigned int kPopulationSize = ER_POPULATION_SIZE;
static const unsigned int kNumIslands = ER_NUM_ISLANDS;
static const double kSimulationTime = ER_SIMULATION_TIME;
static const unsigned int kNumEvaluations = ER_EVALUATIONS;
static const unsigned int kNumHiddenNeurons = ER_NUM_HIDDEN_NODES;



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

static void ArchipelagoEvolve(pagmo::archipelago &archi, const unsigned int &num_generations) {
    // Buffer
    std::vector<double> buff;

    //Evolution is here started on the archipelago
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
    }

    const unsigned int idx = ArchipelagoChampionID(archi);
    std::cout << std::endl << "And the winner is ......" << std::endl << "[";
    const pagmo::decision_vector x = archi.get_island(idx)->get_population().champion().x;
    for (unsigned int i = 0; i < x.size() -1; ++i) {
        std::cout << x[i] << ", ";
    }
    std::cout << x.back() << "]" << std::endl;


    std::cout << std::endl << "Copy this into C++ code:" << std::endl << "{";
    for (unsigned int i = 0; i < x.size() -1; ++i) {
        std::cout << x[i] << ", ";
    }
    std::cout << x.back() << "};" << std::endl;
}

void TrainNeuralNetworkController() {
    ConfigurationPaGMO();
    std::cout << std::setprecision(10);

    // We instantiate a PSO algorithm capable of coping with stochastic prolems
    pagmo::algorithm::pso_generational algo(1,0.7298,2.05,2.05,0.05);

    std::cout << "Initializing NN evolution ....";

    pagmo::archipelago archi = pagmo::archipelago(pagmo::topology::fully_connected());

    for (unsigned int j = 0;j < kNumIslands; ++j) {
        std::cout << " [" << j;
        fflush(stdout);
        pagmo::problem::hovering_problem_neural_network prob(rand(), kNumEvaluations, kSimulationTime, kNumHiddenNeurons);

        // This instantiates a population within the original bounds (-1,1)
        pagmo::population pop_temp(prob, kPopulationSize);

        // We make the bounds larger to allow neurons weights to grow
        prob.set_bounds(-20,20);

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
    ArchipelagoEvolve(archi, kNumGenerations);
}


void TrainProportionalDerivativeController() {
    ConfigurationPaGMO();

    std::cout << std::setprecision(10);

    // We instantiate a PSO algorithm capable of coping with stochastic prolems
    pagmo::algorithm::pso_generational algo(1,0.7298,2.05,2.05,0.05);

    std::cout << "Initializing PD controller evolution ....";

    pagmo::archipelago archi = pagmo::archipelago(pagmo::topology::fully_connected());

    for (unsigned int j = 0;j < kNumIslands; ++j) {
        std::cout << " [" << j;
        fflush(stdout);
        pagmo::problem::hovering_problem_proportional_derivative prob(rand(), kNumEvaluations, kSimulationTime);

        // This instantiates a population within the original bounds (-1,1)
        pagmo::population pop_temp(prob, kPopulationSize);

        // We make the bounds larger to allow neurons weights to grow
        prob.set_bounds(-20,20);

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

    ArchipelagoEvolve(archi, kNumGenerations);
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

    const pagmo::decision_vector &solution = {0.1706789394, 6.391103265, -4.844305805, 14.79450134, 2.461898567, -0.3381245896, -4.949960837, 14.10351753, 8.883169321, -1.326416109, -0.1696018523, -16.89241883, -6.437639831, 6.395617865, -1.647325562, -1.385827668, -2.62581393, -6.500836362, 2.401432207, 1.630369467, 3.385221801, -9.965975545, -3.013213706, 6.000530322, 2.584463077, -2.561646792, 7.579409453, -9.707534915, 4.034683863, 3.444476854, 5.207027969, -1.505561988, -2.964210528, -5.924351493, -2.647210083};


    std::cout << std::setprecision(10);

    pagmo::problem::hovering_problem_neural_network prob(random_seed, kNumEvaluations, kSimulationTime, kNumHiddenNeurons);


    /*
    SampleFactory sample_factory(random_seed);
    pagmo::decision_vector rand_guess;
    for (unsigned int i = 0; i < solution.size(); ++i) {
        rand_guess.push_back(sample_factory.SampleUniform(-1.0, 1.0));
    }
    ConvexityCheck(prob, random_seed, rand_guess);
    return;
    */

    std::cout << "Checking NN controller fitness... ";
    const double fitness = prob.objfun_seeded(random_seed, solution)[0];
    std::cout << fitness << std::endl;

    std::cout << "Simulating NN controller ... ";
    PaGMOSimulationNeuralNetwork simulation(random_seed, 86400.0, kNumHiddenNeurons, solution);
    const boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > result = simulation.EvaluateAdaptive();
    const std::vector<double> &times = boost::get<0>(result);
    const std::vector<Vector3D> &positions = boost::get<2>(result);
    const std::vector<Vector3D> &heights = boost::get<3>(result);
    const std::vector<Vector3D> &velocities = boost::get<4>(result);
    const std::vector<Vector3D> &thrusts = boost::get<5>(result);
    std::cout << "done." << std::endl;

    std::cout << "Writing visualization file ... ";
    FileWriter writer_visualization(PATH_TO_NEURO_TRAJECTORY_FILE);
    writer_visualization.CreateVisualizationFile(simulation.ControlFrequency(), simulation.AsteroidOfSystem(), positions, heights);
    std::cout << "done." << std::endl;

    std::cout << "Writing evaluation file ... ";
    FileWriter writer_evaluation(PATH_TO_NEURO_EVALUATION_FILE);
    writer_evaluation.CreateEvaluationFile(random_seed, simulation.TargetPosition(), times, positions, velocities, thrusts);
    std::cout << "done." << std::endl;


    return;

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

void TestProportionalDerivativeController(const unsigned int &random_seed) {
    ConfigurationPaGMO();

    const pagmo::decision_vector &solution = {19.42617142, -0.1578370174, -1.158468383, 20, -0.645107367, -12.69431901, 1.310052018, 18.60307364, -2.021242612, 3.288074883, 19.81166744, 12.39927012, -0.4653634484, 0.06561330597, 12.82200748, 6.208813021, -9.63074012, 19.93887631};

    std::cout << std::setprecision(10);

    pagmo::problem::hovering_problem_proportional_derivative prob(random_seed, kNumEvaluations, kSimulationTime);


    std::cout << "Checking PD controller fitness... ";
    const double fitness = prob.objfun_seeded(random_seed, solution)[0];
    std::cout << fitness << std::endl;

    std::cout << "Simulating PD controller ... ";
    PaGMOSimulationProportionalDerivative simulation(random_seed, 86400.0, solution);
    const boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > result = simulation.EvaluateAdaptive();
    const std::vector<double> &times = boost::get<0>(result);
    const std::vector<Vector3D> &positions = boost::get<2>(result);
    const std::vector<Vector3D> &heights = boost::get<3>(result);
    const std::vector<Vector3D> &velocities = boost::get<4>(result);
    const std::vector<Vector3D> &thrusts = boost::get<5>(result);
    std::cout << "done." << std::endl;

    std::cout << "Writing visualization file ... ";
    FileWriter writer_visualization(PATH_TO_FULL_STATE_TRAJECTORY_FILE);
    writer_visualization.CreateVisualizationFile(simulation.ControlFrequency(), simulation.AsteroidOfSystem(), positions, heights);
    std::cout << "done." << std::endl;

    std::cout << "Writing evaluation file ... ";
    FileWriter writer_evaluation(PATH_TO_FULL_STATE_EVALUATION_FILE);
    writer_evaluation.CreateEvaluationFile(random_seed, simulation.TargetPosition(), times, positions, velocities, thrusts);
    std::cout << "done." << std::endl;

    std::cout << "Performing post evaluation ... ";
    const boost::tuple<std::vector<unsigned int>, std::vector<double>, std::vector<std::pair<double, double> > > post_evaluation = prob.post_evaluate(solution, random_seed);
    const std::vector<unsigned int> &random_seeds = boost::get<0>(post_evaluation);
    const std::vector<double> &mean_errors = boost::get<1>(post_evaluation);
    const std::vector<std::pair<double, double> > &min_max_errors = boost::get<2>(post_evaluation);

    std::cout << "done." << std::endl << "Writing post evaluation file ... ";
    FileWriter writer_post_evaluation(PATH_TO_FULL_STATE_POST_EVALUATION_FILE);
    writer_post_evaluation.CreatePostEvaluationFile(random_seeds, mean_errors, min_max_errors);
    std::cout << "done." << std::endl;
}


void TestNeuralNetworkVSFullStateController(const unsigned int &random_seed) {
    ConfigurationPaGMO();

    const pagmo::decision_vector &solution_neural_network = {0.04474285262, -1.181798445, 5.990240969, 3.368014356, -0.3884005916, 7.925862891, -4.007351087, -0.9033229497, 0.06888286759, -0.7082884631, -0.5803069954, 4.363130184, 6.764881924, 5.854863165, 0.1400946691, -1.50547086, -0.5521360017, 4.45201907, -4.250967073, -7.918608319, 9.717649144, -0.207032013, -5.833015115, -6.733938867, -4.608356761, -8.060719304, -1.410652379, -1.818076147, -0.8461389996, -2.954327447, -0.8073555435, -0.9967155903, -7.519709687, 3.701398114, 1.227121096, -0.3308526675, 0.7947630271, 4.84080388, -1.145269555, 0.4878662145, 5.394556383, -1.617437627, 3.819445141, -3.798915672, 4.164880788, -1.518176575, -3.984603965, -5.053259696, 2.04070011, -2.4627301, 5.187969076, 0.5709029065, -3.823333832, -2.089997512, 1.734804532, 5.298199453, -3.459244202, 0.3670544974, 3.308344852, 7.871539459, -3.628642295, 0.0001312112673, -0.5016102762};

    const pagmo::decision_vector &solution_full_state = {19.62982929, 2.562497127, 3.627020967, 19.86051527, -0.4731542458, 3.46844508, 0.1620948448, 19.30288406, 2.296871748, -2.52698378, 19.99586136, 4.082133337, 1.996530484, 1.526402493, 12.49818943, -0.5111650787, -1.310274189, 19.98615408};


    pagmo::problem::hovering_problem_proportional_derivative prob_full_state(random_seed, kNumEvaluations, kSimulationTime);
    pagmo::problem::hovering_problem_neural_network prob_neural_network(random_seed, kNumEvaluations, kSimulationTime);

    std::cout << "Performing post evaluation ... ";

    const boost::tuple<std::vector<unsigned int>, std::vector<double>, std::vector<std::pair<double,double> > > post_evaluation_neural_network = prob_neural_network.post_evaluate(solution_neural_network, random_seed);
    const std::vector<unsigned int> &random_seeds = boost::get<0>(post_evaluation_neural_network);
    const std::vector<double> &mean_errors_neural_network = boost::get<1>(post_evaluation_neural_network);
    const std::vector<std::pair<double, double> > &min_max_errors_neural_network = boost::get<2>(post_evaluation_neural_network);


    const boost::tuple<std::vector<unsigned int>, std::vector<double>, std::vector<std::pair<double,double> > > post_evaluation_full_state = prob_full_state.post_evaluate(solution_full_state, 0, random_seeds);
    const std::vector<double> &mean_errors_full_state = boost::get<1>(post_evaluation_full_state);
    const std::vector<std::pair<double, double> > &min_max_errors_full_state = boost::get<2>(post_evaluation_full_state);


    std::cout << "done." << std::endl << "Writing post evaluation files ... ";
    FileWriter writer_neural_network(PATH_TO_NEURO_POST_EVALUATION_FILE);
    writer_neural_network.CreatePostEvaluationFile(random_seeds, mean_errors_neural_network, min_max_errors_neural_network);

    FileWriter writer_full_state(PATH_TO_FULL_STATE_POST_EVALUATION_FILE);
    writer_full_state.CreatePostEvaluationFile(random_seeds, mean_errors_full_state, min_max_errors_full_state);
    std::cout << "done." << std::endl;
}
