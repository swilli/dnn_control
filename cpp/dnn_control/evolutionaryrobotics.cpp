#include "evolutionaryrobotics.h"
#include "hoveringproblemneuralnetwork.h"
#include "hoveringproblemfullstate.h"
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
    std::cout << std::setprecision(10);

    // We instantiate a PSO algorithm capable of coping with stochastic prolems
    pagmo::algorithm::pso_generational algo(1,0.7298,2.05,2.05,0.05);

    std::cout << "Initializing neuro controller evolution ....";

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


void TrainFullStateController() {
    std::cout << std::setprecision(10);

    // We instantiate a PSO algorithm capable of coping with stochastic prolems
    pagmo::algorithm::pso_generational algo(1,0.7298,2.05,2.05,0.05);

    std::cout << "Initializing PID controller evolution ....";

    pagmo::archipelago archi = pagmo::archipelago(pagmo::topology::fully_connected());

    for (unsigned int j = 0;j < kNumIslands; ++j) {
        std::cout << " [" << j;
        fflush(stdout);
        pagmo::problem::hovering_problem_full_state prob(rand(), kNumEvaluations, kSimulationTime);

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

    std::cout << "Checking NN controller convexity... ";

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
    const pagmo::decision_vector &task_42_solution = {-2.264215366, 19.0228605, -19.97297023, -5.280077574, 4.518311502, 5.163603879, 0.8487906712, -0.8810536689, 8.90129035, -20, -6.253330405, -2.441120883, -7.085009194, -0.7619971295, -0.2084938772, 19.99999455, -19.96675742, -6.435747406, 6.397022563, -4.269799254, -5.570030497, -1.16947738, 9.378605361, 9.755412363, -4.393819313, 13.10375298, 17.08885594, -13.20683153, -5.783114715, 4.257917366, -1.687090094, -7.828039553, -3.564753685, -3.392482838, -2.845498846, -1.771198234, -2.503716683, 13.50625966, -19.93187244, 2.580757208, -6.024931516, -7.467124732, -0.6024611812, 0.650475947, -0.1534687856, 1.04748741, 1.133330376, 2.860050062, -1.765945678, 1.081282265, -0.205388396, -4.92600947, -0.2461220559, 2.466981906, -0.7637637321, -0.4239706062, 1.394008687, 0.5186059725, -1.348621749, 0.2102677486, -1.895668227, -1.581702878, -4.70508184};
    const pagmo::decision_vector &task_41_solution = {-0.2653449146, 0.8221059793, 7.115913466, 0.9384741461, 2.460804959, 12.3201749, 1.871427235, -0.07533608116, -8.801983599, 1.642141599, -0.4937348003, -11.74057504, 3.7529415, 5.392689477, -7.77324484, -1.770696028, -1.183963091, -5.650056297, 1.019963074, 1.392104027, 1.926941641, 2.285940845, -1.079293945, -1.105212255, -3.894913852, -1.532887634, 2.790675652, -2.038766662, -0.7017624366, -2.263868159, -1.198603142, -4.826437788, -3.92156976, -2.00905927, -15.71152901, -2.343697858, -1.634728259, -4.242889368, -11.67533841, 7.999951956, 4.194276093, 7.023201787, 2.993808039, 2.996861109, -7.245632379, 1.343217234, -0.08975019422, -2.389846969, 0.25375688, -4.253339765, 7.06271584, 1.547862271, -0.7536306029, 1.070464669, -0.8058834915, -1.815555643, 2.866735509, -1.269174723, 1.178642348, -4.484382836, -0.6004458266, -6.729774456, -0.7779761483};

    std::cout << std::setprecision(10);

    pagmo::problem::hovering_problem_neural_network prob(random_seed, kNumEvaluations, kSimulationTime, kNumHiddenNeurons);

    SampleFactory sample_factory(random_seed);
    pagmo::decision_vector rand_guess;
    for (unsigned int i = 0; i < task_41_solution.size(); ++i) {
        rand_guess.push_back(sample_factory.SampleUniform(-1.0, 1.0));
    }
    ConvexityCheck(prob, random_seed, rand_guess);
    return;

    std::cout << "Checking NN controller fitness... ";
    const double fitness = prob.objfun_seeded(random_seed, task_41_solution)[0];
    std::cout << fitness << std::endl;

    std::cout << "Simulating NN controller ... ";
    PaGMOSimulationNeuralNetwork simulation(random_seed, 86400.0, kNumHiddenNeurons, task_41_solution);
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

    std::cout << "Performing post evaluation ... ";
    std::vector<std::vector<double> > fitness_tasks;
    const boost::tuple<std::vector<double>, std::vector<unsigned int> > post_evaluation_task_42 = prob.post_evaluate(task_42_solution, random_seed);
    const std::vector<unsigned int> &random_seeds = boost::get<1>(post_evaluation_task_42);
    const std::vector<double> &fitness_task_42 = boost::get<0>(post_evaluation_task_42);
    fitness_tasks.push_back(fitness_task_42);
    const boost::tuple<std::vector<double>, std::vector<unsigned int> > post_evaluation_task_41 = prob.post_evaluate(task_41_solution, 0, random_seeds);
    const std::vector<double> &fitness_task_41 = boost::get<0>(post_evaluation_task_41);
    fitness_tasks.push_back(fitness_task_41);

    std::cout << "done." << std::endl << "Writing post evaluation file ... ";
    FileWriter writer_post_evaluation(PATH_TO_NEURO_POST_EVALUATION_FILE);
    writer_post_evaluation.CreatePostEvaluationFile(random_seeds, fitness_tasks);
    std::cout << "done." << std::endl;
}

void TestFullStateController(const unsigned int &random_seed) {
    const pagmo::decision_vector &task_44_solution = {3.187865839, 19.93864888, 0.7697773176, 19.98659286, -17.4575995, 18.70914741};

    std::cout << std::setprecision(10);

    pagmo::problem::hovering_problem_full_state prob(random_seed, kNumEvaluations, kSimulationTime);


    std::cout << "Checking PID controller fitness... ";
    const double fitness = prob.objfun_seeded(random_seed, task_44_solution)[0];
    std::cout << fitness << std::endl;

    std::cout << "Simulating PID controller ... ";
    PaGMOSimulationFullState simulation(random_seed, 86400.0, task_44_solution);
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
    std::vector<std::vector<double> > fitness_tasks;
    const boost::tuple<std::vector<double>, std::vector<unsigned int> > post_evaluation_task_44 = prob.post_evaluate(task_44_solution, random_seed);
    const std::vector<unsigned int> &random_seeds = boost::get<1>(post_evaluation_task_44);
    const std::vector<double> &fitness_task_44 = boost::get<0>(post_evaluation_task_44);
    fitness_tasks.push_back(fitness_task_44);


    std::cout << "done." << std::endl << "Writing post evaluation file ... ";
    FileWriter writer_post_evaluation(PATH_TO_FULL_STATE_POST_EVALUATION_FILE);
    writer_post_evaluation.CreatePostEvaluationFile(random_seeds, fitness_tasks);
    std::cout << "done." << std::endl;
}
