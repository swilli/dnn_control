#include "evolutionaryrobotics.h"
#include "hoveringproblem.h"
#include "samplefactory.h"
#include "filewriter.h"
#include "configuration.h"

// Training configuration
static const unsigned int num_generations = ER_NUM_GENERATIONS;
static const unsigned int population_size = ER_POPULATION_SIZE;
static const unsigned int num_islands = ER_NUM_ISLANDS;
static const double simulation_time = ER_SIMULATION_TIME;
static const unsigned int num_evaluations = ER_EVALUATIONS;
static const unsigned int num_hidden_neurons = ER_NUM_HIDDEN_NODES;



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

void TrainNeuralNetworkController() {
    std::cout << std::setprecision(10);

    // Buffer
    std::vector<double> buff;
    // We instantiate a PSO algorithm capable of coping with stochastic prolems
    pagmo::algorithm::pso_generational algo(1,0.7298,2.05,2.05,0.05);

    // This instantiates the spheres problem
    std::cout << "Initializing ....";

    pagmo::archipelago archi = pagmo::archipelago(pagmo::topology::fully_connected());

    for (unsigned int j = 0;j < num_islands; ++j) {
        std::cout << " [" << j;
        fflush(stdout);
        pagmo::problem::hovering_problem prob(rand(), num_evaluations, simulation_time, num_hidden_neurons);
        //pagmo::problem::spheres prob;
        // This instantiates a population within the original bounds (-1,1)
        pagmo::population pop_temp(prob, population_size);

        // We make the bounds larger to allow neurons weights to grow
        prob.set_bounds(-20,20);

        // We create an empty population on the new prolem (-10,10)
        pagmo::population pop(prob);

        // And we fill it up with (-1,1) individuals having zero velocities
        pagmo::decision_vector v(prob.get_dimension(),0);
        for (unsigned int i = 0; i < population_size; ++i) {
            pop.push_back(pop_temp.get_individual(i).cur_x);
            pop.set_v(i,v);
        }
        archi.push_back(pagmo::island(algo,pop));
        std::cout << "]";
        fflush(stdout);
    }

    std::cout << " done" << std::endl << "Evolving ..." << std::endl;
    fflush(stdout);

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
        std::cout << std::endl << asctime(timeinfo) << "gen: "<< std::setw(20) << i << std::setw(20) <<
                     best_f << std::setw(20) <<
                     archi.get_island(idx)->get_population().mean_velocity() << std::setw(20) <<
                     mean <<	 std::endl << "[";
        const pagmo::decision_vector weights = archi.get_island(idx)->get_population().champion().x;
        for (unsigned int i = 0; i < weights.size() -1; ++i) {
            std::cout << weights[i] << ", ";
        }
        std::cout << weights.back() << "]" << std::endl;
        fflush(stdout);
        archi.evolve(1);
    }

    const unsigned int idx = ArchipelagoChampionID(archi);
    std::cout << std::endl << "and the winner is ......" << std::endl << "[";
    const pagmo::decision_vector weights = archi.get_island(idx)->get_population().champion().x;
    for (unsigned int i = 0; i < weights.size() -1; ++i) {
        std::cout << weights[i] << ", ";
    }
    std::cout << weights.back() << "]" << std::endl;


    std::cout << std::endl << "copy this into c++ code:" << std::endl << "{";
    for (unsigned int i = 0; i < weights.size() -1; ++i) {
        std::cout << weights[i] << ", ";
    }
    std::cout << weights.back() << "};" << std::endl;
}


void TestNeuralNetworkController(const unsigned int &random_seed) {
    const pagmo::decision_vector &task_42_solution = {-2.264215366, 19.0228605, -19.97297023, -5.280077574, 4.518311502, 5.163603879, 0.8487906712, -0.8810536689, 8.90129035, -20, -6.253330405, -2.441120883, -7.085009194, -0.7619971295, -0.2084938772, 19.99999455, -19.96675742, -6.435747406, 6.397022563, -4.269799254, -5.570030497, -1.16947738, 9.378605361, 9.755412363, -4.393819313, 13.10375298, 17.08885594, -13.20683153, -5.783114715, 4.257917366, -1.687090094, -7.828039553, -3.564753685, -3.392482838, -2.845498846, -1.771198234, -2.503716683, 13.50625966, -19.93187244, 2.580757208, -6.024931516, -7.467124732, -0.6024611812, 0.650475947, -0.1534687856, 1.04748741, 1.133330376, 2.860050062, -1.765945678, 1.081282265, -0.205388396, -4.92600947, -0.2461220559, 2.466981906, -0.7637637321, -0.4239706062, 1.394008687, 0.5186059725, -1.348621749, 0.2102677486, -1.895668227, -1.581702878, -4.70508184};
    const pagmo::decision_vector &task_41_solution = {-0.4370981211, 3.21138701, 6.886163448, 1.136612282, 2.527666196, 12.73454651, 1.691078479, -0.2848608371, -6.804388104, 1.622634735, 0.5365383529, -11.0338932, 4.119857307, 1.124319846, -10.51493379, -0.7655563192, -0.7152648527, -3.542941152, 0.7307760809, 0.4844805912, 3.080696121, 2.130790191, -1.748408068, -1.671212504, -5.102739103, 0.02316490976, -0.2382445445, -1.393224155, -0.2236279625, -1.510556893, -1.553896322, -5.524639359, -3.083877895, -3.241961594, -14.64969265, -1.489388566, -2.491355935, -5.108025575, -10.89379337, 6.89323915, 3.082915982, 3.467835035, 2.650842816, 3.376802777, -6.885963194, 0.6006717076, 0.02905346538, -2.568890606, 0.889299374, -4.240239848, 7.601681267, 1.076938352, -2.714019567, 1.670151868, -1.090326347, -0.7680789368, 3.026463472, -0.03705450781, 0.6374352789, -4.513436077, -0.5598452314, -5.834699937, -1.005374523};

    std::cout << std::setprecision(10);

    pagmo::problem::hovering_problem prob(random_seed, num_evaluations, simulation_time, num_hidden_neurons);

    /*
    std::cout << "checking neuro controller fitness... ";
    const double fitness = prob.objfun_seeded(random_seed, task_42_solution)[0];
    std::cout << fitness << std::endl;

    std::cout << "simulating neuro controller ... ";
    PaGMOSimulationNeuralNetwork simulation(random_seed, 86400.0, num_hidden_neurons, task_42_solution);
    const boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > result = simulation.EvaluateAdaptive();
    const std::vector<double> &times = boost::get<0>(result);
    const std::vector<Vector3D> &positions = boost::get<2>(result);
    const std::vector<Vector3D> &heights = boost::get<3>(result);
    const std::vector<Vector3D> &velocities = boost::get<4>(result);
    const std::vector<Vector3D> &thrusts = boost::get<5>(result);


    std::cout << "done." << std::endl << "writing visualization file ... ";
    FileWriter writer_visualization(PATH_TO_NEURO_TRAJECTORY_FILE);
    writer_visualization.CreateVisualizationFile(simulation.ControlFrequency(), simulation.AsteroidOfSystem(), positions, heights);
    std::cout << "done." << std::endl;

    std::cout << "writing evaluation file ... ";
    FileWriter writer_evaluation(PATH_TO_NEURO_EVALUATION_FILE);
    writer_evaluation.CreateEvaluationFile(random_seed, simulation.TargetPosition(), times, positions, velocities, thrusts);
    std::cout << "done." << std::endl;
    */

    std::cout << "performing post evaluation ... ";
    std::vector<std::vector<double> > fitness_tasks;
    const boost::tuple<std::vector<double>, std::vector<unsigned int> > post_evaluation_task_42 = prob.post_evaluate(task_42_solution, random_seed);
    const std::vector<unsigned int> &random_seeds = boost::get<1>(post_evaluation_task_42);
    const std::vector<double> &fitness_task_42 = boost::get<0>(post_evaluation_task_42);
    fitness_tasks.push_back(fitness_task_42);
    const boost::tuple<std::vector<double>, std::vector<unsigned int> > post_evaluation_task_41 = prob.post_evaluate(task_41_solution, 0, random_seeds);
    const std::vector<double> &fitness_task_41 = boost::get<0>(post_evaluation_task_41);
    fitness_tasks.push_back(fitness_task_41);

    std::cout << "done." << std::endl << "writing post evaluation file ... ";
    FileWriter writer_post_evaluation(PATH_TO_NEURO_POST_EVALUATION_FILE);
    writer_post_evaluation.CreatePostEvaluationFile(random_seeds, fitness_tasks);
    std::cout << "done." << std::endl;
}
