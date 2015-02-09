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
    const pagmo::decision_vector &controller_parameters = {-0.747003615, -4.655372696, -5.820524281, 0.4917795358, -10.55576246, -0.7847575396, -3.778163064, 0.01707794801, 2.156707482, -6.186894822, -6.459054113, 6.521388439, -0.9582162699, 2.277597915, -0.3091513314, 0.5313651178, 0.6266070788, -2.264479814, 0.7237817652, -2.189075674, -5.942460416, 0.8335105367, -6.179160576, -4.179755773, -1.268472931, -9.136555202, -3.128254453, 3.209922469, 3.125601262, 0.6768597883, 0.09513850953, 1.557494864, -1.783048039, 11.62262349, 1.92518565, 0.7387033585, -1.259430373, -3.818802523, 3.430180025, 0.06453246724, -13.90993168, 7.293075065, 2.423550511, -8.978639948, 4.835654713, -2.372304393, -6.54246587, 0.8900295273, 4.000157653, 1.3518109, -2.816927661, -3.055357814, 0.08248106602, 2.196977633, 3.739722635, -6.0321973, 2.332685162, -2.354150126, -0.5950197091, -10.14926048, -2.36125312, 2.059603555, 4.018288219};


    pagmo::problem::hovering_problem prob(random_seed, num_evaluations, simulation_time, num_hidden_neurons);
    const double fitness = prob.objfun_seeded(random_seed, controller_parameters)[0];
    std::cout << "fitness of chapmion: " << fitness << std::endl;

    std::cout << "testing neuro controller ... ";
    PaGMOSimulationNeuralNetwork sim(random_seed, 86400.0, num_hidden_neurons, controller_parameters);
    const boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > r1 = sim.EvaluateAdaptive();
    std::cout << "done." << std::endl << "writing result to file ... ";

    const std::vector<Vector3D> &positions = boost::get<2>(r1);
    const std::vector<Vector3D> &heights = boost::get<3>(r1);

    FileWriter writer;
    writer.CreateVisualizationFile(PATH_TO_NEURO_VISUALIZATION_FILE, sim.ControlFrequency(), sim.AsteroidOfSystem(), positions, heights);
    std::cout << "done." << std::endl;

}
