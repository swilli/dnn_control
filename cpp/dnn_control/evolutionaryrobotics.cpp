#include "evolutionaryrobotics.h"
#include "hoveringproblem.h"
#include "samplefactory.h"


// Training configuration
static const unsigned int num_generations = 1000;
static const unsigned int population_size = 100;
static const unsigned int num_islands = 4;
static const double simulation_time = 6.0 * 60.0 * 60.0;
static const unsigned int num_evaluations = 4;
static const unsigned int num_hidden_neurons = 5;



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
    std::cout << std::setprecision(5);

    std::cout << "number of generations: " << num_generations << std::endl;
    std::cout << "population size: " << population_size << std::endl;
    std::cout << "number of islands: " << num_islands << std::endl;
    std::cout << "simulation time: " << simulation_time << std::endl;
    std::cout << "number of evaluations: " << num_evaluations << std::endl;
    std::cout << "number of hidden neurons: " << num_hidden_neurons << std::endl << std::endl;

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
        std::cout << std::endl << asctime(timeinfo) << "gen: "<< std::setw(12) << i << std::setw(12) <<
                     best_f << std::setw(12) <<
                     archi.get_island(idx)->get_population().mean_velocity() << std::setw(12) <<
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

}


void TestNeuralNetworkController(const pagmo::decision_vector &champion) {
    const unsigned int num_tests = 1;
    double mean_fitness = 0.0;

    for (unsigned int i = 0; i < num_tests; ++i) {
#ifdef PROBLEM_FIXED_SEED
        pagmo::problem::hovering_problem prob(PROBLEM_FIXED_SEED, num_evaluations, simulation_time, num_hidden_neurons);
#else
        pagmo::problem::hovering_problem prob(rand(), num_evaluations, simulation_time, num_hidden_neurons);
#endif

        pagmo::fitness_vector fitness = prob.objfun(champion);
        mean_fitness += fitness[0];
    }
    mean_fitness /= (double) num_tests;
    std::cout << "mean fitness of chapmion: " << mean_fitness << std::endl;
}
