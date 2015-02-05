#include "evolutionaryrobotics.h"
#include "hoveringproblem.h"
#include "samplefactory.h"
#include "filewriter.h"
#include "configuration.h"

// Training configuration
static const unsigned int num_generations = 1000;
static const unsigned int population_size = 100;
static const unsigned int num_islands = 4;
static const double simulation_time = 1.0 * 60.0 * 60.0;
static const unsigned int num_evaluations = 4;
static const unsigned int num_hidden_neurons = 6;



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

    std::cout << "Evolutionary training configuration" << std::endl;
    std::cout << "number of generations: " << num_generations << std::endl;
    std::cout << "population size: " << population_size << std::endl;
    std::cout << "number of islands: " << num_islands << std::endl;
    std::cout << "simulation time: " << simulation_time << std::endl;
#ifdef HP_FIXED_SEED
    std::cout << "number of evaluations: 1" << std::endl;
#else
    std::cout << "number of evaluations: " << num_evaluations << std::endl;
#endif
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

}


void TestNeuralNetworkController() {
    const pagmo::decision_vector &controller_parameters = {1.139453374, -0.5035586702, 11.83837624, -0.9314071541, -2.35407996, 0.833989195, -8.985921616, -1.043738767, -2.776785005, -10.22947939, 0.4796012886, -3.150659766, -14.13180575, -0.1625557165, 2.645001955, 0.5260188324, 2.110138671, 1.412664598, -3.365195982, -1.177259109, -1.564083124, -0.05350921614, 3.481537105, 4.797028985, 1.13560567, 0.7365295729, -0.5853098727, -0.7883426354, -1.025992628, 6.183600503, -15.56626127, -12.74000029, 3.143713937, -5.515504565, 1.522294359, 1.619518748, 1.265833975, -2.500423808, 0.1574967832, 4.704057427, -4.804911663, 2.075984812, -1.765734238, -1.513891454, -1.877287754, -0.09109732109, 0.7385496744, 1.454737143, 3.170372551, -0.2155375387, 2.253057173, -2.024282433, -0.4122590442, 1.991515282, -0.4001789465, -1.689672235, 0.08195932494, -2.396089453, 0.6537027275, 2.080296207, 0.2556261137, -1.086728015, -0.2556162367};

    SampleFactory sample_factory;


    double mean_fitness = 0.0;

#ifdef HP_FIXED_SEED
    mean_fitness = pagmo::problem::hovering_problem(sample_factory.SampleRandomInteger(), num_evaluations, simulation_time, num_hidden_neurons).objfun(controller_parameters)[0];

#else
    const unsigned int num_tests = 100;
    for (unsigned int i = 0; i < num_tests; ++i) {
        pagmo::problem::hovering_problem prob(sample_factory.SampleRandomInteger(), num_evaluations, simulation_time, num_hidden_neurons);
        pagmo::fitness_vector fitness = prob.objfun(controller_parameters);
        mean_fitness += fitness[0];
    }
    mean_fitness /= num_tests;
#endif

    std::cout << "mean fitness of chapmion: " << mean_fitness << std::endl;

#ifdef HP_FIXED_SEED
    PaGMOSimulationNeuralNetwork sim(HP_FIXED_SEED, 24.0 * 60.0 * 60.0, 6, controller_parameters);
#else
    PaGMOSimulationNeuralNetwork sim(sample_factory.SampleRandomInteger(), 86400.0, num_hidden_neurons, controller_parameters);
#endif

    std::cout << "testing neuro controller ... ";
    const boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > r1 = sim.EvaluateAdaptive();
    std::cout << "done." << std::endl << "writing result to file ... ";

    const std::vector<Vector3D> &positions = boost::get<2>(r1);
    const std::vector<Vector3D> &heights = boost::get<3>(r1);

    FileWriter writer;
    writer.CreateVisualizationFile(PATH_TO_NEURO_VISUALIZATION_FILE, 1.0 / sim.InteractionInterval(), sim.AsteroidOfSystem(), positions, heights);
    std::cout << "done." << std::endl;

}
