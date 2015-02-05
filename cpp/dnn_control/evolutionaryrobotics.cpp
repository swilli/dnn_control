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
    const pagmo::decision_vector &controller_parameters = {1.139195728, -0.5226369611, 11.50837032, -0.9243382034, -2.353741217, 0.8235154027, -8.055780662, -1.043735176, -2.573237816, -9.513282065, 0.4797968306, -3.154748448, -10.5526454, -0.7267928348, 2.644780927, 0.4329906785, 2.089466159, 1.431430092, -3.605546448, -1.177694193, -1.564619942, -0.05351109618, 3.487903231, 4.94242206, 1.135662977, 0.7474408005, -0.5729041083, -0.7939810009, -1.026073727, 6.185496898, -14.56338275, -12.77003101, 3.141353625, -5.512882646, 1.535766781, 1.619485065, 1.269288631, -2.886643146, 0.134244204, 4.712408697, -4.923125927, 2.011897279, -1.766100603, -1.513967254, -1.878922867, -0.09092549268, 0.7385568409, 1.454678353, 3.170376046, -0.215203178, 2.252984266, -2.024107472, -0.4122462115, 1.990177368, -0.4001560543, -1.689947148, 0.08185300814, -2.396049012, 0.6536741258, 2.080319842, 0.2566107572, -1.086708268, -0.2556895039};

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
    writer.CreateVisualizationFile(PATH_TO_NEURO_VISUALIZATION_FILE, sim.ControlFrequency(), sim.AsteroidOfSystem(), positions, heights);
    std::cout << "done." << std::endl;

}
