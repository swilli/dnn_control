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


void TestNeuralNetworkController(const unsigned int &random_seed) {
    const pagmo::decision_vector &controller_parameters = {0.1978105894, 1.382211678, 2.276014568, 0.6683596403, 0.3859042599, 2.344483419, -2.264480489, 5.793769836, -0.2330933158, -1.627351593, 0.2208166187, -0.6686395508, 2.981372047, -2.026256274, -0.1550951127, 0.4123616579, -1.728663746, 1.300941408, -0.4339265121, -1.306791986, -0.7133511336, -0.1842525721, 0.07738779019, 0.8667420416, -1.56934758, -0.9145037076, -0.6125364647, -0.638653143, -0.7861275365, -0.7205643259, 1.065781712, -3.415023842, 1.303809701, 0.2379075187, 1.975742981, -0.04073831012, 1.576128779, 1.275048406, 0.06431794331, 2.814600633, -0.4550991784, -0.8682688434, -1.291888243, 0.3716955926, -0.09916130628, -0.4672770932, -1.213621322, -0.3700553088, 4.962021989, -1.368753253, 4.91185464, -0.9716114216, -2.554696074, 1.925727968, -0.5785447512, 0.3314102377, -0.6854279439, 0.5132619396, 0.7264452245, -0.09451854397, -1.38357286, -0.7135350908, 0.8917496877};

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
