#include "adaptivesimulation.h"
#include "fixedsimulation.h"
#include "pagmosimulationneuralnetwork.h"
#include "pagmosimulationfullstate.h"
#include "filewriter.h"
#include "hoveringproblem.h"

#include <pagmo/src/pagmo.h>


#define FULL_STATE_CONTROLLED                                                 0

#define GENERATE_SENSOR_DATA_FILES                                            0
#define NUM_SENSOR_DATA_FILES                                                 5
#define PATH_TO_SENSOR_DATA_FOLDER                                            "../../../data/"

#define CREATE_RANDOM_VISUALIZATION_FILE                                      1
#define PATH_TO_RANDOM_VISUALIZATION_FILE                                     "../../../results/visualization.txt"

static const std::vector<double> kNeuralNetworkWeights = {-0.34105, -0.63153, -0.21013, -4.4611, 3.1548, -2.8539, -3.3003, -1.1532, 1.2563, -0.61968, 1.3154, 2.5086, 2.6536, 3.2999, 0.29513, -0.33427, 2.3713, -1.0736, 5.4118, 1.6048, -1.7567, -0.087932, -0.46124, 0.80393, 3.6069, 0.76005, -0.97259, 0.40637, -5.605, 0.99282, 0.75947, 5.5112, -1.1204, -3.2563, 0.58268, 3.6061, 0.74423, 1.0826};

static const std::vector<double> kCoefficientsFullState = {0.23, 20.0, 0.0};

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

static void TrainNeuralNetworkController() {
    // Training configuration
    const unsigned int num_generations = 1000;
    const unsigned int population_size = 80;
    const unsigned int num_islands = 4;
    const double simulation_time = 6.0 * 60.0 * 60.0;
    const unsigned int num_evaluations = 8;
    const unsigned int num_hidden_neurons = 5;

    std::cout << std::setprecision(5);

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
        int idx = ArchipelagoChampionID(archi);
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
        const pagmo::decision_vector &weights = archi.get_island(idx)->get_population().champion().x;
        for (unsigned int i = 0; i < weights.size() -1; ++i) {
            std::cout << weights[i] << ", ";
        }
        std::cout << weights.back() << "]" << std::endl;
        fflush(stdout);
        archi.evolve(1);
    }

    const unsigned int idx = ArchipelagoChampionID(archi);
    std::cout << "and the winner is ......" << "\n";
    const pagmo::decision_vector &weights = archi.get_island(idx)->get_population().champion().x;
    for (unsigned int i = 0; i < weights.size() -1; ++i) {
        std::cout << weights[i] << ", ";
    }
    std::cout << weights.back() << "]" << std::endl;

}

int main(int argc, char *argv[]) {
    srand(time(0));

    //TrainNeuralNetworkController();

    //return 0;



    const unsigned int num_tests = 100;
    double t_adapt = 0.0;
    double s_adapt = 0.0;
    double t_fixed = 0.0;
    double s_fixed = 0.0;
    double error = 0.0;
    double sim_time = 0.0;
    for (unsigned int i = 0; i < num_tests; ++i) {
        std::cout << i << std::endl;
        PaGMOSimulationFullState sim(rand(), 24.0 * 60.0 * 60.0, {4.0, 20.0, 0.0});
        clock_t begin = clock();
        boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > result = sim.EvaluateImpl2();
        clock_t end = clock();
        double simulated_time = boost::get<0>(result).back();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        double speedup = simulated_time/elapsed_secs;
        const std::vector<Vector3D> p_adapt = boost::get<2>(result);
        t_adapt += elapsed_secs;
        s_adapt += speedup;

        begin = clock();
        result = sim.EvaluateDetailedImpl2();
        end = clock();
        simulated_time = boost::get<0>(result).back();
        sim_time += simulated_time;
        elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        speedup = simulated_time/elapsed_secs;
        const std::vector<Vector3D> p_fixed = boost::get<2>(result);
        t_fixed += elapsed_secs;
        s_fixed += speedup;
        double cur_error = 0.0;
        const unsigned int f_size = p_fixed.size();
        const unsigned int a_size = p_adapt.size();
        const unsigned int min_size = (f_size < a_size ? f_size : a_size);
        if (min_size != f_size || min_size != a_size) {
            std::cout << "f_size: " << f_size << " a_size: " << a_size << std::endl;
        }
        for (unsigned int j  = 0; j < min_size;  ++j) {
            cur_error += VectorNorm(VectorSub(p_adapt.at(j), p_fixed.at(j)));
        }
        cur_error /= min_size;
        error += cur_error;
    }
    std::cout << "mean real sim time: " << sim_time / num_tests << std::endl;
    std::cout << "mean sim time adapt: " << t_adapt / num_tests << std::endl;
    std::cout << "mean speedup adapt: " << s_adapt / num_tests << std::endl;
    std::cout << "mean sim time fixed: " << t_fixed / num_tests << std::endl;
    std::cout << "mean speedup fixed: " << s_fixed / num_tests << std::endl;
    std::cout << "mean error: " << error / num_tests << std::endl;
    return 0;

    /*



    PaGMOSimulationNeuralNetwork sim(rand(), 86400.0, kNeuralNetworkWeights, 5);
    const boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > r1 = sim.EvaluateDetailed();
    const std::vector<Vector3D> &r1pos = boost::get<2>(r1);
    const std::vector<Vector3D> &r1hei = boost::get<3>(r1);

    FileWriter writer;
    writer.CreateVisualizationFile(PATH_TO_RANDOM_VISUALIZATION_FILE, 1.0 / sim.FixedStepSize(), sim.AsteroidOfSystem(), r1pos, r1hei);

    return 0;

    */
/*


    const unsigned int seed = 666;
    PaGMOSimulationFullState sim(seed, 24.0 * 60.0 * 60.0, {0.0, 0.0, 0.0}); //{4.0, 20.0, 0.0});
    const boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > result = sim.EvaluateImpl2();
    const std::vector<Vector3D> &pos = boost::get<2>(result);
    const std::vector<Vector3D> &hei = boost::get<3>(result);

    FileWriter writer;
    writer.CreateVisualizationFile(PATH_TO_RANDOM_VISUALIZATION_FILE, 1.0 / sim.InteractionInterval(), sim.AsteroidOfSystem(), pos, hei);

    return 0;

*/


    /*

    std::vector<double> zero_weights(weights.size(), 1.0 / weights.size());
    PaGMOSimulationNeuralNetwork nn_sim(rand(), 86400.0, zero_weights, 5);
    clock_t begin,end;
    begin = clock();
    const boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > r1 = nn_sim.Evaluate();
    end = clock();
    double secs = double (end - begin) / CLOCKS_PER_SEC;
    const std::vector<double> &times = boost::get<0>(r1);
    const double real_secs = times.back();
    std::cout << real_secs << " took " << secs << " seconds to compute (x" << real_secs / secs << ")" << std::endl;
    const std::vector<Vector3D> &r1pos = boost::get<2>(r1);
    const std::vector<Vector3D> &r1hei = boost::get<3>(r1);

    FileWriter writer;
    writer.CreateVisualizationFile(PATH_TO_RANDOM_VISUALIZATION_FILE, 1.0 / nn_sim.FixedStepSize(), nn_sim.AsteroidOfSystem(), r1pos, r1hei);

    */




    /*
    const unsigned int num_tests = 100;
    double t_adapt = 0.0;
    double s_adapt = 0.0;
    double t_fixed = 0.0;
    double s_fixed = 0.0;
    double error = 0.0;
    for (unsigned int i = 0; i < num_tests; ++i) {
        std::cout << i << std::endl;
        PaGMOSimulationNeuralNetwork p_sim(rand(), 6.0 * 60.0 * 60.0, kNeuralNetworkWeights);
        clock_t begin = clock();
        boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > result = p_sim.Evaluate();
        clock_t end = clock();
        double simulated_time = boost::get<0>(result).back();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        double speedup = simulated_time/elapsed_secs;
        const Vector3D p_adapt = boost::get<2>(result).back();
        t_adapt += elapsed_secs;
        s_adapt += speedup;

        begin = clock();
        result = p_sim.EvaluateDetailed();
        end = clock();
        simulated_time = boost::get<0>(result).back();
        elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        speedup = simulated_time/elapsed_secs;
        const Vector3D p_fixed = boost::get<2>(result).back();
        t_fixed += elapsed_secs;
        s_fixed += speedup;

        error =+ VectorNorm(VectorSub(p_adapt, p_fixed));
    }
    std::cout << "mean sim time adapt: " << t_adapt / num_tests << std::endl;
    std::cout << "mean speedup adapt: " << s_adapt / num_tests << std::endl;
    std::cout << "mean sim time fixed: " << t_fixed / num_tests << std::endl;
    std::cout << "mean speedup fixed: " << s_fixed / num_tests << std::endl;
    std::cout << "mean error: " << error / num_tests << std::endl;
    return 0;



    /* Copy constructor & assignment operator test */


    /*
    PaGMOSimulationFullState s1(500, 86400.0);
    PaGMOSimulationFullState s2(0, 86400.0);
    s2.Evaluate();
    PaGMOSimulationFullState s4(s2);
    {
        PaGMOSimulationFullState s3(0, 86400.0);
        s1 = s3;
    }

    const boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > r1 = s1.EvaluateDetailedImpl2();
    const boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > r2 = s2.EvaluateDetailedImpl2();
    const boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > r4 = s4.EvaluateDetailedImpl2();

    const std::vector<Vector3D> &r1pos = boost::get<2>(r1);
    const std::vector<Vector3D> &r1hei = boost::get<3>(r1);

    const std::vector<Vector3D> &r2pos = boost::get<2>(r2);
    const std::vector<Vector3D> &r2hei = boost::get<3>(r2);

    const std::vector<Vector3D> &r4pos = boost::get<2>(r4);
    const std::vector<Vector3D> &r4hei = boost::get<3>(r4);

    const Vector3D r1p = r1pos.back();
    const Vector3D r2p = r2pos.back();
    const Vector3D r4p = r4pos.back();

    FileWriter writer;
    writer.CreateVisualizationFile("../../../results/visualization1.txt", 1.0 / s1.FixedStepSize(), s1.AsteroidOfSystem(), r1pos, r1hei);
    writer.CreateVisualizationFile("../../../results/visualization2.txt", 1.0 / s2.FixedStepSize(), s2.AsteroidOfSystem(), r2pos, r2hei);
    writer.CreateVisualizationFile("../../../results/visualization4.txt", 1.0 / s4.FixedStepSize(), s4.AsteroidOfSystem(), r4pos, r4hei);
    return 0;

    */


    /*
    FixedSimulation sim(0);
    const boost::tuple<std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D> > f_result = sim.Evaluate();
    const std::vector<Vector3D> &f_positions = boost::get<1>(f_result);
    const std::vector<Vector3D> &f_heights = boost::get<2>(f_result);
    FileWriter writer;
    writer.CreateVisualizationFile(PATH_TO_RANDOM_VISUALIZATION_FILE, 1.0 / sim.FixedStepSize(), sim.AsteroidOfSystem(), f_positions, f_heights);

    return 0;

    */

    /*
    double error = 0.0;
    const unsigned int num_tests = 100;

    PaGMOSimulationFullState p_sim(0, 24.0 * 60.0 * 60.0);
    for (unsigned int i = 0; i < num_tests; ++i) {
        const unsigned int random_seed = rand();
        std::cout << "test " << (i + 1) << ", current seed: " << random_seed << std::endl;

        p_sim = PaGMOSimulationFullState(random_seed, 86400.0, {0.0, 0.0, 0.0});
        p_sim = p_sim;
        std::cout << "running fixed ... " << std::endl;
        clock_t begin = clock();
        const boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D> , std::vector<Vector3D> > f_result = p_sim.EvaluateDetailed();
        clock_t end = clock();
        double f_simulated_time = boost::get<0>(f_result).back();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        std::cout << f_simulated_time << " seconds simulation time took " << elapsed_secs << " real time to compute (x" << f_simulated_time/elapsed_secs << ")." << std::endl;

        const std::vector<double> &f_times = boost::get<0>(f_result);
        const std::vector<Vector3D> &f_positions = boost::get<2>(f_result);
        const Vector3D f_pos = f_positions.back();

        std::cout << "running adaptive ... " << std::endl;
        begin = clock();
        const boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>  > a_result = p_sim.Evaluate();
        end = clock();
        double a_simulated_time = boost::get<0>(a_result).back();
        elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        std::cout << a_simulated_time << " seconds simulation time took " << elapsed_secs << " real time to compute (x" << a_simulated_time/elapsed_secs << ")." << std::endl;

        const std::vector<double> &a_times = boost::get<0>(a_result);
        const std::vector<Vector3D> &a_positions = boost::get<2>(a_result);
        const Vector3D a_pos = a_positions.back();


        error += VectorNorm(VectorSub(a_pos, f_pos));
    }
    std::cout << "error: " << error / (double) num_tests << std::endl;
    std::cout << "done" << std::endl;

    return 0;
    */

    /*
    double error = 0.0;
    const unsigned int num_tests = 10;
    FixedSimulation f_sim(0);
    AdaptiveSimulation a_sim(0);
    for (unsigned int i = 0; i < num_tests; ++i) {
        const unsigned int random_seed = rand();
        std::cout << "test " << (i + 1) << ", current seed: " << random_seed << std::endl;

        f_sim = FixedSimulation(random_seed);
        std::cout << "running fixed ... " << std::endl;
        clock_t begin = clock();
        const boost::tuple<std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D> > f_result = f_sim.Evaluate();
        clock_t end = clock();
        double simulated_time = boost::get<0>(f_result).back();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        std::cout << simulated_time << " seconds simulation time took " << elapsed_secs << " real time to compute (x" << simulated_time/elapsed_secs << ")." << std::endl;

        const std::vector<Vector3D> &f_positions = boost::get<1>(f_result);
        const std::vector<Vector3D> &f_heights = boost::get<2>(f_result);
        const Vector3D f_pos = f_positions.back();

        a_sim = AdaptiveSimulation(random_seed);
        std::cout << "running adaptive ... " << std::endl;
        begin = clock();
        const boost::tuple<std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D> > a_result = a_sim.Evaluate();
        end = clock();
        simulated_time = boost::get<0>(a_result).back();
        elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        std::cout << simulated_time << " seconds simulation time took " << elapsed_secs << " real time to compute (x" << simulated_time/elapsed_secs << ")." << std::endl;

        const std::vector<Vector3D> &a_positions = boost::get<1>(a_result);
        const Vector3D a_pos = a_positions.back();


        error += VectorNorm(VectorSub(a_pos, f_pos));
    }
    std::cout << "error: " << error / (double) num_tests << std::endl;
    std::cout << "done" << std::endl;

    */
    return 0;
}

