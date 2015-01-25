#include "adaptivesimulation.h"
#include "fixedsimulation.h"
#include "pagmosimulationneuralnetwork.h"
#include "pagmosimulationfullstate.h"
#include "filewriter.h"
#include "samplefactory.h"

#include <pagmo/pagmo.h>

#define FULL_STATE_CONTROLLED                                                 0

#define GENERATE_SENSOR_DATA_FILES                                            0
#define NUM_SENSOR_DATA_FILES                                                 5
#define PATH_TO_SENSOR_DATA_FOLDER                                            "../../../data/"

#define CREATE_RANDOM_VISUALIZATION_FILE                                      1
#define PATH_TO_RANDOM_VISUALIZATION_FILE                                     "../../../results/visualization.txt"

static const std::vector<double> weights = {2.0265e-316, 6.9483e-310, -2.8118, 0.7502, 1.0965, -1.4687, 0.089297, 0.61147, -2.5362, -1.3803, -1.1204, -0.96183, -0.60804, 0.54146, -1.8596, 0.09163, 0.16093, 1.0272, -0.81006, -0.84041, -1.6257, 1.1091, 0.29541, 0.3173, -1.4341, -1.2566, -0.5318, -0.82395, -0.037266, 3.1756, 0.75006, 1.0821, 1.2641, 2.9441, 0.15292, 0.014959, 0.053071, 0.062749, 0.054537, -0.35081, 0.31024, 0.21875, 1.3239, -1.1156, 0.64877, 0.29212, 0.38048, 0.043356, 0.54404, 1.2707, -0.10552, -0.34132, -0.037371, 2.775, 0.28651, 0.087201, -0.01899, -0.98907, 1.3143, 0.061669, -2.5496, 0.81079, 0.29934, 0.98034, 0.28801, -0.56325, -0.73541, -0.39743, 3.0504, -2.9222, 1.5968, 0.2934, 4.5375e-320};

int main(int argc, char *argv[]) {
    srand(time(0));


    PaGMOSimulationNeuralNetwork sim(rand(), 86400.0, weights);
    const boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > r1 = sim.EvaluateDetailed();
    const std::vector<Vector3D> &r1pos = boost::get<2>(r1);
    const std::vector<Vector3D> &r1hei = boost::get<3>(r1);

    FileWriter writer;
    writer.CreateVisualizationFile(PATH_TO_RANDOM_VISUALIZATION_FILE, 1.0 / sim.FixedStepSize(), sim.AsteroidOfSystem(), r1pos, r1hei);

    return 0;



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
    PaGMOSimulationFullState sim(rand(), 86400.0, {0.0, 0.0, 0.0});
    const boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > result = sim.EvaluateDetailed();
    const std::vector<Vector3D> &pos = boost::get<2>(result);
    const std::vector<Vector3D> &hei = boost::get<3>(result);

    FileWriter writer;
    writer.CreateVisualizationFile(PATH_TO_RANDOM_VISUALIZATION_FILE, 1.0 / sim.FixedStepSize(), sim.AsteroidOfSystem(), pos, hei);


    return 0;

    */

    /*
    const unsigned int num_tests = 100;
    double t = 0.0;
    const unsigned int dim = PaGMOSimulationNeuralNetwork(rand(), 86400.0).ControllerNeuralNetworkSize();
    for (unsigned int i = 0; i < num_tests; ++i) {
        std::vector<double> x(dim, 0.0);
        for (unsigned int j = 0; j < dim; ++j) {
            x[j] = (double) rand() / (double) INT_MAX;
        }
        const clock_t begin = clock();
        PaGMOSimulationNeuralNetwork p_sim(rand(), 3.0 * 60.0 * 60.0, x);
        const boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > a_result = p_sim.Evaluate();
        const clock_t end = clock();
        const double simulated_time = boost::get<0>(a_result).back();
        const double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        t += elapsed_secs;
        std::cout << simulated_time << " seconds simulation time took " << elapsed_secs << " real time to compute (x" << simulated_time/elapsed_secs << ")." << std::endl;
    }
    std::cout << "mean: " << t / num_tests << std::endl;
    return 0;

    */

    /* Copy constructor & assignment operator test */

    /*

    PaGMOSimulationNeuralNetwork s1(500, 86400.0);
    PaGMOSimulationNeuralNetwork s2(0, 86400.0);
    s2.Evaluate();
    PaGMOSimulationNeuralNetwork s4(s2);
    {
        PaGMOSimulationNeuralNetwork s3(0, 86400.0);
        s1 = s3;
    }

    const boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > r1 = s1.EvaluateDetailed();
    const boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > r2 = s2.EvaluateDetailed();
    const boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > r4 = s4.EvaluateDetailed();

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

