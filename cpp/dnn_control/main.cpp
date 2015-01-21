#include "adaptivesimulation.h"
#include "fixedsimulation.h"
#include "pagmosimulation.h"
#include "filewriter.h"
#include "samplefactory.h"

#include <pagmo/pagmo.h>

#define FULL_STATE_CONTROLLED                                                 0

#define GENERATE_SENSOR_DATA_FILES                                       0
#define NUM_SENSOR_DATA_FILES                                                 5
#define PATH_TO_SENSOR_DATA_FOLDER                                      "../../../data/"

#define CREATE_RANDOM_VISUALIZATION_FILE                            1
#define PATH_TO_RANDOM_VISUALIZATION_FILE                          "../../../results/visualization.txt"

static const std::vector<double> weights = {-23.45841816, 9.881433892, 1.516083635, -74.11891075, -38.42983259, -1.575379179, -20.76591088, 66.82263867, 7.921103634, -20.3435389, 31.70442149, 91.83254096, 55.86705557, -15.0762931, -2.850861174, 52.58192286, 12.65944449, 36.95860941, -91.46970756, 52.86597074, 63.43674248, -54.29699267, -14.93516363, -1.953594626, 45.63298073, 64.01985716, 14.180647, -39.51257528, 34.47074147, -60.18534684, 64.34163089, 54.35739281, -39.57683858, -83.61659867, 3.829107059, -20.02412051, 18.42098687, -9.503127714, -82.73239209, 43.24298386, 16.22721733, -11.04838993, 16.09469232, -27.17480605, 40.90291884, 26.4903342, 27.58933777, -8.957763676, 67.2869341, 57.01224964, 3.492742854, -1.960538951, -58.66287606, 35.30659259, 23.00837974, 0.05132899262, 22.24790601, -47.9068869, -37.06097622, 17.07439659, -26.80198811, -76.78152568, -34.24640262, 8.883126443, 77.84015364, -18.21173972, 12.0000837, 40.69885445, 28.50366755, -39.65585556, 53.26963241, 71.95669826, 34.62960199, -62.68444345, -1.463401553, -15.44031379, 36.18374303, 46.89547429, -22.09638049, -19.48470638, 13.07737073, -34.43625988, 30.24909588, -62.33570535, 7.86393044, -38.6486284, 8.586908311, -33.37221824, 33.41633356, 24.53958847, 24.507258, -10.98672868, -21.18266397, -23.728503, -0.5387350291, 68.56486363, -17.26540254, 5.549785122, 7.742460267, 43.40780826, 6.748598031, -66.16455101, 35.01264596, 10.71621233, -15.28572622, -17.90194928, 19.43840045, -43.84797592};



int main(int argc, char *argv[]) {
    srand(time(0));



    PaGMOSimulation sim(rand(), 86400.0, weights);
    const boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > r1 = sim.EvaluateDetailed();
    const std::vector<Vector3D> &r1pos = boost::get<2>(r1);
    const std::vector<Vector3D> &r1hei = boost::get<3>(r1);
    const std::vector<Vector3D> &r1omg = boost::get<5>(r1);

    FileWriter writer;
    writer.CreateVisualizationFile(PATH_TO_RANDOM_VISUALIZATION_FILE, 1.0 / sim.FixedStepSize(), sim.AsteroidOfSystem(), r1pos, r1hei);

    /*

    const unsigned int num_tests = 1000;
    double t = 0.0;
    const unsigned int dim = PaGMOSimulation(rand()).ControllerNeuralNetworkSize();
    for (unsigned int i = 0; i < num_tests; ++i) {
        const clock_t begin = clock();
        std::vector<double> x(dim, 0.0);
        for (unsigned int j = 0; j < dim; ++j) {
            x[j] = (double) rand() / (double) INT_MAX;
        }
        PaGMOSimulation p_sim(rand(), x);
        const boost::tuple<std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<double> > a_result = p_sim.Evaluate();
        const clock_t end = clock();
        const double simulated_time = boost::get<0>(a_result).back();
        const double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        t += elapsed_secs;
        std::cout << simulated_time << " seconds simulation time took " << elapsed_secs << " real time to compute (x" << simulated_time/elapsed_secs << ")." << std::endl;
    }
    std::cout << "mean: " << t / num_tests << std::endl;
    return 0;


    /* Copy constructor & assignment operator test */

    /*PaGMOSimulation s1(500);
    PaGMOSimulation s2(0);
    s2.Evaluate();
    PaGMOSimulation s4(s2);
    {
        PaGMOSimulation s3(0);
        s1 = s3;
    }

    const boost::tuple<std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<double> > r1 = s1.EvaluateDetailed();
    const boost::tuple<std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<double> > r2 = s2.EvaluateDetailed();
    const boost::tuple<std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<double> > r4 = s4.EvaluateDetailed();

    const std::vector<Vector3D> &r1pos = boost::get<1>(r1);
    const std::vector<Vector3D> &r1hei = boost::get<2>(r1);

    const std::vector<Vector3D> &r2pos = boost::get<1>(r2);
    const std::vector<Vector3D> &r2hei = boost::get<2>(r2);

    const std::vector<Vector3D> &r4pos = boost::get<1>(r4);
    const std::vector<Vector3D> &r4hei = boost::get<2>(r4);

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

    PaGMOSimulation p_sim(0);
    for (unsigned int i = 0; i < num_tests; ++i) {
        const unsigned int random_seed = rand();
        std::cout << "test " << (i + 1) << ", current seed: " << random_seed << std::endl;

        p_sim = PaGMOSimulation(random_seed);
        p_sim = p_sim;
        std::cout << "running fixed ... " << std::endl;
        clock_t begin = clock();
        const boost::tuple<std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D> > f_result = p_sim.EvaluateDetailed();
        clock_t end = clock();
        double f_simulated_time = boost::get<0>(f_result).back();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        std::cout << f_simulated_time << " seconds simulation time took " << elapsed_secs << " real time to compute (x" << f_simulated_time/elapsed_secs << ")." << std::endl;

        const std::vector<double> &f_times = boost::get<0>(f_result);
        const std::vector<Vector3D> &f_positions = boost::get<1>(f_result);
        const std::vector<Vector3D> &f_heights = boost::get<2>(f_result);
        const Vector3D f_pos = f_positions.back();

        std::cout << "running adaptive ... " << std::endl;
        begin = clock();
        const boost::tuple<std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D> > a_result = p_sim.Evaluate();
        end = clock();
        double a_simulated_time = boost::get<0>(a_result).back();
        elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        std::cout << a_simulated_time << " seconds simulation time took " << elapsed_secs << " real time to compute (x" << a_simulated_time/elapsed_secs << ")." << std::endl;

        const std::vector<double> &a_times = boost::get<0>(a_result);
        const std::vector<Vector3D> &a_positions = boost::get<1>(a_result);
        const Vector3D a_pos = a_positions.back();


        error += VectorNorm(VectorSub(a_pos, f_pos));
    }
    std::cout << "error: " << error / (double) num_tests << std::endl;
    std::cout << "done" << std::endl;

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

