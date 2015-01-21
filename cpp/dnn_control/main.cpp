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

static const std::vector<double> weights = {43.9777381, 7.790147644, -46.03367053, 8.472116331, -6.540703234, -25.17279439, -54.62108151, 32.84404501, 22.28317739, 28.18680844, 70.56850145, -3.455183223, 68.06356974, 15.63936738, -47.93488463, 1.012026265, -3.359601348, 24.69321623, -22.62783262, 16.4700738, -8.86097869, 20.05772462, -25.55834141, 0.7551885126, 33.73260706, -45.40106986, -82.62362607, 10.94507782, 23.1903436, 31.07965466, 59.81531571, -54.46535077, -24.14960921, 16.55465426, -37.41702812, 41.14145006, -72.43622751, 34.17680551, 50.86402751, 48.43881957, 1.374996017, 14.76514405, 30.59705202, 23.72827023, -14.95927914, -18.05289073, -17.48653955, 31.19970141, -24.19757218, 15.11896867, 2.674090929, -3.38367809, 24.00465369, -36.49751495, -38.21373213, 50.89167418, -83.62861425, 47.80219177, 24.4605401, -8.983465051, 5.832324082, -29.27200923, -33.66491327, 20.68867005, -36.65737209, -10.50887239, 0.3990891023, 11.72340484, -10.54546159, 28.82811704, -16.07991016, 41.05847102, 12.11291604, -7.519902895, 55.20745744};

int main(int argc, char *argv[]) {
    srand(time(0));

    PaGMOSimulation sim(rand(), 86400.0); // weights);
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

