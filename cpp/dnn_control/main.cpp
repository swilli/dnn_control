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

int main(int argc, char *argv[]) {
    srand(time(0));

    /* Copy constructor & assignment operator test */

    PaGMOSimulation s1(500);
    PaGMOSimulation s2(0);
    PaGMOSimulation s4(s2);
    {
        PaGMOSimulation s3(s2);
        s1 = s3;
    }

    const boost::tuple<std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D> > r1 = s1.EvaluateDetailed();
    const boost::tuple<std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D> > r2 = s2.EvaluateDetailed();
    const boost::tuple<std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D> > r4 = s4.EvaluateDetailed();

    const std::vector<Vector3D> &r1pos = boost::get<1>(r1);
    const std::vector<Vector3D> &r1hei = boost::get<2>(r1);

    const std::vector<Vector3D> &r2pos = boost::get<1>(r2);
    const std::vector<Vector3D> &r2hei = boost::get<2>(r2);

    const std::vector<Vector3D> &r4pos = boost::get<1>(r4);
    const std::vector<Vector3D> &r4hei = boost::get<2>(r4);

    const Vector3D r1p = r1pos.back();
    const Vector3D r2p = r2pos.back();
    const Vector3D r4p = r4pos.back();

    /*FileWriter writer;
    writer.CreateVisualizationFile("../../../results/visualization1.txt", 1.0 / s1.FixedStepSize(), s1.AsteroidOfSystem(), r1pos, r1hei);
    writer.CreateVisualizationFile("../../../results/visualization2.txt", 1.0 / s2.FixedStepSize(), s2.AsteroidOfSystem(), r2pos, r2hei);
    writer.CreateVisualizationFile("../../../results/visualization4.txt", 1.0 / s4.FixedStepSize(), s4.AsteroidOfSystem(), r4pos, r4hei);
    return 0;
    */

    return 0;

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



    /*PaGMOSimulation sim(0);
    const boost::tuple<std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D> > result1 = sim.Evaluate();
    const std::vector<Vector3D> &positions1 = boost::get<1>(result1);
    const std::vector<Vector3D> &heights1 = boost::get<2>(result1);

    const boost::tuple<std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D> > result2 = sim.Evaluate();
    const std::vector<Vector3D> &positions2 = boost::get<1>(result2);
    const std::vector<Vector3D> &heights2 = boost::get<2>(result2);

    FileWriter writer;
    writer.CreateVisualizationFile("../../../results/visualization1.txt", 1.0 / sim.FixedStepSize(), sim.AsteroidOfSystem(), positions1, heights1);
    writer.CreateVisualizationFile("../../../results/visualization2.txt", 1.0 / sim.FixedStepSize(), sim.AsteroidOfSystem(), positions2, heights2);
    */
    return 0;
}

