#include "adaptivesimulation.h"
#include "fixedsimulation.h"
#include "filewriter.h"

#include <pagmo/pagmo.h>

#define FULL_STATE_CONTROLLED                                                 0

#define GENERATE_SENSOR_DATA_FILES                                       0
#define NUM_SENSOR_DATA_FILES                                                 5
#define PATH_TO_SENSOR_DATA_FOLDER                                      "../../../data/"

#define CREATE_RANDOM_VISUALIZATION_FILE                            1
#define PATH_TO_RANDOM_VISUALIZATION_FILE                          "../../../results/visualization.txt"

int main(int argc, char *argv[]) {
    srand(time(0));

    FixedSimulation f_sim(rand());
    std::cout << "running fixed ... " << std::endl;
    clock_t begin = clock();
    const boost::tuple<std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D> > f_result = f_sim.Evaluate();
    clock_t end = clock();
    double simulated_time = boost::get<0>(f_result).back();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << simulated_time << " seconds simulation time took " << elapsed_secs << " real time to compute (x" << simulated_time/elapsed_secs << ")." << std::endl;

    const std::vector<Vector3D> &f_positions = boost::get<1>(f_result);
    const std::vector<Vector3D> &f_heights = boost::get<2>(f_result);

    FileWriter writer;
    writer.CreateVisualizationFile(PATH_TO_RANDOM_VISUALIZATION_FILE, 1.0 / f_sim.FixedStepSize(), f_sim.AsteroidOfSystem(), f_positions, f_heights);
    exit(0);



    /*double error = 0.0;
    const unsigned int num_tests = 1000;
    for (unsigned int i = 0; i < num_tests; ++i) {
        const unsigned int random_seed = rand();
        std::cout << "test " << (i + 1) << ", current seed: " << random_seed << std::endl;

        AdaptiveSimulation a_sim(random_seed);
        FixedSimulation f_sim(random_seed);

        std::cout << "running adaptive ... " << std::endl;
        clock_t begin = clock();
        const boost::tuple<std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D> > a_result = a_sim.Evaluate();
        clock_t end = clock();
        double simulated_time = boost::get<0>(a_result).back();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        std::cout << simulated_time << " seconds simulation time took " << elapsed_secs << " real time to compute (x" << simulated_time/elapsed_secs << ")." << std::endl;

        const std::vector<Vector3D> &a_positions = boost::get<1>(a_result);
        const Vector3D a_pos = a_positions.back();


        std::cout << "running fixed ... " << std::endl;
        begin = clock();
        const boost::tuple<std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D> > f_result = f_sim.Evaluate();
        end = clock();
        simulated_time = boost::get<0>(f_result).back();
        elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        std::cout << simulated_time << " seconds simulation time took " << elapsed_secs << " real time to compute (x" << simulated_time/elapsed_secs << ")." << std::endl;

        const std::vector<Vector3D> &f_positions = boost::get<1>(f_result);
        const std::vector<Vector3D> &f_heights = boost::get<2>(f_result);
        const Vector3D f_pos = f_positions.back();

        error += VectorNorm(VectorSub(a_pos, f_pos));
    }
    std::cout << "error: " << error / (double) num_tests << std::endl;
    std::cout << "done" << std::endl;
    */

}

