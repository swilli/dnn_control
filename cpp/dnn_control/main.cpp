#include "configuration.h"
#include "pagmosimulationneuralnetwork.h"
#include "pagmosimulationproportionalderivative.h"

#include "filewriter.h"
#include "sensordatagenerator.h"

#include "evolutionaryrobotics.h"

#include "leastsquarespolicyrobotics.h"

int main(int argc, char *argv[]) {
    srand(time(0));

    TestNeuralNetworkController(0);
    return 0;

    //TestFullStateController(0);
    //return 0;

    //TestNeuralNetworkVSFullStateController(9782);
    //return 0;

    //TestLeastSquaresPolicyController(9782);
    //return 0;

    //TrainNeuralNetworkController();
    //return 0;

    //TrainProportionalDerivativeController();
    //return 0;

    //TrainLeastSquaresPolicyController();
    //return 0;

    //SensorDataGenerator generator(PATH_TO_SENSOR_DATA_FOLDER, 24.0 * 60.0 * 60.0);
    //generator.Generate(100, rand());
    //return 0;


    /*
    PaGMOSimulationFullState sim(1990, 24.0 * 60.0 * 60.0);
    const boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > result = sim.EvaluateAdaptive();
    const std::vector<Vector3D> &pos = boost::get<2>(result);
    const std::vector<Vector3D> &hei = boost::get<3>(result);

    FileWriter writer(PATH_TO_FULL_STATE_TRAJECTORY_FILE);
    writer.CreateVisualizationFile(sim.ControlFrequency(), sim.AsteroidOfSystem(), pos, hei);

    return 0;
    */


    /*
    const unsigned int num_tests = 100;
    double t_adapt = 0.0;
    double s_adapt = 0.0;
    double t_fixed = 0.0;
    double s_fixed = 0.0;
    double error_a_fi = 0.0;
    double sim_time = 0.0;
    const pagmo::decision_vector &task_41_solution = {-0.2653449146, 0.8221059793, 7.115913466, 0.9384741461, 2.460804959, 12.3201749, 1.871427235, -0.07533608116, -8.801983599, 1.642141599, -0.4937348003, -11.74057504, 3.7529415, 5.392689477, -7.77324484, -1.770696028, -1.183963091, -5.650056297, 1.019963074, 1.392104027, 1.926941641, 2.285940845, -1.079293945, -1.105212255, -3.894913852, -1.532887634, 2.790675652, -2.038766662, -0.7017624366, -2.263868159, -1.198603142, -4.826437788, -3.92156976, -2.00905927, -15.71152901, -2.343697858, -1.634728259, -4.242889368, -11.67533841, 7.999951956, 4.194276093, 7.023201787, 2.993808039, 2.996861109, -7.245632379, 1.343217234, -0.08975019422, -2.389846969, 0.25375688, -4.253339765, 7.06271584, 1.547862271, -0.7536306029, 1.070464669, -0.8058834915, -1.815555643, 2.866735509, -1.269174723, 1.178642348, -4.484382836, -0.6004458266, -6.729774456, -0.7779761483};
    for (unsigned int i = 0; i < num_tests; ++i) {
        const unsigned seed = rand();
        std::cout << "seed for round " << i << " is " << seed << std::endl;
        PaGMOSimulationNeuralNetwork sim(seed,  1.0 * 60.0 * 60.0, 6, task_41_solution);


        // Adaptive bucket
        std::cout << "adaptive" << std::endl;
        clock_t begin = clock();
        boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>  > result = sim.EvaluateAdaptive();
        clock_t end = clock();
        double simulated_time = boost::get<0>(result).back();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        double speedup = simulated_time/elapsed_secs;
        const std::vector<Vector3D> p_adapt = boost::get<2>(result);
        t_adapt += elapsed_secs;
        s_adapt += speedup;

        // Fixed bucket
        std::cout << "fixed" << std::endl;
        begin = clock();
        result = sim.EvaluateFixed();
        end = clock();
        simulated_time = boost::get<0>(result).back();
        elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        speedup = simulated_time/elapsed_secs;
        const std::vector<Vector3D> p_fixed = boost::get<2>(result);
        t_fixed += elapsed_secs;
        s_fixed += speedup;

        const unsigned int fi_size = p_fixed.size();
        const unsigned int a_size = p_adapt.size();

        unsigned int min_size = fi_size;
        if (min_size > a_size) {
            min_size = a_size;
        }

        if (min_size != fi_size || min_size != a_size) {
            std::cout << "fi_size: " << fi_size << " a_size: " << a_size <<  std::endl;
        }

        double cur_error_a_fi = 0.0;
        for (unsigned int j  = 0; j < min_size;  ++j) {
            cur_error_a_fi += VectorNorm(VectorSub(p_adapt.at(j), p_fixed.at(j)));
        }
        cur_error_a_fi /= min_size;
        std::cout << "current errors: " << cur_error_a_fi << std::endl;
        error_a_fi += cur_error_a_fi;
    }
    std::cout << "mean real sim time: " << sim_time / num_tests << std::endl;
    std::cout << "mean sim time adapt: " << t_adapt / num_tests << std::endl;
    std::cout << "mean speedup adapt: " << s_adapt / num_tests << std::endl;
    std::cout << "mean sim time fixed: " << t_fixed / num_tests << std::endl;
    std::cout << "mean speedup fixed: " << s_fixed / num_tests << std::endl;
    std::cout << "mean error a-fi: " << error_a_fi / num_tests << std::endl;
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

    */

    /* Copy constructor & assignment operator test

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

    return 0;
}

