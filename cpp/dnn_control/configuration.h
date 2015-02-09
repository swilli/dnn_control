#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <iostream>
#include <iomanip>

/*
 * GLOBAL CONFIGURATION FILE FOR ALL AVAILABLE COMPILE TIME OPTIONS
 */

// Evolutionary Robotics configs
#define ER_NUM_GENERATIONS  1000
#define ER_POPULATION_SIZE  100
#define ER_NUM_ISLANDS  4
#define ER_SIMULATION_TIME  1.0 * 60.0 * 60.0
#define ER_EVALUATIONS  4
#define ER_NUM_HIDDEN_NODES 6

// Class hovering_problem configs
#define HP_OBJ_FUN_METHOD_1    1   // Compare start and ending position and velocity.
#define HP_OBJ_FUN_METHOD_2    2   // Compare mean distance to target point.
#define HP_OBJ_FUN_METHOD_3    3   // Compare mean distance to target point, also consider velocity.
#define HP_OBJ_FUN_METHOD_4    4   // Compare mean distance to target point, but don't take into consideration some amount of starting positions.
#define HP_OBJ_FUN_METHOD_5    5   // Compare mean distance to target point, but don't take into consideration some amount of starting positions. Additionally, take into consideration total fuel consumption.
#define HP_OBJ_FUN_METHOD_6    6   // Compare mean distance to target point, also consider velocity, but don't take into consideration some amount of starting positions.
#define HP_OBJ_FUN_METHOD_7    7   // Compare mean distance to target point, also consider velocity, punish later offsets more.

#define HP_OBJECTIVE_FUNCTION_METHOD  HP_OBJ_FUN_METHOD_6
//#define HP_FIXED_SEED  1990

// Class ODESystem configs
#define ODES_FUEL_ENABLED   true

// Class PaGMOSimulation configs
#define PGMOS_IC_INERTIAL_ZERO_VELOCITY      0
#define PGMOS_IC_BODY_ZERO_VELOCITY          1
#define PGMOS_IC_INERTIAL_ORBITAL_VELOCITY   2
#define PGMOS_IC_BODY_RANDOM_VELOCITY        3

#define PGMOS_IC_VELOCITY_TYPE  PGMOS_IC_BODY_RANDOM_VELOCITY
#define PGMOS_IC_POSITION_OFFSET_ENABLED true

// Class SensorSimulatorNeuralNetwork configs
#define SSNN_WITH_VELOCITY  true

// Class ControllerNeuralNetwork configs
#define CNN_NN_TYPE_FFNN    0
#define CNN_NN_TYPE_ESRN    1
#define CNN_NN_TYPE_CTRNN   2

#define CNN_NEURAL_NETWORK_TYPE     CNN_NN_TYPE_FFNN
#define CNN_WITH_VELOCITY   true

// Class SensorSimulatorAutoencoder configs
#define SSA_DATA_DIMENSIONS  5
#define SSA_DATA_MULTIPLIER  3
#define SSA_DATA_HISTORY     3

// Least Squares Policy Robotics configs
#define LSPR_NUM_SAMPLES    1000
#define LSPR_NUM_STEPS  50
#define LSPR_GAMMA  0.9
#define LSPR_EPSILON 1e-10

// Other stuff configs, not relevant for simulation
#define PATH_TO_NEURO_TRAJECTORY_FILE   "../../../results/trajectory_neuro.txt"
#define PATH_TO_LSPI_TRAJECTORY_FILE   "../../../results/trajectory_lspi.txt"
#define PATH_TO_NEURO_PERFORMANCE_FILE  "../../../results/performance_neuro.txt"
#define PATH_TO_SENSOR_DATA_FOLDER  "/home/willist/Documents/data/"

#ifdef HP_FIXED_SEED
#undef ER_EVALUATIONS
#define ER_EVALUATIONS  1
#endif

static inline std::string ToString(const bool &value) {
    return (value ? "true" : "false");
}

inline void Configuration() {
    std::cout << "PaGMOSimulation global configuration" << std::endl;
    std::cout << "ER_NUM_GENERATIONS   " << ER_NUM_GENERATIONS << std::endl;
    std::cout << "ER_POPULATION_SIZE   " << ER_POPULATION_SIZE << std::endl;
    std::cout << "ER_NUM_ISLANDS   " << ER_NUM_ISLANDS << std::endl;
    std::cout << "ER_SIMULATION_TIME   " << ER_SIMULATION_TIME << std::endl;
    std::cout << "ER_NUM_HIDDEN_NODES   " << ER_NUM_HIDDEN_NODES << std::endl;
    std::cout << "ER_EVALUATIONS   " << ER_EVALUATIONS << std::endl;
    std::cout << "HP_OBJECTIVE_FUNCTION_METHOD   " << HP_OBJECTIVE_FUNCTION_METHOD << std::endl;
#ifdef HP_FIXED_SEED
    std::cout << "HP_FIXED_SEED   " << HP_FIXED_SEED << std::endl;
#else
    std::cout << "HP_FIXED_SEED   undefined" << std::endl;
#endif
    std::cout << "ODES_FUEL_ENABLED   " << ToString(ODES_FUEL_ENABLED) << std::endl;
    std::cout << "PGMOS_IC_VELOCITY_TYPE   " << PGMOS_IC_VELOCITY_TYPE << std::endl;
    std::cout << "PGMOS_IC_POSITION_OFFSET_ENABLED   " << ToString(PGMOS_IC_POSITION_OFFSET_ENABLED) << std::endl;
    std::cout << "SSNN_WITH_VELOCITY   " << ToString(SSNN_WITH_VELOCITY) << std::endl;
    std::cout << "CNN_NEURAL_NETWORK_TYPE   " << CNN_NEURAL_NETWORK_TYPE << std::endl;
    std::cout << "CNN_WITH_VELOCITY   " << ToString(CNN_WITH_VELOCITY) << std::endl;
    std::cout << "SSA_DATA_DIMENSIONS   " << SSA_DATA_DIMENSIONS << std::endl;
    std::cout << "SSA_DATA_MULTIPLIER   " << SSA_DATA_MULTIPLIER << std::endl;
    std::cout << "SSA_DATA_HISTORY   " << SSA_DATA_HISTORY << std::endl;
    std::cout << "LSPR_NUM_SAMPLES   " << LSPR_NUM_SAMPLES << std::endl;
    std::cout << "LSPR_NUM_STEPS   " << LSPR_NUM_STEPS << std::endl;
    std::cout << "LSPR_GAMMA   " << LSPR_GAMMA << std::endl;
    std::cout << "LSPR_EPSILON   " << LSPR_EPSILON << std::endl;
    std::cout << std::endl;
}

#endif // CONFIGURATION_H
