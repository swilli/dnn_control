#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <iostream>
#include <iomanip>

/*
 * GLOBAL CONFIGURATION FILE FOR ALL AVAILABLE COMPILE TIME OPTIONS
 */

// Task Name
#define TASK_NAME   "master"

// Euler home or not
#define ENABLE_EULER_HOME   false

// Evolutionary Robotics configs
#define ER_NUM_GENERATIONS  1000
#define ER_POPULATION_SIZE  20
#define ER_NUM_ISLANDS  24
//#define ER_SIMULATION_TIME  3.0 * 60.0 * 60.0
#define ER_EVALUATIONS  10
#define ER_NUM_HIDDEN_NODES 6


// Class hovering_problem configs
#define HP_OBJ_FUN_METHOD_1     1   // Compare start and ending position and velocity.
#define HP_OBJ_FUN_METHOD_2     2   // Compare mean distance to target point. Transient response aware.
#define HP_OBJ_FUN_METHOD_3     3   // Compare mean distance to target point, also consider velocity. Transient response aware.
#define HP_OBJ_FUN_METHOD_4     4   // Compare mean distance to target point, also consider fuel consumption. Transient response aware.
#define HP_OBJ_FUN_METHOD_5     5   // Compare mean distance to target point, , also consider velocity, also consider fuel consumption. Transient response aware.
#define HP_OBJ_FUN_METHOD_6     6   // Mean velocity. Transient response aware.
#define HP_OBJ_FUN_METHOD_7     7   // Mean optical flow, constant divergence. Transient response aware.
#define HP_OBJ_FUN_METHOD_8     8   // Mean offset to optimal landing path.
#define HP_OBJ_FUN_METHOD_9     9   // Mean distance to target point, target point set to position after the deep controller starts to work.

#define HP_OBJECTIVE_FUNCTION_METHOD  HP_OBJ_FUN_METHOD_3

#define HP_OBJ_FUN_TRANSIENT_RESPONSE_TIME  150.0
#define HP_OBJ_FUN_COEF_DIVERGENCE  0.0001
#define HP_OBJ_FUN_PUNISH_UNFINISHED_SIMULATIONS_ENABLED    true


#define HP_POST_EVAL_METHOD_1   1   // Compare mean distance to target point. Transient response aware.
#define HP_POST_EVAL_METHOD_2   2   // Compare mean velocity. Transient response aware.
#define HP_POST_EVAL_METHOD_3   3   // Compare mean distance to target path.

#define HP_POST_EVALUATION_METHOD   HP_POST_EVAL_METHOD_1

//#define HP_FIXED_SEED  1990


// Class ODESystem configs
#define ENABLE_ODES_FUEL   true


// Class PaGMOSimulation configs
#define PGMOS_IC_INERTIAL_ZERO_VELOCITY      0
#define PGMOS_IC_INERTIAL_ORBITAL_VELOCITY   1
#define PGMOS_IC_BODY_ZERO_VELOCITY          2
#define PGMOS_IC_BODY_RANDOM_VELOCITY        3
#define PGMOS_IC_BODY_PROPORTIONAL_VELOCITY 4

#define PGMOS_IC_VELOCITY_TYPE  PGMOS_IC_INERTIAL_ORBITAL_VELOCITY
#define PGMOS_IC_ENABLE_POSITION_OFFSET    false
#define PGMOS_ENABLE_RELATIVE_POSITION  false
#define PGMOS_ENABLE_VELOCITY   false
#define PGMOS_ENABLE_OPTICAL_FLOW   false
#define PGMOS_ENABLE_ACCELEROMETER  false
#define PGMOS_ENABLE_NOISE false
#define PGMOS_STANDARDIZE_SENSOR_VALUES    false
#define PGMOS_ENABLE_SENSOR_DATA_RECORDING  true


// Class ControllerNeuralNetwork configs
#define CNN_ENABLE_STACKED_AUTOENCODER  false
#define CNN_STACKED_AUTOENCODER_CONFIGURATION   "100_50_7"

// Class SensorSimulatorPartialState configs




// Least Squares Policy Robotics configs
#define LSPR_IC_BODY_ZERO_VELOCITY          0
#define LSPR_IC_BODY_RANDOM_VELOCITY        1

#define LSPR_IC_VELOCITY_TYPE  LSPR_IC_BODY_RANDOM_VELOCITY
#define LSPR_IC_POSITION_OFFSET_ENABLED true
//#define LSPR_FIXED_SEED     1990

#define LSPR_DIRECTION_RESOLUTION   5
#define LSPR_THRUST_RESOLUTION  7
#define LSPR_TRANSIENT_RESPONSE_TIME  100.0
#define LSPR_REWARD_WITH_VELOCITY   true
#define LSPR_NUM_EPISODES    1000
#define LSPR_NUM_STEPS  50
#define LSPR_GAMMA  0.9
#define LSPR_EPSILON 1e-10
#define LSPR_WRITE_ACTION_SET_TO_FILE   true

// Other stuff configs, not relevant for simulation
#if ENABLE_EULER_HOME
#define OUTPUT_ROOT_PATH   "/cluster/home/willist/"
#else
#define OUTPUT_ROOT_PATH   "/home/willist/Documents/dnn/"
#endif
#define PATH_TO_NEURO_TRAJECTORY_FILE   OUTPUT_ROOT_PATH    "results/trajectory_neuro_" TASK_NAME ".txt"
#define PATH_TO_NEURO_EVALUATION_FILE  OUTPUT_ROOT_PATH "results/evaluation_neuro_" TASK_NAME ".txt"
#define PATH_TO_NEURO_POST_EVALUATION_FILE    OUTPUT_ROOT_PATH  "results/post_evaluation_neuro_" TASK_NAME ".txt"
#define PATH_TO_NEURO_CONVEXITY_PATH    OUTPUT_ROOT_PATH    "results/convexity/"
#define PATH_TO_PROPORTIONAL_DERIVATIVE_TRAJECTORY_FILE  OUTPUT_ROOT_PATH    "results/trajectory_pd_" TASK_NAME ".txt"
#define PATH_TO_PROPORTIONAL_DERIVATIVE_EVALUATION_FILE  OUTPUT_ROOT_PATH    "results/evaluation_pd_" TASK_NAME ".txt"
#define PATH_TO_PROPORTIONAL_DERIVATIVE_POST_EVALUATION_FILE     OUTPUT_ROOT_PATH "results/post_evaluation_pd_" TASK_NAME ".txt"
#define PATH_TO_LSPI_TRAJECTORY_FILE   OUTPUT_ROOT_PATH "results/trajectory_lspi.txt"
#define PATH_TO_LSPI_EVALUATION_FILE  OUTPUT_ROOT_PATH "results/evaluation_lspi.txt"
#define PATH_TO_LSPI_POST_EVALUATION_FILE   OUTPUT_ROOT_PATH "results/post_evaluation_lspi.txt"
#define PATH_TO_LSPI_ACTION_SET_FILE    OUTPUT_ROOT_PATH "results/lspi_action_set.txt"
#define PATH_TO_LSPI_WEIGHT_VECTOR_FILE OUTPUT_ROOT_PATH "results/lspi_weights.txt"
#define PATH_TO_COMPARISON_POST_EVALUATION_FILE     OUTPUT_ROOT_PATH "results/post_evaluation_comparison.txt"
#define PATH_TO_SENSOR_DATA_FOLDER  OUTPUT_ROOT_PATH    "data/raw/"
#define PATH_TO_AUTOENCODER_LAYER_CONFIGURATION OUTPUT_ROOT_PATH "autoencoder/conf_" CNN_STACKED_AUTOENCODER_CONFIGURATION "_" TASK_NAME "/"

#ifdef HP_FIXED_SEED
#undef ER_EVALUATIONS
#define ER_EVALUATIONS  1
#endif

static inline std::string ToString(const bool &value) {
    return (value ? "true" : "false");
}

inline void ConfigurationPaGMO() {
    std::cout << TASK_NAME << std::endl;
    std::cout << "PaGMOSimulation global configuration" << std::endl;
    std::cout << "ER_NUM_ISLANDS   " << ER_NUM_ISLANDS << std::endl;
    std::cout << "ER_POPULATION_SIZE   " << ER_POPULATION_SIZE << std::endl;
    std::cout << "ER_EVALUATIONS   " << ER_EVALUATIONS << std::endl;
    std::cout << "ER_NUM_GENERATIONS   " << ER_NUM_GENERATIONS << std::endl;
#ifdef ER_SIMULATION_TIME
    std::cout << "ER_SIMULATION_TIME   " << ER_SIMULATION_TIME << std::endl;
#else
    std::cout << "ER_SIMULATION_TIME   dynamic" << std::endl;
#endif
    std::cout << "ER_NUM_HIDDEN_NODES   " << ER_NUM_HIDDEN_NODES << std::endl;
#ifdef HP_FIXED_SEED
    std::cout << "HP_FIXED_SEED   " << HP_FIXED_SEED << std::endl;
#else
    std::cout << "HP_FIXED_SEED   undefined" << std::endl;
#endif
    std::cout << "HP_OBJECTIVE_FUNCTION_METHOD   " << HP_OBJECTIVE_FUNCTION_METHOD << std::endl;
    std::cout << "HP_OBJ_FUN_PUNISH_UNFINISHED_SIMULATIONS_ENABLED   " << ToString(HP_OBJ_FUN_PUNISH_UNFINISHED_SIMULATIONS_ENABLED) << std::endl;
    std::cout << "HP_OBJ_FUN_TRANSIENT_RESPONSE_TIME   " << HP_OBJ_FUN_TRANSIENT_RESPONSE_TIME << std::endl;
    std::cout << "HP_OBJ_FUN_COEF_DIVERGENCE   " << HP_OBJ_FUN_COEF_DIVERGENCE << std::endl;
    std::cout << "HP_POST_EVALUATION_METHOD   " << HP_POST_EVALUATION_METHOD << std::endl;
    std::cout << "PGMOS_IC_VELOCITY_TYPE   " << PGMOS_IC_VELOCITY_TYPE << std::endl;
    std::cout << "PGMOS_IC_ENABLE_POSITION_OFFSET   " << ToString(PGMOS_IC_ENABLE_POSITION_OFFSET) << std::endl;
    std::cout << "PGMOS_ENABLE_RELATIVE_POSITION   " << ToString(PGMOS_ENABLE_RELATIVE_POSITION) << std::endl;
    std::cout << "PGMOS_ENABLE_VELOCITY   " << ToString(PGMOS_ENABLE_VELOCITY) << std::endl;
    std::cout << "PGMOS_ENABLE_OPTICAL_FLOW   " << ToString(PGMOS_ENABLE_OPTICAL_FLOW) << std::endl;
    std::cout << "PGMOS_ENABLE_ACCELEROMETER   " << ToString(PGMOS_ENABLE_ACCELEROMETER) << std::endl;
    std::cout << "PGMOS_ENABLE_NOISE   " << ToString(PGMOS_ENABLE_NOISE) << std::endl;
    std::cout << "PGMOS_STANDARDIZE_SENSOR_VALUES   " << ToString(PGMOS_STANDARDIZE_SENSOR_VALUES) << std::endl;
    std::cout << "PGMOS_ENABLE_SENSOR_DATA_RECORDING   " << ToString(PGMOS_ENABLE_SENSOR_DATA_RECORDING) << std::endl;
    std::cout << "ENABLE_ODES_FUEL   " << ToString(ENABLE_ODES_FUEL) << std::endl;
    std::cout << "CNN_ENABLE_STACKED_AUTOENCODER   " << ToString(CNN_ENABLE_STACKED_AUTOENCODER) << std::endl;
    std::cout << "CNN_STACKED_AUTOENCODER_CONFIGURATION   " << CNN_STACKED_AUTOENCODER_CONFIGURATION << std::endl;
    std::cout << std::endl;
}

inline void ConfigurationLSPI() {
    std::cout << "LSPI global configuration" << std::endl;
    std::cout << "LSPR_NUM_EPISODES   " << LSPR_NUM_EPISODES << std::endl;
    std::cout << "LSPR_NUM_STEPS   " << LSPR_NUM_STEPS << std::endl;
    std::cout << "LSPR_DIRECTION_RESOLUTION   " << LSPR_DIRECTION_RESOLUTION << std::endl;
    std::cout << "LSPR_THRUST_RESOLUTION   " << LSPR_THRUST_RESOLUTION << std::endl;
    std::cout << "LSPR_TRANSIENT_RESPONSE_TIME   " << LSPR_TRANSIENT_RESPONSE_TIME << std::endl;
    std::cout << "LSPR_REWARD_WITH_VELOCITY   " << ToString(LSPR_REWARD_WITH_VELOCITY) << std::endl;
    std::cout << "LSPR_GAMMA   " << LSPR_GAMMA << std::endl;
    std::cout << "LSPR_EPSILON   " << LSPR_EPSILON << std::endl;
#ifdef LSPR_FIXED_SEED
    std::cout << "LSPR_FIXED_SEED   " << LSPR_FIXED_SEED << std::endl;
#else
    std::cout << "LSPR_FIXED_SEED   undefined" << std::endl;
#endif
    std::cout << "LSPR_IC_POSITION_OFFSET_ENABLED   " << ToString(LSPR_IC_POSITION_OFFSET_ENABLED) << std::endl;
    std::cout << "LSPR_IC_VELOCITY_TYPE   " << LSPR_IC_VELOCITY_TYPE << std::endl;
    std::cout << "LSPR_WRITE_ACTION_SET_TO_FILE   " << ToString(LSPR_WRITE_ACTION_SET_TO_FILE) << std::endl;
    std::cout << std::endl;
}

#endif // CONFIGURATION_H
