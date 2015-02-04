#ifndef CONFIGURATION_H
#define CONFIGURATION_H

/*
 * GLOBAL CONFIGURATION FILE FOR ALL AVAILABLE COMPILE TIME OPTIONS
 */

// Class hovering_problem configs
#define HP_OBJ_FUN_METHOD_1    1   // Compare start and ending position and velocity.
#define HP_OBJ_FUN_METHOD_2    2   // Compare mean distance to target point.
#define HP_OBJ_FUN_METHOD_3    3   // Compare mean distance to target point, also consider velocity.
#define HP_OBJ_FUN_METHOD_4    4   // Compare mean distance to target point, but don't take into consideration some amount of starting positions.
#define HP_OBJ_FUN_METHOD_5    5   // Compare mean distance to target point, but don't take into consideration some amount of starting positions. Additionally, take into consideration total fuel consumption.
#define HP_OBJ_FUN_METHOD_6    6   // Compare mean distance to target point, also consider velocity, but don't take into consideration some amount of starting positions.
#define HP_OBJ_FUN_METHOD_7    7   // Compare mean distance to target point, also consider velocity, punish later offsets more.

#define HP_OBJECTIVE_FUNCTION_METHOD  HP_OBJ_FUN_METHOD_3
#define HP_FIXED_SEED  1990

// Class ODESystem configs
#define ODES_FUEL_ENABLED   false

// Class PaGMOSimulation configs
#define PGMOS_IC_INERTIAL_ZERO_VELOCITY      0
#define PGMOS_IC_BODY_ZERO_VELOCITY          1
#define PGMOS_IC_INERTIAL_ORBITAL_VELOCITY   2

#define PGMOS_INITIAL_CONDITION_TYPE  PGMOS_IC_BODY_ZERO_VELOCITY

// Class SensorSimulatorNeuralNetwork configs
#define SSNN_WITH_VELOCITY  false

// Class ControllerNeuralNetwork configs
#define CNN_NN_TYPE_FFNN    0
#define CNN_NN_TYPE_ESRN    1
#define CNN_NN_TYPE_CTRNN   2

#define CNN_NEURAL_NETWORK_TYPE     CNN_NN_TYPE_FFNN
#define CNN_WITH_VELOCITY   false

// Class SensorSimulatorAutoencoder configs
#define SSA_DATA_DIMENSIONS  5
#define SSA_DATA_MULTIPLIER  3
#define SSA_DATA_HISTORY     3

#endif // CONFIGURATION_H
