cmake_minimum_required (VERSION 2.8.11)
project(dnn_control)

# 1 - Preliminary checks and messages
MESSAGE(STATUS "OS detected: ${CMAKE_SYSTEM_NAME}")
MESSAGE(STATUS "CXX Compiler detected: ${CMAKE_CXX_COMPILER_ID}")

# 2 - Setting compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -std=c++11 -pthread -DNDEBUG")

# 3 - Find Third-party Libraries
# GSL
FIND_LIBRARY(GSL_GSL_LIBRARY NAMES gsl)
FIND_LIBRARY(GSL_GSLCBLAS_LIBRARY NAMES gslcblas)

# PaGMO
FIND_LIBRARY(PAGMO_LIBRARY NAMES pagmo)

# Boost
FIND_LIBRARY(BOOST_SYSTEM_LIBRARY NAMES boost_system)
FIND_LIBRARY(BOOST_PYTHON_LIBRARY NAMES boost_python3)

# F2C
FIND_LIBRARY(F2C_LIBRARY NAMES f2c)

FIND_PATH(GSL_INCLUDE_DIR NAMES gsl/gsl_version.h)
FIND_PATH(PAGMO_INCLUDE_DIR NAMES pagmo/src/pagmo.h)

MESSAGE(STATUS "GSL library: ${GSL_GSL_LIBRARY}")
MESSAGE(STATUS "GSL CBLAS library: ${GSL_GSLCBLAS_LIBRARY}")
MESSAGE(STATUS "GSL include dir: ${GSL_INCLUDE_DIR}")
MESSAGE(STATUS "BOOST SYSTEM library: ${BOOST_SYSTEM_LIBRARY}")
MESSAGE(STATUS "BOOST PYTHON library: ${BOOST_PYTHON_LIBRARY}")
MESSAGE(STATUS "F2C library: ${F2C_LIBRARY}")
MESSAGE(STATUS "PAGMO library: ${PAGMO_LIBRARY}")
MESSAGE(STATUS "PAGMO include dir: ${PAGMO_INCLUDE_DIR}")

IF(NOT GSL_GSL_LIBRARY OR NOT GSL_INCLUDE_DIR)
	MESSAGE(FATAL_ERROR "GSL support was requested, but either the library or the include files could not be located.")
ENDIF(NOT GSL_GSL_LIBRARY OR NOT GSL_INCLUDE_DIR)
IF(NOT GSL_GSLCBLAS_LIBRARY)
	MESSAGE(FATAL_ERROR  "GSL BLAS library not found")
ENDIF(NOT GSL_GSLCBLAS_LIBRARY)
IF(NOT BOOST_SYSTEM_LIBRARY)
	MESSAGE(FATAL_ERROR "BOOST SYSTEM library not found")
ENDIF(NOT BOOST_SYSTEM_LIBRARY)
IF(NOT BOOST_PYTHON_LIBRARY)
	MESSAGE(FATAL_ERROR "BOOST PYTHON library not found")
ENDIF(NOT BOOST_PYTHON_LIBRARY)
IF(NOT F2C_LIBRARY)
	MESSAGE(FATAL_ERROR "F2C library not found")
ENDIF(NOT F2C_LIBRARY)
IF(NOT PAGMO_LIBRARY OR NOT PAGMO_INCLUDE_DIR)
	MESSAGE(FATAL_ERROR "PaGMO installation not found")
ENDIF(NOT PAGMO_LIBRARY OR NOT PAGMO_INCLUDE_DIR)

# 4 - Define mandatory libraries and include directories
SET(MANDATORY_LIBRARIES ${MANDATORY_LIBRARIES} ${GSL_GSL_LIBRARY} ${GSL_GSLCBLAS_LIBRARY} ${BOOST_SYSTEM_LIBRARY} ${BOOST_PYTHON_LIBRARY} ${F2C_LIBRARY} ${PAGMO_LIBRARY})
INCLUDE_DIRECTORIES("${GSL_INCLUDE_DIR} ${PAGMO_INCLUDE_DIR}")

# 5 - Define the target and linked libraries
add_executable(main main.cpp asteroid.cpp pagmosimulation.cpp pagmosimulationfullstate.cpp pagmosimulationneuralnetwork.cpp odesystem.cpp filewriter.cpp samplefactory.cpp sensordatagenerator.cpp neuralnetwork.cpp feedforwardneuralnetwork.cpp sensorsimulator.cpp controller.cpp controllerfullstate.cpp controllerneuralnetwork.cpp sensorsimulatorfullstate.cpp sensorsimulatorneuralnetwork.cpp sensorsimulatorautoencoder.cpp hoveringproblemneuralnetwork.cpp hoveringproblemfullstate.cpp evolutionaryrobotics.cpp lspisimulator.cpp leastsquarespolicyrobotics.cpp) 

TARGET_LINK_LIBRARIES(main ${MANDATORY_LIBRARIES})

