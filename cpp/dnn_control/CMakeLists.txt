CMAKE_MINIMUM_REQUIRED(VERSION 2.8.11)
PROJECT(dnn_control)

# 1 - Preliminary checks and messages
MESSAGE(STATUS "OS detected: ${CMAKE_SYSTEM_NAME}")
MESSAGE(STATUS "CXX Compiler detected: ${CMAKE_CXX_COMPILER_ID}")

# 2 - Setting compiler flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -std=c++11 -pthread -DNDEBUG")

# 3 - Set compiler warning flags
IF(MSVC)
  # Force to always compile with W4
  IF(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
    STRING(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  ELSE()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
  ENDIF()
ELSEIF(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  # Update if necessary
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-long-long -pedantic")
ENDIF()

# 4 - Find Third-party Libraries
# GSL
FIND_LIBRARY(GSL_GSL_LIBRARY NAMES gsl)
FIND_LIBRARY(GSL_GSLCBLAS_LIBRARY NAMES gslcblas)
FIND_PATH(GSL_INCLUDE_DIR NAMES gsl/gsl_version.h)

# PaGMO
FIND_LIBRARY(PAGMO_LIBRARY NAMES pagmo)
FIND_PATH(PAGMO_INCLUDE_DIR NAMES pagmo/src/pagmo.h)

# Boost
FIND_PACKAGE(Boost 1.55.0 COMPONENTS system serialization thread filesystem)

# MPI
OPTION(ENABLE_MPI "ENABLE MPI" OFF)
MESSAGE(STATUS "ENABLE MPI: ${ENABLE_MPI}")
IF(ENABLE_MPI)
  FIND_PACKAGE(MPI REQUIRED)
ENDIF()

MESSAGE(STATUS "GSL library: ${GSL_GSL_LIBRARY}")
MESSAGE(STATUS "GSL include dir: ${GSL_INCLUDE_DIR}")
MESSAGE(STATUS "GSL CBLAS library: ${GSL_GSLCBLAS_LIBRARY}")
MESSAGE(STATUS "BOOST SYSTEM library: ${Boost_SYSTEM_LIBRARY}")
MESSAGE(STATUS "BOOST SERIALIZATION library: ${Boost_SERIALIZATION_LIBRARY}")
MESSAGE(STATUS "BOOST THREAD library: ${Boost_THREAD_LIBRARY}")
MESSAGE(STATUS "BOOST FILESYSTEM library: ${Boost_FILESYSTEM_LIBRARY}")
MESSAGE(STATUS "BOOST include dir: ${Boost_INCLUDE_DIRS}")
MESSAGE(STATUS "PaGMO library: ${PAGMO_LIBRARY}")
MESSAGE(STATUS "PaGMO include dir: ${PAGMO_INCLUDE_DIR}")
IF(ENABLE_MPI)
  MESSAGE(STATUS "MPI include dir: ${MPI_INCLUDE_PATH}")
  MESSAGE(STATUS "MPI library: ${MPI_LIBRARIES}")
ENDIF()

IF(NOT GSL_GSL_LIBRARY OR NOT GSL_INCLUDE_DIR OR NOT GSL_GSLCBLAS_LIBRARY)
  MESSAGE(FATAL_ERROR "GSL support was requested, but either the library or the include files could not be located.")
ENDIF()
IF(NOT Boost_INCLUDE_DIRS OR NOT Boost_SYSTEM_LIBRARY)
  MESSAGE(FATAL_ERROR "Boost support was requested, but either the library or the include files could not be located.")
ENDIF()
IF(NOT PAGMO_LIBRARY OR NOT PAGMO_INCLUDE_DIR)
  MESSAGE(FATAL_ERROR "PaGMO installation not found")
ENDIF()

# 5 - Define mandatory libraries and include directories
SET(MANDATORY_LIBRARIES ${MANDATORY_LIBRARIES} ${GSL_GSL_LIBRARY} ${GSL_GSLCBLAS_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_SERIALIZATION_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${PAGMO_LIBRARY})
IF(ENABLE_MPI)
  SET(MANDATORY_LIBRARIES ${MANDATORY_LIBRARIES} ${MPI_LIBRARIES})
ENDIF()

INCLUDE_DIRECTORIES(${GSL_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${PAGMO_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
IF(ENABLE_MPI)
  INCLUDE_DIRECTORIES(${MPI_INCLUDE_PATH})
ENDIF()

# 6 - Define the target and linked libraries
#OPTION(BUILD_WITH_LSPI "BUILD WITH LSPI" OFF)
#MESSAGE(STATUS "BUILD with LSPI: ${BUILD_WITH_LSPI}")


#IF(BUILD_WITH_LSPI)
add_executable(main main.cpp asteroid.cpp pagmosimulation.cpp pagmosimulationneuralnetwork.cpp odesystem.cpp filewriter.cpp samplefactory.cpp sensordatagenerator.cpp neuralnetwork.cpp feedforwardneuralnetwork.cpp simplerecurrentneuralnetwork.cpp stackedautoencoder.cpp sensorsimulator.cpp controller.cpp controllerneuralnetwork.cpp controllerdeepneuralnetwork.cpp hoveringproblemneuralnetwork.cpp evolutionaryrobotics.cpp lspisimulator.cpp leastsquarespolicyrobotics.cpp) 
#ELSE()
#add_executable(main main.cpp asteroid.cpp pagmosimulation.cpp pagmosimulationneuralnetwork.cpp odesystem.cpp filewriter.cpp samplefactory.cpp sensordatagenerator.cpp neuralnetwork.cpp feedforwardneuralnetwork.cpp simplerecurrentneuralnetwork.cpp stackedautoencoder.cpp sensorsimulator.cpp controller.cpp controllerneuralnetwork.cpp controllerdeepneuralnetwork.cpp hoveringproblemneuralnetwork.cpp evolutionaryrobotics.cpp lspisimulator.cpp) 
#ENDIF()

IF(MPI_COMPILE_FLAGS)
  SET_TARGET_PROPERTIES(main PROPERTIES COMPILE_FLAGS "${MPI_COMPILE_FLAGS}")
ENDIF()
IF(MPI_LINK_FLAGS)
  SET_TARGET_PROPERTIES(main PROPERTIES LINK_FLAGS "${MPI_LINK_FLAGS}")
ENDIF()

TARGET_LINK_LIBRARIES(main ${MANDATORY_LIBRARIES})
