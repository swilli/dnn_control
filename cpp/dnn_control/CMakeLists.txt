cmake_minimum_required (VERSION 2.8.11)
project (dnn_control)

# 1 - Preliminary checks and messages
MESSAGE(STATUS "OS detected: ${CMAKE_SYSTEM_NAME}")
MESSAGE(STATUS "CXX Compiler detected: ${CMAKE_CXX_COMPILER_ID}")

# 2 - Setting compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

# 3 - Find Third-party Libraries
# GSL
FIND_LIBRARY(GSL_GSL_LIBRARY NAMES gsl)
FIND_LIBRARY(GSL_GSLCBLAS_LIBRARY NAMES gslcblas)
FIND_PATH(GSL_INCLUDE_DIR NAMES gsl/gsl_version.h)
MESSAGE(STATUS "GSL library: ${GSL_GSL_LIBRARY}")
MESSAGE(STATUS "GSL CBLAS library: ${GSL_GSLCBLAS_LIBRARY}")
MESSAGE(STATUS "GSL include dir: ${GSL_INCLUDE_DIR}")
IF(NOT GSL_GSL_LIBRARY OR NOT GSL_INCLUDE_DIR)
	MESSAGE(FATAL_ERROR "GSL support was requested, but either the library or the include files could not be located.")
ENDIF(NOT GSL_GSL_LIBRARY OR NOT GSL_INCLUDE_DIR)
IF(NOT GSL_GSLCBLAS_LIBRARY)
	MESSAGE(STATUS "GSL BLAS library not found")
ENDIF(NOT GSL_GSLCBLAS_LIBRARY)

# 4 - Define mandatory libraries and include directories
SET(MANDATORY_LIBRARIES ${MANDATORY_LIBRARIES} ${GSL_GSL_LIBRARY} ${GSL_GSLCBLAS_LIBRARY})
INCLUDE_DIRECTORIES("${GSL_INCLUDE_DIR}")

# 5 - Define the target and linked libraries
add_executable(main asteroid.cpp main.cpp odesystem.cpp sensorsimulator.cpp simulator.cpp spacecraftcontroller.cpp utility.cpp controller5d.cpp fullstatecontroller.cpp controlleranyd.cpp fullstatesensorsimulator.cpp sensorsimulator5d.cpp filewriter.cpp 
sensordatagenerator.cpp sensorsimulatoranyd.cpp) 
TARGET_LINK_LIBRARIES(main ${MANDATORY_LIBRARIES})

