cmake_minimum_required (VERSION 2.8.11)
project (dnn_control)

add_executable(main asteroid.cpp main.cpp odesystem.cpp sensorsimulator.cpp simulator.cpp spacecraftcontroller.cpp test.cpp utility.cpp) 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -lgsl -lgslcblas -O2")
