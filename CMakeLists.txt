cmake_minimum_required(VERSION 2.8.3)
project(STLAutomaton)

# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -std=c++14 -march=native -funroll-all-loops -fpeel-loops -ftracer -ftree-vectorize -mavx2 -mfma -fomit-frame-pointer -fpermissive")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -std=c++17")

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(ompl REQUIRED)
find_package(Threads REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${BOOST_INCLUDE_DIRS} ${ompl_INCLUDE_DIRS}})


add_library (stl_lib
    src/STLAutomaton.cpp
)


target_link_libraries (stl_lib
    ${Boost_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${OMPL_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)

target_include_directories(stl_lib
    PUBLIC
    ${OMPL_INCLUDE_DIRS}
)

add_executable (main.a main.cpp)

target_link_libraries (main.a
    stl_lib
    ${OMPL_LIBRARIES}
    ${Boost_LIBRARIES}
)