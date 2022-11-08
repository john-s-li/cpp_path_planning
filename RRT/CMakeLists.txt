cmake_minimum_required(VERSION 3.0)

project(rrt_solver)

add_executable(rrt rrt.cpp)

find_package(Matplot++ REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

target_link_libraries(
    rrt
    PUBLIC
        Matplot++::matplot
        Eigen3::Eigen
)

target_compile_features(rrt PRIVATE cxx_std_17)

