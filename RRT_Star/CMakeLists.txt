cmake_minimum_required(VERSION 3.0)

project(rrt_star)

find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(Python3 3.0 REQUIRED)

include(FetchContent)
FetchContent_Declare(
    matplotlib
    GIT_REPOSITORY https://github.com/lava/matplotlib-cpp.git
    GIT_TAG        f23347fca25219d1c42cbb91608b5556814bf572
)
FetchContent_GetProperties(matplotlib)
if(NOT matplotlib_POPULATED)
    FetchContent_Populate(matplotlib)
endif()
include_directories(SYSTEM ${matplotlib_SOURCE_DIR})

add_executable(rrt_star RRT_Star.cpp RRT.cpp)

target_include_directories(
    rrt_star
    PRIVATE
        ${Python3_INCLUDE_DIRS} 
        ${NUMPY_INCLUDE_DIRS}
        "include/"
)

target_link_libraries(
    rrt_star
    PRIVATE
        ${Python3_LIBRARIES}
        Python3::NumPy

)

target_compile_features(rrt_star PRIVATE cxx_std_17)