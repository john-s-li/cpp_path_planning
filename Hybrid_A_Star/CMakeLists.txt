cmake_minimum_required(VERSION 3.13)

project(hybrid_a_star)

find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(Python3 3.0 REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
# find_package(Boost 1.80 COMPONENTS program_options REQUIRED)

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

add_executable(hybrid_a_star hybrid_a_star.cpp)

add_subdirectory(kdtree-cpp)
add_subdirectory(reeds-shepp)

target_include_directories(
    hybrid_a_star
    PRIVATE
        ${Python3_INCLUDE_DIRS}
        ${NUMPY_INCLUDE_DIRS}
        kdtree-cpp/include
        reeds-shepp/include
        ${hybrid_a_star_SOURCE_DIR}/include
)

target_link_libraries(
    hybrid_a_star
    PRIVATE
        ${Python3_LIBRARIES}
        Python3::NumPy
        kdtree-cpp
        reeds-shepp
        Eigen3::Eigen
)

target_compile_features(hybrid_a_star PRIVATE cxx_std_17)