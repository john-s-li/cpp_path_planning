cmake_minimum_required(VERSION 3.13)

project(hybrid_a_star)

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

include(FetchContent)
FetchContent_Declare(
	pybind11
	GIT_REPOSITORY https://github.com/pybind/pybind11.git
	GIT_TAG        v2.6.2
	GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(pybind11)

add_executable(hybrid_a_star hybrid_a_star.cpp)

add_subdirectory(kdtree-cpp)

target_include_directories(
    hybrid_a_star
    PRIVATE
        ${Python3_INCLUDE_DIRS}
        ${NUMPY_INCLUDE_DIRS}
        kdtree-cpp/include
        ${hybrid_a_star_SOURCE_DIR}/include
)

target_link_libraries(
    hybrid_a_star
    PRIVATE
        ${Python3_LIBRARIES}
        Python3::NumPy
        kdtree-cpp
        pybind11::embed
)

target_compile_features(hybrid_a_star PRIVATE cxx_std_17)