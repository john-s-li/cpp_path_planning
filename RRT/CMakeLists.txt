cmake_minimum_required(VERSION 3.0)

project(rrt_solver)

find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(Python3 3.0 REQUIRED)

# set(PYTHON_INCLUDE_DIRS "opt/homebrew/Caskroom/miniforge/base/include/python3.9")
# set(PYTHON_LIBRARIES "/opt/homebrew/Caskroom/miniforge/base/lib/libpython3.9.dylib")
# set(NUMPY_INCLUDE_DIRS "/opt/homebrew/Caskroom/miniforge/base/lib/python3.9/site-packages/numpy/core/include/numpy")

include_directories(${PYTHON3_INCLUDE_DIRS} ${NUMPY_INCLUDE_DIRS})

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

find_package(Matplot++ REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

add_executable(rrt rrt.cpp)

target_link_libraries(
    rrt
    PUBLIC
        Matplot++::matplot
        Eigen3::Eigen
        ${PYTHON_LIBRARIES}
        Python3::NumPy
)

target_compile_features(rrt PRIVATE cxx_std_17)

