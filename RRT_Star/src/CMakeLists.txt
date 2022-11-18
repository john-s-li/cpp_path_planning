add_executable(rrt_star RRT_Star.cpp)

target_include_directories(
    rrt_star
    PRIVATE
        ${Python3_INCLUDE_DIRS} 
        ${NUMPY_INCLUDE_DIRS}
        ../include
)

target_link_libraries(
    rrt_star
    PRIVATE
        ${Python3_LIBRARIES}
        Python3::NumPy
)

target_compile_features(rrt_star PRIVATE cxx_std_17)