#find_package(Boost 1.80 COMPONENTS program_options REQUIRED)

include_directories(include)
add_library(reeds-shepp reeds_shepp.cpp)
set_property(TARGET reeds-shepp PROPERTY CXX_STANDARD 17)
#target_link_libraries(reeds-shepp PRIVATE Boost::program_options)