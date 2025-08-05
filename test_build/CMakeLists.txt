# Simple test build script
cmake_minimum_required(VERSION 3.20)
project(PhysicsTest)

# Physics test executable
add_executable(test_physics
    ../test_physics.cpp
    ../source/solver.cpp
    ../source/rigid.cpp
    ../source/manifold.cpp
    ../source/collision.cpp
    ../source/force.cpp
    ../source/joint.cpp
    ../source/spring.cpp
)

target_include_directories(test_physics PRIVATE ..)
target_compile_features(test_physics PRIVATE cxx_std_11)

# Set compiler flags for proper math
if(UNIX)
    target_link_libraries(test_physics m)
endif()