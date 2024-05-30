cmake_minimum_required(VERSION 3.24)
project(Cpp-VDW)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

# Include directories
include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${GLFW_INCLUDE_DIRS})

set(CMAKE_CXX_STANDARD 17)
include_directories(/Users/yangtianyue/Desktop/Programming/CPP_Libs/eigen-3.4.0)
include_directories(GLFW)
include_directories(glm)
include_directories(glad)
add_executable(Cpp-VDW main.cpp Ball.cpp Container.cpp Simulation.cpp Simulation.h)


# Link libraries
target_link_libraries(Cpp-VDW ${OPENGL_LIBRARIES} glfw)


