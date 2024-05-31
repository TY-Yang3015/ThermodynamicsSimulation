cmake_minimum_required(VERSION 3.24)
project(Cpp)
set(CMAKE_CXX_STANDARD 17)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

file(GLOB_RECURSE SOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/Vanilla/*.cpp")
add_executable(Cpp main.cpp ${SOURCE_FILES})

include_directories(lib)
target_include_directories(Cpp PRIVATE ${OPENGL_INCLUDE_DIR} ${GLFW_INCLUDE_DIR})
target_link_libraries(Cpp ${OPENGL_LIBRARIES} glfw ${LIB_FILES})




