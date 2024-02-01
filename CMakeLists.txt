cmake_minimum_required(VERSION 3.5)
project(SensorFusionCpp VERSION 0.1.0 LANGUAGES C CXX)

# Set the path to the Eigen library
set(EIGEN_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/external/eigen)

# Add the path to the include directories
include_directories(
    ${EIGEN_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/include  # Add this line to include the "include" directory
)

# Add your source and header files (use file(GLOB ...) instead of file(GLOB_RECURSE ...))
file(GLOB SOURCE_FILES src/*.cpp)
file(GLOB HEADER_FILES include/*.hpp)

# Add your source files here
add_executable(SensorFusionCpp ${SOURCE_FILES} ${HEADER_FILES})

# Set include directories for the target
target_include_directories(SensorFusionCpp SYSTEM PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src
    ${EIGEN_INCLUDE_DIR}
)

# Link any additional libraries if needed
target_link_libraries(SensorFusionCpp)