# Specify the minimum required CMake version
cmake_minimum_required(VERSION 3.16)

# Define the project name
project(aquila LANGUAGES CXX)

# Set the C++ version standard to be used
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Include file containing custom functions to enable testing
include(CTest)

# Add FSW module with respective CMakeLists.txt
add_subdirectory(fsw)