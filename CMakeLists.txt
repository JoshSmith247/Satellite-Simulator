cmake_minimum_required(VERSION 3.10)
project(SatelliteSim)

set(CMAKE_CXX_STANDARD 17)

# Tell CMake where to look for Homebrew libraries on Apple Silicon
list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew/opt/raylib")

find_package(raylib REQUIRED)

add_executable(sat_sim main.cpp)

# Link the raylib library to your executable
target_link_libraries(sat_sim raylib)