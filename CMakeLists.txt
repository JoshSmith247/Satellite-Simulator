cmake_minimum_required(VERSION 3.10)
project(SatelliteSim)

set(CMAKE_CXX_STANDARD 17)

# Tell CMake where to look for Homebrew libraries on Apple Silicon
list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew/opt/raylib")

find_package(raylib REQUIRED)
find_package(CURL REQUIRED)

file(GLOB SGP4_SOURCES "third_party/sgp4/libsgp4/*.cc")

add_executable(sat_sim 
    main.cpp 
    SunMath.cpp 
    EarthMath.cpp
    fetchTLE.cpp
    ${SGP4_SOURCES}
)

target_link_libraries(sat_sim raylib CURL::libcurl)

target_include_directories(sat_sim PRIVATE
    third_party/sgp4/libsgp4
    third_party/raygui
)

add_custom_command(TARGET sat_sim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/assets
        ${CMAKE_BINARY_DIR}
)
