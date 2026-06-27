cmake_minimum_required(VERSION 3.10)
project(SatelliteSim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Help CMake find a Homebrew-installed raylib without hardcoding one layout:
# Apple Silicon (/opt/homebrew) and Intel (/usr/local) both work, and an
# explicit -DCMAKE_PREFIX_PATH or a system/package-manager install still wins.
if(APPLE)
    list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew/opt/raylib" "/usr/local/opt/raylib")
endif()

find_package(raylib REQUIRED)
find_package(CURL REQUIRED)

file(GLOB SGP4_SOURCES "third_party/sgp4/libsgp4/*.cc")

add_executable(sat_sim
    main.cpp
    SunMath.cpp
    EarthMath.cpp
    fetchTLE.cpp
    OrbitalMechanics.cpp
    SimClock.cpp
    PassPredict.cpp
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

# ── Tests ───────────────────────────────────────────────────────────────────
# Headless unit tests for the orbital math — no raylib or network needed.
# Build with -DBUILD_TESTING=OFF to skip, run with `ctest` in the build dir.
option(BUILD_TESTING "Build unit tests" ON)
if(BUILD_TESTING)
    enable_testing()

    add_executable(test_orbital tests/test_orbital.cpp OrbitalMechanics.cpp)
    target_include_directories(test_orbital PRIVATE .)
    add_test(NAME orbital COMMAND test_orbital)

    add_executable(test_passpredict tests/test_passpredict.cpp PassPredict.cpp ${SGP4_SOURCES})
    target_include_directories(test_passpredict PRIVATE . third_party/sgp4/libsgp4)
    add_test(NAME passpredict COMMAND test_passpredict)
endif()
