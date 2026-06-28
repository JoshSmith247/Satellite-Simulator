cmake_minimum_required(VERSION 3.10)
project(SatelliteSim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build the GUI application (needs raylib). Turn OFF to build only the headless
# unit tests (e.g. in CI), which need neither raylib nor a display.
option(BUILD_APP "Build the GUI application (requires raylib)" ON)
option(BUILD_TESTING "Build unit tests" ON)

# Help CMake find a Homebrew-installed raylib without hardcoding one layout:
# Apple Silicon (/opt/homebrew) and Intel (/usr/local) both work, and an
# explicit -DCMAKE_PREFIX_PATH or a system/package-manager install still wins.
if(APPLE)
    list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew/opt/raylib" "/usr/local/opt/raylib")
endif()

find_package(CURL REQUIRED)

file(GLOB SGP4_SOURCES "third_party/sgp4/libsgp4/*.cc")

if(BUILD_APP)
    find_package(raylib REQUIRED)

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
endif()

# ── Tests ───────────────────────────────────────────────────────────────────
# Headless unit tests — no raylib or display needed (run with `ctest`).
# Build with -DBUILD_TESTING=OFF to skip.
if(BUILD_TESTING)
    enable_testing()

    add_executable(test_orbital tests/test_orbital.cpp OrbitalMechanics.cpp)
    target_include_directories(test_orbital PRIVATE .)
    add_test(NAME orbital COMMAND test_orbital)

    add_executable(test_passpredict tests/test_passpredict.cpp PassPredict.cpp ${SGP4_SOURCES})
    target_include_directories(test_passpredict PRIVATE . third_party/sgp4/libsgp4)
    add_test(NAME passpredict COMMAND test_passpredict)

    add_executable(test_simclock tests/test_simclock.cpp SimClock.cpp ${SGP4_SOURCES})
    target_include_directories(test_simclock PRIVATE . third_party/sgp4/libsgp4)
    add_test(NAME simclock COMMAND test_simclock)

    add_executable(test_earthmath tests/test_earthmath.cpp EarthMath.cpp)
    target_include_directories(test_earthmath PRIVATE .)
    add_test(NAME earthmath COMMAND test_earthmath)

    # fetchTLE pulls in libcurl (the TLE fetch lives in the same TU), but the test
    # itself only exercises offline validation/parsing.
    add_executable(test_fetchtle tests/test_fetchtle.cpp fetchTLE.cpp ${SGP4_SOURCES})
    target_include_directories(test_fetchtle PRIVATE . third_party/sgp4/libsgp4)
    target_link_libraries(test_fetchtle CURL::libcurl)
    add_test(NAME fetchtle COMMAND test_fetchtle)
endif()
