#ifndef SUN_MATH_HPP
#define SUN_MATH_HPP

#include "raylib.h"

namespace SunSim {
    struct SunState {
        Vector3 position;
        float dayOfYear;
    };

    // Sun state at the current wall-clock instant.
    SunState GetCurrentSunState(float distance);

    // Sun state at an arbitrary instant given as a Julian Date, so the day/night
    // terminator and day-of-year track the simulation clock.
    SunState GetSunState(double julianDate, float distance);
}

#endif