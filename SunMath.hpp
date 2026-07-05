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

    // Sun state at a given Julian Date, so the terminator tracks the sim clock.
    SunState GetSunState(double julianDate, float distance);
}

#endif