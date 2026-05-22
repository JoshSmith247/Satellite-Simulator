#ifndef SUN_MATH_HPP
#define SUN_MATH_HPP

#include "raylib.h"

namespace SunSim {
    struct SunState {
        Vector3 position;
        float dayOfYear;
    };

    SunState GetCurrentSunState(float distance);
}

#endif