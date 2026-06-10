#include "SunMath.hpp"
#include "EarthMath.hpp"
#include <cmath>
#include <ctime>

namespace SunSim {

    // Calculates the Sun's position relative to Earth based on the current system date
    SunState GetCurrentSunState(float distance) {
        double jd = EarthSim::getJulianDate();
        double n = jd - 2451545.0;

        // Mean longitude (L)
        double L = fmod(280.460 + 0.9856474 * n, 360.0);
        // Mean anomaly (g)
        double g = fmod(357.528 + 0.9856003 * n, 360.0);

        // Ecliptic longitude (lambda)
        double lambda = L + 1.915 * sin(g * DEG2RAD) + 0.020 * sin(2.0 * g * DEG2RAD);
        double lambdaRad = lambda * DEG2RAD;

        SunState state;
        
        // Approx. day of year for UI
        time_t t = time(NULL);
        struct tm *now = gmtime(&t);
        if (now) {
            state.dayOfYear = (float)now->tm_yday + 1.0f + ((float)now->tm_hour / 24.0f);
        } else {
            state.dayOfYear = 0.0f;
        }
        
        // Ecliptic → ECI → canonical Raylib (X=-ECI_Y, Y=ECI_Z, Z=-ECI_X)
        // → world space (+RotateZ 23.44°, same as satellites in main.cpp).
        // The obliquity cancels: sun always lands at world Y=0 (ecliptic = world XZ plane).
        state.position.x = -sinf(lambdaRad) * distance;
        state.position.y = 0.0f;
        state.position.z = -cosf(lambdaRad) * distance;

        return state;
    }
}