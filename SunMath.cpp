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
        state.dayOfYear = (float)(n + 182.5); // Approximate for UI display
        
        // Position in the Ecliptic Plane (X, Z)
        state.position.x = cosf(lambdaRad) * distance;
        state.position.z = sinf(lambdaRad) * distance;
        state.position.y = 0.0f; 

        return state;
    }
}