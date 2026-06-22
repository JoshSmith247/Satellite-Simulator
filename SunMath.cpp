#include "SunMath.hpp"
#include "EarthMath.hpp"
#include <cmath>
#include <ctime>

namespace SunSim {

    // Day-of-year (1-based, with fractional day) for a Julian Date, for the UI label.
    static float dayOfYearFromJD(double jd) {
        // Gregorian calendar from JD (Fliegel–Van Flandern).
        long J = (long)(jd + 0.5);
        double frac = (jd + 0.5) - (double)J;
        long a = J + 32044;
        long b = (4 * a + 3) / 146097;
        long c = a - (146097 * b) / 4;
        long d = (4 * c + 3) / 1461;
        long e = c - (1461 * d) / 4;
        long m = (5 * e + 2) / 153;
        int day   = (int)(e - (153 * m + 2) / 5 + 1);
        int month = (int)(m + 3 - 12 * (m / 10));
        int year  = (int)(100 * b + d - 4800 + m / 10);

        static const int cumulative[] = {0,31,59,90,120,151,181,212,243,273,304,334};
        bool leap = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
        int doy = cumulative[month - 1] + day + ((leap && month > 2) ? 1 : 0);
        return (float)doy + (float)frac;
    }

    // Calculates the Sun's position relative to Earth at the given Julian Date.
    SunState GetSunState(double jd, float distance) {
        double n = jd - 2451545.0;

        // Mean longitude (L)
        double L = fmod(280.460 + 0.9856474 * n, 360.0);
        // Mean anomaly (g)
        double g = fmod(357.528 + 0.9856003 * n, 360.0);

        // Ecliptic longitude (lambda)
        double lambda = L + 1.915 * sin(g * DEG2RAD) + 0.020 * sin(2.0 * g * DEG2RAD);
        double lambdaRad = lambda * DEG2RAD;

        SunState state;
        state.dayOfYear = dayOfYearFromJD(jd);

        // Ecliptic → ECI → canonical Raylib (X=-ECI_Y, Y=ECI_Z, Z=-ECI_X)
        // → world space (+RotateZ 23.44°, same as satellites in main.cpp).
        // The obliquity cancels: sun always lands at world Y=0 (ecliptic = world XZ plane).
        state.position.x = -sinf(lambdaRad) * distance;
        state.position.y = 0.0f;
        state.position.z = -cosf(lambdaRad) * distance;

        return state;
    }

    // Calculates the Sun's position relative to Earth based on the current system date.
    SunState GetCurrentSunState(float distance) {
        return GetSunState(EarthSim::getJulianDate(), distance);
    }
}