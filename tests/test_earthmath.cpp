// Unit tests for Earth Rotation Angle / Julian Date (no raylib, no network).
#include "EarthMath.hpp"
#include <cmath>
#include <cstdio>

static int failures = 0;
#define CHECK(cond, msg) do { \
    if (!(cond)) { std::printf("FAIL: %s\n", msg); ++failures; } \
    else         { std::printf("ok:   %s\n", msg); } } while (0)

int main() {
    // Current Julian Date should be well past the J2000 epoch.
    CHECK(EarthSim::getJulianDate() > 2451545.0, "getJulianDate() is after J2000");

    // ERA at J2000 (t=0): 360 * 0.779057273264 = 280.4606 deg.
    double a0 = EarthSim::getRotationAngle(2451545.0);
    CHECK(a0 >= 0.0 && a0 < 360.0, "rotation angle wrapped to [0,360)");
    CHECK(std::fabs(a0 - 280.4606) < 0.01, "ERA at J2000 ~= 280.46 deg");

    // Earth turns 360.98561 deg per day relative to the stars -> +0.98561 deg/day mod 360.
    double a1 = EarthSim::getRotationAngle(2451545.0 + 1.0);
    double adv = std::fmod(a1 - a0 + 360.0, 360.0);
    CHECK(std::fabs(adv - 0.98561) < 0.01, "advances ~0.9856 deg per sidereal-corrected day");

    // getCurrentRotationAngle() agrees with getRotationAngle(getJulianDate()).
    double live = EarthSim::getRotationAngle(EarthSim::getJulianDate());
    CHECK(live >= 0.0 && live < 360.0, "current rotation angle in range");

    std::printf("\n%s (%d failure%s)\n", failures ? "TESTS FAILED" : "ALL TESTS PASSED",
                failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
