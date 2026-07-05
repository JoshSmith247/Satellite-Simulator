#include "EarthMath.hpp"
#include <chrono>
#include <cmath>
#include <ctime>

namespace EarthSim {

double getJulianDate() {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    double seconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() / 1000.0;

    // 2440587.5 = Julian Date of the Unix epoch.
    return (seconds / 86400.0) + 2440587.5;
}

double getRotationAngle(double jd) {
    const double t = jd - 2451545.0;   // days since J2000

    // Earth Rotation Angle (IERS formula), converted to degrees in [0, 360).
    double angle = 360.0 * (0.779057273264 + 1.00273781191135448 * t);
    angle = std::fmod(angle, 360.0);
    if (angle < 0) angle += 360.0;

    return angle;
}

double getCurrentRotationAngle() {
    return getRotationAngle(getJulianDate());
}

} // namespace EarthSim