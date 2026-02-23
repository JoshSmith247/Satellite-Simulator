#include "EarthMath.hpp"
#include <chrono>
#include <cmath>

namespace EarthSim {

double getJulianDate() {
    // Get current system time in seconds since Unix Epoch (Jan 1, 1970)
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    double seconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() / 1000.0;

    // Convert Unix Epoch to Julian Date
    // 2440587.5 is the Julian Date of the Unix Epoch
    return (seconds / 86400.0) + 2440587.5;
}

double getCurrentRotationAngle() {
    double jd = getJulianDate();
    
    // Constant for the Julian Date of J2000 epoch
    const double t = jd - 2451545.0;
    
    // Earth Rotation Angle (ERA) formula (IERS standard)
    // Returns the angle in fractions of a circle, then converted to degrees
    double angle = 360.0 * (0.779057273264 + 1.00273781191135448 * t);
    
    // Keep the result between 0 and 360 degrees
    angle = std::fmod(angle, 360.0);
    if (angle < 0) angle += 360.0;
    
    return angle;
}

} // namespace EarthSim