#ifndef EARTH_MATH_HPP
#define EARTH_MATH_HPP

namespace EarthSim {

    // Earth Rotation Angle (degrees, IERS formula) at the current wall-clock instant.
    double getCurrentRotationAngle();

    // Earth Rotation Angle (degrees) at a given Julian Date, so the globe
    // spins to match the simulation clock.
    double getRotationAngle(double julianDate);

    double getJulianDate();
}

#endif