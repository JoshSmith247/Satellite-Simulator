#ifndef EARTH_MATH_HPP
#define EARTH_MATH_HPP

namespace EarthSim {

    // Earth Rotation Angle (degrees, IERS formula) at the current wall-clock instant.
    double getCurrentRotationAngle();

    // Earth Rotation Angle (degrees) at an arbitrary instant given as a Julian Date.
    // Used so the globe spins to match the simulation clock, not just "now".
    double getRotationAngle(double julianDate);

    double getJulianDate();
}

#endif