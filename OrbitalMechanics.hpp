#ifndef ORBITAL_MECHANICS_HPP
#define ORBITAL_MECHANICS_HPP

#include <array>

// Minimal two-body (Keplerian) orbital mechanics, independent of SGP4.
// Seeded from an ECI state vector, it derives the classical orbital elements
// and propagates analytically by solving Kepler's equation. Unlike SGP4 it
// includes no perturbations (no J2, no drag), so its track drifts from SGP4
// over time — which is the point of plotting the two side by side.
namespace OrbitalMechanics {

    // Earth gravitational parameter (km^3 / s^2) and mean equatorial radius (km).
    constexpr double MU_EARTH    = 398600.4418;
    constexpr double EARTH_RADIUS_KM = 6378.137;

    struct Elements {
        double a      = 0.0;  // semi-major axis (km)
        double e      = 0.0;  // eccentricity
        double i      = 0.0;  // inclination (rad)
        double raan   = 0.0;  // right ascension of ascending node (rad)
        double argp   = 0.0;  // argument of perigee (rad)
        double nu0    = 0.0;  // true anomaly at epoch (rad)
        double M0     = 0.0;  // mean anomaly at epoch (rad)
        double n      = 0.0;  // mean motion (rad/s)
        double period = 0.0;  // orbital period (s)
        bool   valid  = false;

        double apogeeAltKm()  const { return a * (1.0 + e) - EARTH_RADIUS_KM; }
        double perigeeAltKm() const { return a * (1.0 - e) - EARTH_RADIUS_KM; }
    };

    // Classical elements from an ECI state vector {x,y,z (km), vx,vy,vz (km/s)}.
    Elements fromStateVector(const std::array<double, 6>& state);

    // Two-body propagation: ECI position {x,y,z} (km) at dtSeconds after the epoch
    // the elements were derived for.
    std::array<double, 3> propagate(const Elements& el, double dtSeconds);

} // namespace OrbitalMechanics

#endif
