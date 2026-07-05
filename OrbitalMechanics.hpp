#ifndef ORBITAL_MECHANICS_HPP
#define ORBITAL_MECHANICS_HPP

#include <array>

// Minimal two-body Keplerian mechanics, independent of SGP4. No perturbations
// (no J2, no drag), so its track intentionally drifts from SGP4 over time.
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

    // ECI position {x,y,z} (km) on an orbit at a given true anomaly (rad).
    std::array<double, 3> orbitPositionAt(double a, double e, double i,
                                          double raan, double argp, double nu);

    // A two-impulse Hohmann transfer between two circular orbits of radius r1 -> r2 (km).
    struct Hohmann {
        double r1 = 0.0, r2 = 0.0;        // initial / target circular radii (km)
        double dv1 = 0.0, dv2 = 0.0;      // burn magnitudes (km/s)
        double dvTotal = 0.0;             // km/s
        double transferTime = 0.0;        // seconds (perigee -> apogee)
        double aTransfer = 0.0;           // transfer-ellipse semi-major axis (km)
        double eTransfer = 0.0;           // transfer-ellipse eccentricity
        bool   raising = true;            // r2 >= r1 (burns prograde) vs lowering
    };
    Hohmann computeHohmann(double r1, double r2);

} // namespace OrbitalMechanics

#endif
