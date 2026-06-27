// Unit tests for the from-scratch two-body propagator (no raylib, no network).
#include "OrbitalMechanics.hpp"
#include <array>
#include <cmath>
#include <cstdio>

static int failures = 0;
#define CHECK(cond, msg) do { \
    if (!(cond)) { std::printf("FAIL: %s\n", msg); ++failures; } \
    else         { std::printf("ok:   %s\n", msg); } } while (0)

int main() {
    using namespace OrbitalMechanics;

    // Equatorial circular LEO: r on +x, v on +y, |v| = sqrt(mu/r).
    const double r  = 6798.0;
    const double vc = std::sqrt(MU_EARTH / r);          // ~7.657 km/s
    std::array<double, 6> sv = { r, 0.0, 0.0, 0.0, vc, 0.0 };

    Elements el = fromStateVector(sv);
    CHECK(el.valid,                       "elements are valid");
    CHECK(std::fabs(el.a - r) < 1.0,      "semi-major axis ~= r");
    CHECK(el.e < 1e-3,                    "eccentricity ~= 0 (circular)");
    CHECK(el.i < 1e-3,                    "inclination ~= 0 (equatorial)");

    // Period of a 6798 km circular orbit is ~92.9 min.
    double Tmin = el.period / 60.0;
    CHECK(Tmin > 92.0 && Tmin < 94.0,     "period ~= 93 min");

    // propagate(0) must reproduce the seed position.
    auto p0 = propagate(el, 0.0);
    CHECK(std::fabs(p0[0] - r) < 1.0 && std::fabs(p0[1]) < 1.0 && std::fabs(p0[2]) < 1.0,
          "propagate(0) reproduces seed position");

    // A quarter period later a prograde equatorial orbit is at (0, +r, 0).
    auto pq = propagate(el, el.period / 4.0);
    CHECK(std::fabs(pq[1] - r) < 5.0 && std::fabs(pq[0]) < 5.0,
          "propagate(T/4) -> +y");

    // A full period returns to the start.
    auto pT = propagate(el, el.period);
    CHECK(std::fabs(pT[0] - r) < 5.0 && std::fabs(pT[1]) < 5.0,
          "propagate(T) returns to start");

    // Apogee/perigee of a circular orbit both ~= altitude (r - Re).
    CHECK(std::fabs(el.apogeeAltKm() - el.perigeeAltKm()) < 1.0,
          "apogee == perigee for circular orbit");

    std::printf("\n%s (%d failure%s)\n", failures ? "TESTS FAILED" : "ALL TESTS PASSED",
                failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
