#include "OrbitalMechanics.hpp"
#include <cmath>

namespace OrbitalMechanics {

namespace {
    constexpr double TWO_PI = 6.283185307179586;

    // Wrap an angle into [0, 2π).
    double wrap2pi(double a) {
        a = std::fmod(a, TWO_PI);
        return a < 0.0 ? a + TWO_PI : a;
    }
}

// Standard state-vector → classical-orbital-elements conversion (e.g. Vallado, RV2COE).
Elements fromStateVector(const std::array<double, 6>& s) {
    Elements el;

    const double rx = s[0], ry = s[1], rz = s[2];
    const double vx = s[3], vy = s[4], vz = s[5];

    const double r  = std::sqrt(rx * rx + ry * ry + rz * rz);
    const double v2 = vx * vx + vy * vy + vz * vz;
    if (r < 1e-6) return el; // invalid (valid stays false)

    // Specific angular momentum  h = r × v
    const double hx = ry * vz - rz * vy;
    const double hy = rz * vx - rx * vz;
    const double hz = rx * vy - ry * vx;
    const double h  = std::sqrt(hx * hx + hy * hy + hz * hz);

    // Node vector  n = k̂ × h = (-hy, hx, 0)
    const double nx = -hy, ny = hx;
    const double nmag = std::sqrt(nx * nx + ny * ny);

    // Eccentricity vector  e = ((v² − μ/r) r − (r·v) v) / μ
    const double rv = rx * vx + ry * vy + rz * vz;
    const double c1 = (v2 - MU_EARTH / r) / MU_EARTH;
    const double c2 = rv / MU_EARTH;
    const double ex = c1 * rx - c2 * vx;
    const double ey = c1 * ry - c2 * vy;
    const double ez = c1 * rz - c2 * vz;
    const double e  = std::sqrt(ex * ex + ey * ey + ez * ez);

    // Semi-major axis from specific orbital energy
    const double energy = v2 / 2.0 - MU_EARTH / r;
    if (std::fabs(energy) < 1e-12) return el; // parabolic — not handled
    const double a = -MU_EARTH / (2.0 * energy);

    const double i = std::acos(hz / h);

    double raan = 0.0;
    if (nmag > 1e-9) {
        raan = std::acos(nx / nmag);
        if (ny < 0.0) raan = TWO_PI - raan;
    }

    double argp = 0.0;
    if (nmag > 1e-9 && e > 1e-9) {
        argp = std::acos((nx * ex + ny * ey) / (nmag * e));
        if (ez < 0.0) argp = TWO_PI - argp;
    }

    double nu = 0.0;
    if (e > 1e-9) {
        nu = std::acos((ex * rx + ey * ry + ez * rz) / (e * r));
        if (rv < 0.0) nu = TWO_PI - nu;
    }

    // Eccentric and mean anomaly at epoch
    const double E = std::atan2(std::sqrt(1.0 - e * e) * std::sin(nu), e + std::cos(nu));
    const double M = E - e * std::sin(E);

    el.a      = a;
    el.e      = e;
    el.i      = i;
    el.raan   = raan;
    el.argp   = argp;
    el.nu0    = nu;
    el.M0     = wrap2pi(M);
    el.n      = std::sqrt(MU_EARTH / (a * a * a));
    el.period = TWO_PI / el.n;
    el.valid  = true;
    return el;
}

std::array<double, 3> propagate(const Elements& el, double dtSeconds) {
    if (!el.valid) return {0.0, 0.0, 0.0};

    // Mean anomaly at the requested time.
    const double M = el.M0 + el.n * dtSeconds;

    // Solve Kepler's equation  E − e·sin E = M  by Newton–Raphson.
    double E = M;
    for (int k = 0; k < 60; ++k) {
        const double f  = E - el.e * std::sin(E) - M;
        const double fp = 1.0 - el.e * std::cos(E);
        const double dE = f / fp;
        E -= dE;
        if (std::fabs(dE) < 1e-12) break;
    }

    const double nu = 2.0 * std::atan2(std::sqrt(1.0 + el.e) * std::sin(E / 2.0),
                                       std::sqrt(1.0 - el.e) * std::cos(E / 2.0));
    const double r  = el.a * (1.0 - el.e * std::cos(E));

    // Position in the perifocal (PQW) frame.
    const double xp = r * std::cos(nu);
    const double yp = r * std::sin(nu);

    // Rotate PQW → ECI via R = Rz(raan)·Rx(i)·Rz(argp).
    const double cO = std::cos(el.raan), sO = std::sin(el.raan);
    const double ci = std::cos(el.i),    si = std::sin(el.i);
    const double cw = std::cos(el.argp), sw = std::sin(el.argp);

    const double R11 = cO * cw - sO * sw * ci;
    const double R12 = -cO * sw - sO * cw * ci;
    const double R21 = sO * cw + cO * sw * ci;
    const double R22 = -sO * sw + cO * cw * ci;
    const double R31 = sw * si;
    const double R32 = cw * si;

    return { R11 * xp + R12 * yp,
             R21 * xp + R22 * yp,
             R31 * xp + R32 * yp };
}

std::array<double, 3> orbitPositionAt(double a, double e, double i,
                                      double raan, double argp, double nu) {
    const double p = a * (1.0 - e * e);
    const double r = p / (1.0 + e * std::cos(nu));
    const double xp = r * std::cos(nu);
    const double yp = r * std::sin(nu);

    const double cO = std::cos(raan), sO = std::sin(raan);
    const double ci = std::cos(i),    si = std::sin(i);
    const double cw = std::cos(argp), sw = std::sin(argp);

    const double R11 = cO * cw - sO * sw * ci, R12 = -cO * sw - sO * cw * ci;
    const double R21 = sO * cw + cO * sw * ci, R22 = -sO * sw + cO * cw * ci;
    const double R31 = sw * si,                R32 = cw * si;

    return { R11 * xp + R12 * yp,
             R21 * xp + R22 * yp,
             R31 * xp + R32 * yp };
}

Hohmann computeHohmann(double r1, double r2) {
    Hohmann h;
    h.r1 = r1;
    h.r2 = r2;
    h.raising = (r2 >= r1);

    const double at = (r1 + r2) / 2.0;             // transfer-ellipse semi-major axis
    const double v1 = std::sqrt(MU_EARTH / r1);    // circular speed, initial orbit
    const double v2 = std::sqrt(MU_EARTH / r2);    // circular speed, target orbit
    const double vp = std::sqrt(MU_EARTH * (2.0 / r1 - 1.0 / at));  // transfer speed at r1
    const double va = std::sqrt(MU_EARTH * (2.0 / r2 - 1.0 / at));  // transfer speed at r2

    h.aTransfer    = at;
    h.eTransfer    = std::fabs(r2 - r1) / (r1 + r2);
    h.dv1          = std::fabs(vp - v1);
    h.dv2          = std::fabs(v2 - va);
    h.dvTotal      = h.dv1 + h.dv2;
    h.transferTime = TWO_PI / 2.0 * std::sqrt(at * at * at / MU_EARTH);  // half the ellipse period
    return h;
}

} // namespace OrbitalMechanics
