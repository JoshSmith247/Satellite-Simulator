// Unit tests for the controllable simulation clock (no raylib, no network).
#include "SimClock.hpp"
#include "DateTime.h"
#include <cmath>
#include <cstdio>

static int failures = 0;
#define CHECK(cond, msg) do { \
    if (!(cond)) { std::printf("FAIL: %s\n", msg); ++failures; } \
    else         { std::printf("ok:   %s\n", msg); } } while (0)

int main() {
    // The Unix epoch must map to 0 seconds (this is the microsecond-tick conversion
    // that silently breaks if the epoch constant is wrong).
    CHECK(std::fabs(SimClock::toUnixSeconds(libsgp4::DateTime(1970, 1, 1)) - 0.0) < 1e-3,
          "DateTime(1970,1,1) -> 0 Unix seconds");

    // 2024-06-01 12:00:00 UTC == 1717243200.
    const double t = 1717243200.0;
    SimClock c;
    c.setUnixSeconds(t);
    CHECK(std::fabs(c.unixSeconds() - t) < 1e-6, "setUnixSeconds round-trips");
    CHECK(std::fabs(SimClock::toUnixSeconds(c.dateTime()) - t) < 1e-3,
          "unix -> DateTime -> unix round-trips");
    CHECK(c.dateTime().ToString().substr(0, 19) == "2024-06-01 12:00:00",
          "DateTime renders the expected calendar instant");

    // Julian Date relationship.
    CHECK(std::fabs(c.julianDate() - (t / 86400.0 + 2440587.5)) < 1e-9, "julianDate() matches");

    // Speed scaling and pause.
    c.setUnixSeconds(1000.0);
    c.setSpeed(60.0);
    c.update(1.0);
    CHECK(std::fabs(c.unixSeconds() - 1060.0) < 1e-6, "1s at 60x advances 60 sim-seconds");
    c.pause();
    c.update(5.0);
    CHECK(std::fabs(c.unixSeconds() - 1060.0) < 1e-6, "paused clock does not advance");
    c.resume();
    c.update(2.0);
    CHECK(std::fabs(c.unixSeconds() - 1180.0) < 1e-6, "resumes at the set speed (2s*60)");

    // Absolute and relative jumps.
    c.setUnixSeconds(5000.0);
    c.jumpSeconds(123.0);
    CHECK(std::fabs(c.unixSeconds() - 5123.0) < 1e-6, "jumpSeconds offsets the instant");

    // A freshly constructed clock starts near real 'now' (well after the J2000-era epoch).
    SimClock now;
    CHECK(now.unixSeconds() > 1.7e9, "fresh clock initialises to wall-clock now");

    std::printf("\n%s (%d failure%s)\n", failures ? "TESTS FAILED" : "ALL TESTS PASSED",
                failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
