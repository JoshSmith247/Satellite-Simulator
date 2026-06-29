// Unit tests for pass prediction, look angles and Doppler (no raylib, no network).
#include "PassPredict.hpp"
#include "Tle.h"
#include "SGP4.h"
#include "CoordGeodetic.h"
#include "DateTime.h"
#include <cmath>
#include <cstdio>

static int failures = 0;
#define CHECK(cond, msg) do { \
    if (!(cond)) { std::printf("FAIL: %s\n", msg); ++failures; } \
    else         { std::printf("ok:   %s\n", msg); } } while (0)

int main() {
    using namespace libsgp4;

    // Fixed ISS TLE so the test is deterministic (prediction is relative to epoch).
    Tle tle("ISS (ZARYA)",
        "1 25544U 98067A   24153.51782528  .00016717  00000-0  30074-3 0  9993",
        "2 25544  51.6398 211.6055 0004168  86.9505 273.2003 15.49960977 12345");
    SGP4 sgp4(tle);
    CoordGeodetic station(38.627, -90.199, 0.142);   // St. Louis (WUSat)
    DateTime start = tle.Epoch();

    auto passes = PassPredict::predictPasses(sgp4, station, start, 24.0, 5.0);
    CHECK(!passes.empty(),               "found at least one pass in 24 h");
    CHECK(passes.size() >= 4 && passes.size() <= 10, "ISS pass count over St. Louis is plausible");

    bool geometryOk = true;
    for (const auto& p : passes) {
        double dur = (p.los - p.aos).TotalSeconds();
        if (dur <= 0.0 || dur > 1200.0)                  geometryOk = false;  // ISS passes < 20 min
        if (p.maxElevationDeg < 5.0 || p.maxElevationDeg > 90.0) geometryOk = false;
        if (p.aosAzimuthDeg < 0.0 || p.aosAzimuthDeg > 360.0)    geometryOk = false;
        if (!(p.los > p.aos))                            geometryOk = false;
        if (!(p.tca >= p.aos && p.tca <= p.los))         geometryOk = false;
    }
    CHECK(geometryOk, "every pass has sane duration/elevation/azimuth/ordering");

    // Passes must be in chronological order.
    bool ordered = true;
    for (size_t i = 1; i < passes.size(); i++)
        if (!(passes[i].aos > passes[i - 1].aos)) ordered = false;
    CHECK(ordered, "passes are chronologically ordered");

    // The elevation mask must be honoured.
    auto few = PassPredict::predictPasses(sgp4, station, start, 24.0, 60.0);
    bool maskOk = true;
    for (const auto& p : few) if (p.maxElevationDeg < 60.0) maskOk = false;
    CHECK(maskOk && few.size() <= passes.size(), "higher elevation mask filters passes");

    // Doppler: approaching (negative range-rate) shifts UP. f*v/c at -7 km/s on 145.8 MHz ~ +3404 Hz.
    double shifted = PassPredict::dopplerShiftedHz(145.8e6, -7.0);
    double offset  = shifted - 145.8e6;
    CHECK(offset > 3300.0 && offset < 3500.0, "Doppler magnitude/sign correct (~+3404 Hz)");
    CHECK(PassPredict::dopplerShiftedHz(145.8e6, +7.0) < 145.8e6, "receding shifts down");

    // Conjunction: a satellite against itself must have ~zero closest approach.
    auto selfCj = PassPredict::closestApproach(sgp4, sgp4, start, 24.0);
    CHECK(selfCj.valid && selfCj.minRangeKm < 0.001, "self-conjunction min range ~ 0 km");
    CHECK(selfCj.tca >= start && selfCj.tca <= start.AddSeconds(24.0 * 3600.0),
          "self-conjunction TCA lies within the window");

    // Conjunction between two distinct objects must be valid and physically plausible.
    Tle hst("HST",
        "1 20580U 90037B   24153.45833333  .00000900  00000-0  44000-4 0  9990",
        "2 20580  28.4700  10.0000 0002800 100.0000 260.0000 15.09000000 12345");
    SGP4 sgp4b(hst);
    auto crossCj = PassPredict::closestApproach(sgp4, sgp4b, start, 24.0);
    CHECK(crossCj.valid && crossCj.minRangeKm > 1.0 && crossCj.minRangeKm < 50000.0,
          "ISS/HST closest approach is positive and within Earth-orbit scale");

    std::printf("\n%s (%d failure%s)\n", failures ? "TESTS FAILED" : "ALL TESTS PASSED",
                failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
