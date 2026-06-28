// Unit tests for TLE validation/parsing (no network is exercised here).
#include "fetchTLE.hpp"
#include "Tle.h"
#include <string>
#include <cmath>
#include <cstdio>

static int failures = 0;
#define CHECK(cond, msg) do { \
    if (!(cond)) { std::printf("FAIL: %s\n", msg); ++failures; } \
    else         { std::printf("ok:   %s\n", msg); } } while (0)

int main() {
    const std::string good =
        "ISS (ZARYA)\n"
        "1 25544U 98067A   24153.51782528  .00016717  00000-0  30074-3 0  9993\n"
        "2 25544  51.6398 211.6055 0004168  86.9505 273.2003 15.49960977 12345\n";

    CHECK(FetchTLE::validateTLE(good),                       "valid 3-line TLE accepted");
    CHECK(!FetchTLE::validateTLE(""),                        "empty string rejected");
    CHECK(!FetchTLE::validateTLE("just one line"),           "single line rejected");
    CHECK(!FetchTLE::validateTLE("name\nfoo\nbar"),          "missing line-1/line-2 markers rejected");

    // CRLF line endings should still validate (CelesTrak/Windows files).
    CHECK(FetchTLE::validateTLE(
        "ISS\r\n1 25544U 98067A   24153.51782528  .00016717  00000-0  30074-3 0  9993\r\n"
        "2 25544  51.6398 211.6055 0004168  86.9505 273.2003 15.49960977 12345\r\n"),
        "CRLF line endings accepted");

    // buildTle parses the three lines into a usable element set.
    libsgp4::Tle tle = FetchTLE::buildTle(good);
    CHECK(tle.Name() == "ISS (ZARYA)",                      "buildTle preserves the name line");
    CHECK(std::fabs(tle.Inclination(true) - 51.6398) < 1e-3, "buildTle parses inclination");

    std::printf("\n%s (%d failure%s)\n", failures ? "TESTS FAILED" : "ALL TESTS PASSED",
                failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
