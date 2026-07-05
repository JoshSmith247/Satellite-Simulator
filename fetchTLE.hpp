#ifndef FETCH_TLE_HPP
#define FETCH_TLE_HPP

#include <array>
#include <string>
#include <vector>
#include "SGP4.h"
#include "DateTime.h"

namespace FetchTLE {

    bool validateTLE(const std::string& tle);
    std::string fetchTLE(const std::string& noradID);
    libsgp4::Tle buildTle(const std::string& raw);

    // Fetch a TLE with on-disk cache fallback: good fetches are cached, and the last
    // cached TLE is returned when the network fails. Empty only if both fail.
    std::string fetchTLECached(const std::string& noradID, bool* fromCache = nullptr);

    struct SatEntry {
        std::string name;   // object name (TLE line 0)
        std::string id;     // NORAD catalog number
    };

    // CelesTrak group catalog (e.g. "active") with the same cache fallback as fetchTLECached.
    std::string fetchGroupCached(const std::string& group, bool* fromCache = nullptr);

    // Parse raw multi-TLE catalog text into entries, skipping malformed records.
    std::vector<SatEntry> parseCatalog(const std::string& raw);

    // Geodetic sub-satellite point {latitude_deg, longitude_deg, altitude_km} at `when`.
    std::array<float, 3> getSubPoint(const libsgp4::SGP4& sgp4, const libsgp4::DateTime& when);
    std::vector<std::array<float, 3>> getFutureSubPoints(const libsgp4::SGP4& sgp4,
                                                         int numPoints, double intervalMinutes,
                                                         const libsgp4::DateTime& start);

    // ECI state vector {x,y,z (km), vx,vy,vz (km/s)} at `when`; seeds the Kepler propagator.
    std::array<double, 6> getStateVector(const libsgp4::SGP4& sgp4, const libsgp4::DateTime& when);

}

#endif