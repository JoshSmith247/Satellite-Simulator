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

    // Fetch a TLE, transparently using an on-disk cache: a successful fetch is written
    // to the cache; if the network fails (or returns garbage), the last good cached TLE
    // for that NORAD ID is returned instead. `fromCache` (if given) reports which path
    // was taken. Returns empty only when both network and cache fail.
    std::string fetchTLECached(const std::string& noradID, bool* fromCache = nullptr);

    // One entry in a browsable satellite catalog.
    struct SatEntry {
        std::string name;   // object name (TLE line 0)
        std::string id;     // NORAD catalog number
    };

    // Fetch a whole CelesTrak group catalog (e.g. "active", "visual", "stations") as raw
    // multi-TLE text, with the same on-disk cache/fallback behaviour as fetchTLECached.
    // Cached under tle_cache/group_<group>.tle.
    std::string fetchGroupCached(const std::string& group, bool* fromCache = nullptr);

    // Parse raw multi-TLE catalog text into {name, NORAD id} entries (skips malformed records).
    std::vector<SatEntry> parseCatalog(const std::string& raw);

    // Geodetic sub-satellite point {latitude_deg, longitude_deg, altitude_km} at `when`.
    // Longitude is geographic East-longitude (SGP4 ToGeodetic), ready for geoToWorld().
    std::array<float, 3> getSubPoint(const libsgp4::SGP4& sgp4, const libsgp4::DateTime& when);
    std::vector<std::array<float, 3>> getFutureSubPoints(const libsgp4::SGP4& sgp4,
                                                         int numPoints, double intervalMinutes,
                                                         const libsgp4::DateTime& start);

    // ECI (TEME) state vector {x,y,z (km), vx,vy,vz (km/s)} at `when` — seeds the Kepler propagator.
    std::array<double, 6> getStateVector(const libsgp4::SGP4& sgp4, const libsgp4::DateTime& when);

}

#endif