#ifndef FETCH_TLE_HPP
#define FETCH_TLE_HPP

#include <array>
#include <string>
#include <vector>
#include "SGP4.h"

namespace FetchTLE {

    bool validateTLE(const std::string& tle);
    std::string fetchTLE(const std::string& noradID);
    libsgp4::Tle buildTle(const std::string& raw);

    // Geodetic sub-satellite point {latitude_deg, longitude_deg, altitude_km}.
    // Longitude is geographic East-longitude (SGP4 ToGeodetic), ready for geoToWorld().
    std::array<float, 3> getSubPoint(const libsgp4::SGP4& sgp4);
    std::vector<std::array<float, 3>> getFutureSubPoints(const libsgp4::SGP4& sgp4,
                                                         int numPoints, double intervalMinutes);

    // Current ECI (TEME) state vector {x,y,z (km), vx,vy,vz (km/s)} — seeds the Kepler propagator.
    std::array<double, 6> getStateVector(const libsgp4::SGP4& sgp4);

}

#endif