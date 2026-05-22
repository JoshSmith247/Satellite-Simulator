#ifndef FETCH_TLE_HPP
#define FETCH_TLE_HPP

#include <array>
#include <string>
#include "SGP4.h"

namespace FetchTLE {

    bool validateTLE(const std::string& tle);
    std::string fetchTLE(const std::string& noradID);
    libsgp4::Tle buildTle(const std::string& raw);
    std::array<float, 3> getScenePosition(const libsgp4::SGP4& sgp4);

}

#endif