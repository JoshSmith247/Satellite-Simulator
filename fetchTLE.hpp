#ifndef FETCH_TLE_HPP
#define FETCH_TLE_HPP

#include <cstddef>
#include <string>
#include "SGP4.h"
#include "Tle.h"
#include "DateTime.h"
#include "raylib.h"

struct TLEData {
    std::string name;
    double epoch;
    double inclination;
    double raan;
    double eccentricity;
    double argPerigee;
    double meanAnomaly;
    double meanMotion;
};

namespace FetchTLE {

    size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* userp);
    bool validateTLE(const std::string& tle);
    std::string fetchTLE(const std::string& noradID);
    TLEData parseTLE(const std::string& raw);
    Vector3 getScenePosition(const std::string& rawTLE);

}

#endif