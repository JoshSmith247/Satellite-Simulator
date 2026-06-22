#include "fetchTLE.hpp"
#include <array>
#include <vector>
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <filesystem>
#include <system_error>
#include <curl/curl.h>
#include "Tle.h"
#include "SGP4.h"
#include "Eci.h"
#include "CoordGeodetic.h"
#include "DateTime.h"
#include "Vector.h"

using namespace libsgp4;

namespace FetchTLE {

    /**
     * @brief Callback function to handle data received from CelesTrak.
     * libcurl calls this function as it receives chunks of data.
     */
    static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* userp) {
        size_t totalSize = size * nmemb;
        userp->append((char*)contents, totalSize);
        return totalSize;
    }

    /**
     * @brief Validates that a string contains a well-formed TLE (3-line format).
     * Checks for a name line, a line starting with '1', and a line starting with '2'.
     * @param tle The raw TLE string to validate.
     * @return true if the TLE appears valid, false otherwise.
     */
    bool validateTLE(const std::string& tle) {
        if (tle.empty()) return false;

        int line1Found = 0, line2Found = 0;
        std::istringstream stream(tle);
        std::string line;

        while (std::getline(stream, line)) {
            // Strip carriage returns in case of Windows-style line endings
            if (!line.empty() && line.back() == '\r')
                line.pop_back();

            if (line.size() > 0 && line[0] == '1') line1Found = 1;
            if (line.size() > 0 && line[0] == '2') line2Found = 1;
        }

        return line1Found && line2Found;
    }

    /**
     * @brief Fetches TLE data for a given NORAD Catalog Number.
     * @param noradID The catalog ID (e.g., 25544 for ISS)
     * @return std::string The raw TLE text or an empty string on failure.
     */
    std::string fetchTLE(const std::string& noradID) {
        CURL* curl;
        CURLcode res;
        std::string readBuffer;

        // Construct the CelesTrak GP (General Perturbations) API URL
        std::string url = "https://celestrak.org/NORAD/elements/gp.php?CATNR=" + noradID + "&FORMAT=tle";

        curl = curl_easy_init();
        if (curl) {
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());

            // CelesTrak requests require a User-Agent header to identify the client
            curl_easy_setopt(curl, CURLOPT_USERAGENT, "SatelliteSim/1.0");

            // Follow redirects if the URL changes
            curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);

            // Timeout settings to prevent hanging if CelesTrak is unreachable
            curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10L);
            curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 5L);

            // Set up the callback to capture the response
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);

            // Perform the request
            res = curl_easy_perform(curl);

            if (res != CURLE_OK) {
                std::cerr << "CURL Error: " << curl_easy_strerror(res) << std::endl;
                readBuffer.clear();
            } else {
                // Check the HTTP response code — CelesTrak returns 200 even for bad IDs,
                // but we still want to catch non-200 responses (rate limits, server errors, etc.)
                long httpCode = 0;
                curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &httpCode);
                if (httpCode != 200) {
                    std::cerr << "HTTP Error: " << httpCode << std::endl;
                    readBuffer.clear();
                }
            }

            curl_easy_cleanup(curl);
        }

        return readBuffer;
    }

    Tle buildTle(const std::string& raw) {
        std::istringstream stream(raw);
        std::string line0, line1, line2;
        std::getline(stream, line0);
        std::getline(stream, line1);
        std::getline(stream, line2);
        auto strip = [](std::string& s) {
            if (!s.empty() && s.back() == '\r') s.pop_back();
        };
        strip(line0); strip(line1); strip(line2);
        return Tle(line0, line1, line2);
    }

    static constexpr double RAD2DEG = 57.29577951308232;

    // Geodetic sub-satellite point at the given instant.
    std::array<float, 3> getSubPoint(const libsgp4::SGP4& sgp4, const DateTime& when) {
        Eci eci = sgp4.FindPosition(when);
        CoordGeodetic geo = eci.ToGeodetic();
        return { (float)(geo.latitude * RAD2DEG),
                 (float)(geo.longitude * RAD2DEG),
                 (float)geo.altitude };
    }

    // Geodetic sub-points sampled forward from `start` (for the ground-track prediction).
    std::vector<std::array<float, 3>> getFutureSubPoints(const libsgp4::SGP4& sgp4,
                                                         int numPoints, double intervalMinutes,
                                                         const DateTime& start) {
        std::vector<std::array<float, 3>> points;
        points.reserve(numPoints);
        for (int i = 0; i < numPoints; i++) {
            try {
                Eci eci = sgp4.FindPosition(start.AddMinutes(i * intervalMinutes));
                CoordGeodetic geo = eci.ToGeodetic();
                points.push_back({ (float)(geo.latitude * RAD2DEG),
                                   (float)(geo.longitude * RAD2DEG),
                                   (float)geo.altitude });
            } catch (...) {
                break; // satellite decayed or propagation out of range
            }
        }
        return points;
    }

    std::array<double, 6> getStateVector(const libsgp4::SGP4& sgp4, const DateTime& when) {
        Eci eci = sgp4.FindPosition(when);
        Vector p = eci.Position();
        Vector v = eci.Velocity();
        return { p.x, p.y, p.z, v.x, v.y, v.z };
    }

    // Filesystem-friendly cache path for a NORAD ID's last good TLE.
    static std::string cachePath(const std::string& noradID) {
        return "tle_cache/" + noradID + ".tle";
    }

    std::string fetchTLECached(const std::string& noradID, bool* fromCache) {
        if (fromCache) *fromCache = false;

        std::string raw = fetchTLE(noradID);
        if (validateTLE(raw)) {
            // Persist the fresh TLE for offline use next time.
            std::error_code ec;
            std::filesystem::create_directories("tle_cache", ec);
            std::ofstream out(cachePath(noradID), std::ios::binary | std::ios::trunc);
            if (out) out << raw;
            return raw;
        }

        // Network failed or returned junk — fall back to the cached copy.
        std::ifstream in(cachePath(noradID), std::ios::binary);
        if (in) {
            std::stringstream ss;
            ss << in.rdbuf();
            std::string cached = ss.str();
            if (validateTLE(cached)) {
                std::cerr << "Using cached TLE for NORAD " << noradID
                          << " (network fetch failed)" << std::endl;
                if (fromCache) *fromCache = true;
                return cached;
            }
        }
        return std::string();
    }

} // namespace FetchTLE