#include "fetchTLE.hpp"
#include <iostream>
#include <sstream>
#include <string>
#include <cmath>
#include <curl/curl.h>
#include "Tle.h"
#include "SGP4.h"
#include "Eci.h"
#include "DateTime.h"
#include "Vector.h"
#include "raylib.h"

using namespace libsgp4;

namespace FetchTLE {

    /**
     * @brief Callback function to handle data received from CelesTrak.
     * libcurl calls this function as it receives chunks of data.
     */
    size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* userp) {
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

    TLEData parseTLE(const std::string& raw) {
        std::istringstream stream(raw);
        std::string line0, line1, line2;

        std::getline(stream, line0);
        std::getline(stream, line1);
        std::getline(stream, line2);

        // Strip carriage returns
        auto strip = [](std::string& s) {
            if (!s.empty() && s.back() == '\r') s.pop_back();
        };
        strip(line0); strip(line1); strip(line2);

        TLEData tle;
        tle.name         = line0;
        tle.epoch        = std::stod(line1.substr(18, 14));
        tle.inclination  = std::stod(line2.substr(8,  8));
        tle.raan         = std::stod(line2.substr(17, 8));
        tle.eccentricity = std::stod("0." + line2.substr(26, 7)); // implied decimal
        tle.argPerigee   = std::stod(line2.substr(34, 8));
        tle.meanAnomaly  = std::stod(line2.substr(43, 8));
        tle.meanMotion   = std::stod(line2.substr(52, 11));

        return tle;
    }

    Vector3 getScenePosition(const std::string& rawTLE) {
        // Split the raw string into the three TLE lines
        std::istringstream stream(rawTLE);
        std::string line0, line1, line2;
        std::getline(stream, line0);
        std::getline(stream, line1);
        std::getline(stream, line2);

        // Strip carriage returns
        auto strip = [](std::string& s) {
            if (!s.empty() && s.back() == '\r') s.pop_back();
        };
        strip(line0); strip(line1); strip(line2);

        // Build the SGP4 Tle object and propagator
        Tle tle(line0, line1, line2);
        SGP4 sgp4(tle);

        // Get current UTC time
        DateTime now = DateTime::Now(true);

        // Propagate — returns ECI position in km
        Eci eci = sgp4.FindPosition(now);
        Vector eciPos = eci.Position();

        // Rotate ECI → ECEF using GMST
        double gmst = now.ToGreenwichSiderealTime();
        double ecef_x = eciPos.x * cos(gmst) + eciPos.y * sin(gmst);
        double ecef_y = -eciPos.x * sin(gmst) + eciPos.y * cos(gmst);
        double ecef_z = eciPos.z;

        // Scale from km to scene units (Earth radius = 6371km → 5.0f in scene)
        const float SCALE = 5.0f / 6371.0f;

        // Remap axes: ECEF X→X, Z→Y (up in Raylib), Y→Z
        return Vector3{
            (float)(ecef_x * SCALE),
            (float)(ecef_z * SCALE),
            (float)(ecef_y * SCALE)
        };
    }

} // namespace FetchTLE