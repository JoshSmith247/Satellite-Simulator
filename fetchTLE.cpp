#include "fetchTLE.hpp"
#include <array>
#include <vector>
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <filesystem>
#include <system_error>
#include <cctype>
#include <curl/curl.h>
#include "Tle.h"
#include "SGP4.h"
#include "Eci.h"
#include "CoordGeodetic.h"
#include "DateTime.h"
#include "Vector.h"

using namespace libsgp4;

namespace FetchTLE {

    // libcurl write callback: appends response chunks to the buffer.
    static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* userp) {
        size_t totalSize = size * nmemb;
        userp->append((char*)contents, totalSize);
        return totalSize;
    }

    // A well-formed TLE needs a line starting with '1' and one starting with '2'.
    bool validateTLE(const std::string& tle) {
        if (tle.empty()) return false;

        int line1Found = 0, line2Found = 0;
        std::istringstream stream(tle);
        std::string line;

        while (std::getline(stream, line)) {
            if (!line.empty() && line.back() == '\r')
                line.pop_back();

            if (line.size() > 0 && line[0] == '1') line1Found = 1;
            if (line.size() > 0 && line[0] == '2') line2Found = 1;
        }

        return line1Found && line2Found;
    }

    // Fetch TLE text for a NORAD ID from the CelesTrak GP API; empty on failure.
    std::string fetchTLE(const std::string& noradID) {
        CURL* curl;
        CURLcode res;
        std::string readBuffer;
        std::string url = "https://celestrak.org/NORAD/elements/gp.php?CATNR=" + noradID + "&FORMAT=tle";

        curl = curl_easy_init();
        if (curl) {
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            // CelesTrak requires a User-Agent header identifying the client.
            curl_easy_setopt(curl, CURLOPT_USERAGENT, "SatelliteSim/1.0");
            curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
            curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10L);
            curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 5L);
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);

            res = curl_easy_perform(curl);

            if (res != CURLE_OK) {
                std::cerr << "CURL Error: " << curl_easy_strerror(res) << std::endl;
                readBuffer.clear();
            } else {
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

    // Geodetic sub-points sampled forward from `start`.
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

    static std::string cachePath(const std::string& noradID) {
        return "tle_cache/" + noradID + ".tle";
    }

    std::string fetchTLECached(const std::string& noradID, bool* fromCache) {
        if (fromCache) *fromCache = false;

        std::string raw = fetchTLE(noradID);
        if (validateTLE(raw)) {
            // Persist the fresh TLE for offline use.
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

    // Same curl setup as fetchTLE() but a longer timeout — the "active" catalog is a few MB.
    static std::string fetchGroup(const std::string& group) {
        CURL* curl;
        std::string readBuffer;
        std::string url = "https://celestrak.org/NORAD/elements/gp.php?GROUP=" + group + "&FORMAT=tle";

        curl = curl_easy_init();
        if (curl) {
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_USERAGENT, "SatelliteSim/1.0");
            curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
            curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30L);
            curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 5L);
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);

            CURLcode res = curl_easy_perform(curl);
            if (res != CURLE_OK) {
                std::cerr << "CURL Error (group): " << curl_easy_strerror(res) << std::endl;
                readBuffer.clear();
            } else {
                long httpCode = 0;
                curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &httpCode);
                if (httpCode != 200) {
                    std::cerr << "HTTP Error (group): " << httpCode << std::endl;
                    readBuffer.clear();
                }
            }
            curl_easy_cleanup(curl);
        }
        return readBuffer;
    }

    std::string fetchGroupCached(const std::string& group, bool* fromCache) {
        if (fromCache) *fromCache = false;
        std::string path = "tle_cache/group_" + group + ".tle";

        std::string raw = fetchGroup(group);
        if (validateTLE(raw)) {
            std::error_code ec;
            std::filesystem::create_directories("tle_cache", ec);
            std::ofstream out(path, std::ios::binary | std::ios::trunc);
            if (out) out << raw;
            return raw;
        }

        std::ifstream in(path, std::ios::binary);
        if (in) {
            std::stringstream ss;
            ss << in.rdbuf();
            std::string cached = ss.str();
            if (validateTLE(cached)) {
                std::cerr << "Using cached catalog for group " << group
                          << " (network fetch failed)" << std::endl;
                if (fromCache) *fromCache = true;
                return cached;
            }
        }
        return std::string();
    }

    std::vector<SatEntry> parseCatalog(const std::string& raw) {
        // Collect non-empty, CR-stripped lines, then group into 3-line records.
        std::vector<std::string> lines;
        std::istringstream stream(raw);
        std::string line;
        while (std::getline(stream, line)) {
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (!line.empty()) lines.push_back(line);
        }

        auto idFromLine1 = [](const std::string& l1) -> std::string {
            size_t i = 1;
            while (i < l1.size() && !std::isdigit((unsigned char)l1[i])) i++;
            std::string id;
            while (i < l1.size() && std::isdigit((unsigned char)l1[i])) id += l1[i++];
            return id;
        };
        auto trim = [](const std::string& s) -> std::string {
            size_t a = s.find_first_not_of(" \t");
            size_t b = s.find_last_not_of(" \t");
            return (a == std::string::npos) ? std::string() : s.substr(a, b - a + 1);
        };

        std::vector<SatEntry> entries;
        for (size_t i = 0; i + 1 < lines.size(); i++) {
            if (lines[i][0] == '1' && lines[i + 1][0] == '2' && i >= 1) {
                std::string id = idFromLine1(lines[i]);
                std::string name = trim(lines[i - 1]);
                if (!id.empty() && !name.empty()) entries.push_back({ name, id });
            }
        }
        return entries;
    }

} // namespace FetchTLE