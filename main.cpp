#include "raylib.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#include "raymath.h"
#include "rlgl.h"
#include "EarthMath.hpp"
#include "SunMath.hpp"
#include "fetchTLE.hpp"
#include "OrbitalMechanics.hpp"
#include "SimClock.hpp"
#include "PassPredict.hpp"
#include "SGP4.h"
#include "Tle.h"
#include "Eci.h"
#include "CoordGeodetic.h"
#include "DateTime.h"
#include "Vector.h"
#include <exception>
#include <memory>
#include <string>
#include <vector>
#include <array>
#include <algorithm>
#include <cstring>
#include <cstdlib>
#include <cctype>
#include <cmath>
#include <future>
#include <chrono>
#include <fstream>
#include <filesystem>
#include <system_error>

// ── Asynchronous satellite loading ────────────────────────────────────────────
// Network fetch + TLE parse run on a worker thread so the window never freezes.
// The result carries shared_ptrs that the main thread swaps in when ready.
struct SatLoadResult {
    bool        ok        = false;
    bool        fromCache = false;
    std::string id;
    std::string status;
    std::shared_ptr<libsgp4::Tle>  tle;
    std::shared_ptr<libsgp4::SGP4> sgp4;
};

static SatLoadResult loadSatelliteData(std::string noradID) {
    SatLoadResult r;
    r.id = noradID;
    bool fromCache = false;
    std::string raw = FetchTLE::fetchTLECached(noradID, &fromCache);
    if (!FetchTLE::validateTLE(raw)) {
        r.status = "Load failed: " + noradID;
        return r;
    }
    try {
        r.tle  = std::make_shared<libsgp4::Tle>(FetchTLE::buildTle(raw));
        r.sgp4 = std::make_shared<libsgp4::SGP4>(*r.tle);
    } catch (const std::exception& e) {
        r.status = std::string("Bad TLE: ") + e.what();
        return r;
    }
    r.ok        = true;
    r.fromCache = fromCache;
    r.status    = (fromCache ? "Loaded (cached): " : "Loaded: ") + r.tle->Name();
    return r;
}

// Result of an asynchronous browsable-catalog fetch (CelesTrak group).
struct CatalogResult {
    bool        ok = false;
    std::string status;
    std::vector<FetchTLE::SatEntry> entries;
};

static CatalogResult loadCatalogData(std::string group) {
    CatalogResult r;
    bool fromCache = false;
    std::string raw = FetchTLE::fetchGroupCached(group, &fromCache);
    r.entries = FetchTLE::parseCatalog(raw);
    if (r.entries.empty()) {
        r.status = "Catalog fetch failed";
        return r;
    }
    r.ok = true;
    r.status = (fromCache ? "Loaded (cached) " : "Loaded ") +
               std::to_string(r.entries.size()) + " satellites";
    return r;
}

// One tracked satellite. Every open tab carries its own propagator + track
// history so background tabs keep simulating while only the active one is drawn.
struct SatTab {
    std::string id;
    std::shared_ptr<libsgp4::Tle>  tle;
    std::shared_ptr<libsgp4::SGP4> sgp4;
    bool        tleFromCache = false;
    std::string loadStatus   = "Loading...";
    float       futureInterval = 1.0f;               // minutes between ground-track samples
    std::vector<Vector3> observedPath, futurePath, keplerPath;
    std::vector<std::array<float, 2>> futureGeo;     // raw {lat,lon} for the 2D map
    float  subLat = 0.0f, subLon = 0.0f, subAlt = 0.0f;
    double lastRecordSim = -1e18;
    bool   recordBreak = false;
    bool   forcePassRecompute = true;
    float  futureTimer = 1e9f;                        // large => recompute on first frame loaded
    float  passRecomputeTimer = 0.0f;
    OrbitalMechanics::Elements     kepEl;
    std::vector<PassPredict::Pass> passes;
    std::future<SatLoadResult>     loadFuture;        // in-flight network load (per tab)

    // Per-tab view/planning state — what's open is remembered separately for each tab.
    bool   showMap      = false;
    bool   showHohmann  = false;
    double targetAltKm  = 1000.0;                     // Hohmann target circular-orbit altitude
    char   targetAltBuf[16] = "1000";
    bool   targetAltEdit = false;
};

static Color stationPalette(int i) {
    static const Color pal[] = {
        {255, 221,  51, 255}, { 80, 220, 255, 255}, {255, 140,  80, 255},
        {160, 120, 255, 255}, {120, 255, 140, 255}, {255, 110, 180, 255},
    };
    return pal[i % (int)(sizeof(pal) / sizeof(pal[0]))];
}

int main()
{
    // ── Ground stations (runtime-editable) ────────────────────────────────────
    struct GroundStation { std::string name; float lat, lon, altKm, maskDeg; Color color; };
    std::vector<GroundStation> stations = {
        { "WUSat", 38.627f, -90.199f, 0.142f, 5.0f, stationPalette(0) },
        { "0N/0E",  0.0f,    0.0f,    0.0f,   5.0f, stationPalette(1) },
    };
    int selectedStation = 0;            // observer used for pass prediction / telemetry

    // ── Radio link (for Doppler) ──────────────────────────────────────────────
    double downlinkHz = 145.800e6;      // ISS APRS/voice default; edit per CubeSat

    // ── Satellite tabs ────────────────────────────────────────────────────────
    // Each tab is one tracked satellite. Only the active tab is drawn, but every
    // tab keeps simulating in the background so each keeps a continuous history.
    std::string exportStatus;
    const int   FUTURE_POINTS = 360;
    const float TAB_BAR_H     = 30.0f;
    const int   MAX_TABS      = 7;

    std::vector<SatTab> tabs;
    int  activeTab = -1;                 // -1 ⇒ no tabs ⇒ empty-state screen
    bool conjDirty = true;               // conjunction recompute needed (set on tab add/close)

    auto addTab = [&](const std::string& id) {
        if ((int)tabs.size() >= MAX_TABS) { exportStatus = "Tab limit reached (max 7)"; return; }
        SatTab t;
        t.id         = id;
        t.loadStatus = "Loading...";
        t.loadFuture = std::async(std::launch::async, loadSatelliteData, id);
        tabs.push_back(std::move(t));
        activeTab = (int)tabs.size() - 1;
        conjDirty = true;
    };
    auto closeTab = [&](int i) {
        if (i < 0 || i >= (int)tabs.size()) return;
        tabs.erase(tabs.begin() + i);
        conjDirty = true;
        if (tabs.empty())                       activeTab = -1;
        else if (activeTab > i)                 activeTab--;          // keep same tab active
        else if (activeTab >= (int)tabs.size()) activeTab = (int)tabs.size() - 1;
    };
    // Observer/time changes invalidate every tab's predictions, not just the active one.
    auto markAllPassRecompute = [&]{ for (auto& t : tabs) t.forcePassRecompute = true; };
    auto markAllRecordBreak   = [&]{ for (auto& t : tabs) t.recordBreak        = true; };

    addTab("25544");                     // ISS (Zarya) — default tab on startup

    // ── Simulation clock ──────────────────────────────────────────────────────
    SimClock clock;

    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(0, 0, "Satellite Orbit Sim");
    ToggleFullscreen();

    Camera3D camera = {0};
    camera.position   = (Vector3){20.0f, 20.0f, 20.0f};
    camera.target     = (Vector3){0.0f, 0.0f, 0.0f};
    camera.up         = (Vector3){0.0f, 1.0f, 0.0f};
    camera.fovy       = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    SetTargetFPS(60);

    // ── Earth ────────────────────────────────────────────────────────────────
    Image earthImage = LoadImage("earth_tx.jpg");
    if (earthImage.data == NULL)
        TraceLog(LOG_ERROR, "Failed to load earth_tx.jpg");
    Texture2D earthTexture = LoadTextureFromImage(earthImage);
    SetTextureFilter(earthTexture, TEXTURE_FILTER_TRILINEAR);
    SetTextureWrap(earthTexture, TEXTURE_WRAP_CLAMP);
    UnloadImage(earthImage);

    Model earthModel = LoadModel("earth_sphere.obj");
    if (earthModel.meshCount == 0)
        TraceLog(LOG_ERROR, "Failed to load earth_sphere.obj");
    earthModel.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = earthTexture;

    const float EARTH_RADIUS        = 5.0f;
    const float SUN_VISUAL_DISTANCE = 2000.0f;
    const float SUN_RADIUS          = EARTH_RADIUS * 109.0f / 200.0f;

    // ── Skybox ───────────────────────────────────────────────────────────────
    Texture2D skyTexture = LoadTexture("stars_tx.jpg");
    SetTextureFilter(skyTexture, TEXTURE_FILTER_BILINEAR);

    Mesh  skySphere = GenMeshSphere(1.0f, 96, 96);
    Model skybox    = LoadModelFromMesh(skySphere);
    skybox.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = skyTexture;

    Shader skyShader = LoadShader("skybox.vs", "skybox.fs");
    skybox.materials[0].shader = skyShader;

    // ── Lighting shader (Earth) ──────────────────────────────────────────────
    Shader shader       = LoadShader("lighting.vs", "lighting.fs");
    int lightPosLoc     = GetShaderLocation(shader, "lightPos");
    int ambientLoc      = GetShaderLocation(shader, "ambient");
    int viewPosLoc      = GetShaderLocation(shader, "viewPos");

    float lightIntensity = 1.2f;
    Vector3 ambient      = Vector3Scale({0.08f, 0.08f, 0.10f}, lightIntensity);
    SetShaderValue(shader, ambientLoc, &ambient, SHADER_UNIFORM_VEC3);
    earthModel.materials[0].shader = shader;

    // ── Satellite shader ──────────────────────────────────────────────────────
    Shader satShader    = LoadShader("lighting.vs", "sat_lighting.fs");
    int satLightPosLoc  = GetShaderLocation(satShader, "lightPos");
    int satAmbientLoc   = GetShaderLocation(satShader, "ambient");
    int satViewPosLoc   = GetShaderLocation(satShader, "viewPos");
    SetShaderValue(satShader, satAmbientLoc, &ambient, SHADER_UNIFORM_VEC3);

    // ── Bloom pipeline ────────────────────────────────────────────────────────
    Shader brightPassShader = LoadShader(0, "bloom_bright.fs");
    Shader blurShader       = LoadShader(0, "bloom_blur.fs");
    int bloomThresholdLoc   = GetShaderLocation(brightPassShader, "threshold");
    int blurTexelSizeLoc    = GetShaderLocation(blurShader, "texelSize");
    int blurHorizontalLoc   = GetShaderLocation(blurShader, "horizontal");
    float bloomThreshold    = 0.72f;
    SetShaderValue(brightPassShader, bloomThresholdLoc, &bloomThreshold, SHADER_UNIFORM_FLOAT);

    // Render targets sized to the framebuffer; rebuilt on window resize so the
    // scene/bloom buffers always match the (possibly HiDPI) render resolution.
    RenderTexture2D sceneTarget = {0}, brightTarget = {0}, blurTargetA = {0}, blurTargetB = {0};
    int       bloomW = 0, bloomH = 0;
    Rectangle fullSrc{}, halfSrc{}, halfDst{};
    bool      targetsReady = false;

    auto rebuildTargets = [&](int w, int h) {
        if (targetsReady) {
            UnloadRenderTexture(sceneTarget);
            UnloadRenderTexture(brightTarget);
            UnloadRenderTexture(blurTargetA);
            UnloadRenderTexture(blurTargetB);
        }
        bloomW = (w > 1) ? w / 2 : 1;
        bloomH = (h > 1) ? h / 2 : 1;
        sceneTarget  = LoadRenderTexture(w, h);
        brightTarget = LoadRenderTexture(bloomW, bloomH);
        blurTargetA  = LoadRenderTexture(bloomW, bloomH);
        blurTargetB  = LoadRenderTexture(bloomW, bloomH);
        Vector2 texel = {1.0f / (float)bloomW, 1.0f / (float)bloomH};
        SetShaderValue(blurShader, blurTexelSizeLoc, &texel, SHADER_UNIFORM_VEC2);
        fullSrc = {0, 0, (float)w,      -(float)h};
        halfSrc = {0, 0, (float)bloomW, -(float)bloomH};
        halfDst = {0, 0, (float)bloomW,  (float)bloomH};
        targetsReady = true;
    };
    rebuildTargets(GetRenderWidth(), GetRenderHeight());

    // ── Orbit data ─────────────────────────────────────────────────────────────
    const float FUTURE_UPDATE_S = 5.0f;

    Model satModel = LoadModel("satellite.obj");
    satModel.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = GRAY;
    satModel.materials[0].shader = satShader;

    // ── Fonts ────────────────────────────────────────────────────────────────
    Font font      = LoadFontEx("Roboto-Med.ttf",      96, 0, 0);
    Font font_bold = LoadFontEx("Montserrat-Bold.ttf", 96, 0, 0);
    if (font.texture.id == 0 || font_bold.texture.id == 0) {
        TraceLog(LOG_ERROR, "Failed to load fonts; using default.");
        font      = GetFontDefault();
        font_bold = GetFontDefault();
    } else {
        GenTextureMipmaps(&font.texture);
        GenTextureMipmaps(&font_bold.texture);
        SetTextureFilter(font.texture,      TEXTURE_FILTER_TRILINEAR);
        SetTextureFilter(font_bold.texture, TEXTURE_FILTER_TRILINEAR);
    }

    // ── Add-satellite UI state ──────────────────────────────────────────────────
    char addInput[16]    = {0};
    bool addEditMode     = false;
    bool showAddOverlay  = false;        // '+' opens the search box over the scene

    // Procedural satellite + question-mark icon for the add UI (no asset needed).
    auto drawSatQuestionIcon = [&](float cx, float cy) {
        Color body = (Color){150,160,180,255}, panel = (Color){70,110,180,255};
        DrawRectangle((int)(cx-16),(int)(cy-12), 32, 24, body);
        DrawRectangle((int)(cx-46),(int)(cy-8),  24, 16, panel);
        DrawRectangle((int)(cx+22),(int)(cy-8),  24, 16, panel);
        DrawLineEx({cx-22,cy},{cx-16,cy}, 2, body);
        DrawLineEx({cx+16,cy},{cx+22,cy}, 2, body);
        Vector2 q = MeasureTextEx(font_bold, "?", 30, 1);
        DrawTextEx(font_bold, "?", {cx - q.x*0.5f, cy - q.y*0.5f - 1.0f}, 30, 1, RAYWHITE);
    };

    // ── Browse-catalog UI state ─────────────────────────────────────────────────
    bool showCatalog    = false;
    char catFilter[32]  = {0};
    bool catFilterEdit  = false;
    int  catScroll = 0, catActive = -1;
    std::string catStatus = "Loading catalog...";
    std::vector<FetchTLE::SatEntry> catalog;          // full fetched catalog
    std::future<CatalogResult>      catFuture;
    // Filtered view rebuilt only when the filter text (or catalog) changes.
    std::string lastFilter = "\x01";                  // sentinel ⇒ forces first rebuild
    std::vector<std::string> filterLabels;            // "<name>   <id>" backing strings
    std::vector<std::string> filterIds;               // NORAD id parallel to filterLabels
    std::vector<char*>       filterPtrs;              // pointers into filterLabels (for GuiListViewEx)

    auto openCatalog = [&]() {
        showCatalog = true;
        if (catalog.empty() && !catFuture.valid())
            catFuture = std::async(std::launch::async, loadCatalogData, std::string("active"));
    };
    auto rebuildFilter = [&]() {
        auto lower = [](std::string s) { for (char& c : s) c = (char)std::tolower((unsigned char)c); return s; };
        std::string needle = lower(catFilter);
        filterLabels.clear(); filterIds.clear(); filterPtrs.clear();
        for (const auto& e : catalog) {
            if (needle.empty() || lower(e.name + " " + e.id).find(needle) != std::string::npos) {
                filterLabels.push_back(e.name + "   " + e.id);
                filterIds.push_back(e.id);
            }
        }
        filterPtrs.reserve(filterLabels.size());
        for (auto& s : filterLabels) filterPtrs.push_back(s.data());
        catScroll = 0; catActive = -1;
        lastFilter = catFilter;
    };
    // Quick-access categories: each just sets the filter over the loaded active catalog,
    // so "Starlink" yields all currently-active Starlink satellites. Easy to extend.
    struct Category { const char* label; const char* filter; };
    static const Category CATEGORIES[] = {
        { "Starlink", "starlink" },
        { "OneWeb",   "oneweb"   },
        { "Iridium",  "iridium"  },
        { "GPS",      "gps"      },
    };
    const Color CHIP_BLUE = {140, 200, 255, 255};   // light blue for the category chips

    // Draws the browse-catalog modal (must be called inside a drawing context). Returns the
    // chosen NORAD id when the user picks a row, otherwise an empty string.
    auto drawCatalogModal = [&]() -> std::string {
        if (!showCatalog) return std::string();
        float sw = (float)GetScreenWidth(), sh = (float)GetScreenHeight();
        DrawRectangleRec((Rectangle){0, 0, sw, sh}, ColorAlpha(BLACK, 0.6f));
        float pw = 420.0f, ph = 460.0f, px = (sw - pw) * 0.5f, py = (sh - ph) * 0.5f;
        DrawRectangleRec((Rectangle){px, py, pw, ph}, (Color){24, 26, 32, 255});
        DrawRectangleLinesEx((Rectangle){px, py, pw, ph}, 2, SKYBLUE);
        DrawTextEx(font_bold, "BROWSE SATELLITES", {px + 16, py + 12}, 18, 1.5f, SKYBLUE);
        bool loadingCat = catFuture.valid();
        DrawTextEx(font, loadingCat ? "Loading catalog..." : catStatus.c_str(),
                   {px + 16, py + 38}, 13, 1, LIGHTGRAY);

        float fx = px + 16, fy = py + 56, fw = pw - 32;
        DrawTextEx(font, "Filter:", {fx, fy + 4}, 14, 1, RAYWHITE);
        if (GuiTextBox((Rectangle){fx + 56, fy, fw - 56, 26}, catFilter, sizeof(catFilter), catFilterEdit))
            catFilterEdit = !catFilterEdit;

        // Quick-access category chips (light blue) — click to set the filter.
        Vector2 mp = GetMousePosition();
        float chipX = fx, chipY = py + 90;
        DrawTextEx(font, "Quick:", {chipX, chipY + 3.0f}, 13, 1, GRAY);
        chipX += 50.0f;
        for (const Category& cat : CATEGORIES) {
            float tw = MeasureTextEx(font, cat.label, 14, 1).x;
            Rectangle chip = {chipX, chipY, tw + 16.0f, 22.0f};
            bool hov = CheckCollisionPointRec(mp, chip);
            if (hov) DrawRectangleRec(chip, ColorAlpha(CHIP_BLUE, 0.22f));
            DrawRectangleLinesEx(chip, 1.0f, CHIP_BLUE);
            DrawTextEx(font, cat.label, {chipX + 8.0f, chipY + 3.0f}, 14, 1, CHIP_BLUE);
            if (hov && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                std::strncpy(catFilter, cat.filter, sizeof(catFilter) - 1);
                catFilter[sizeof(catFilter) - 1] = '\0';
                catFilterEdit = false;
            }
            chipX += chip.width + 6.0f;
        }

        Rectangle listRect = {fx, chipY + 30.0f, fw, ph - 188.0f};
        GuiListViewEx(listRect, filterPtrs.data(), (int)filterPtrs.size(), &catScroll, &catActive, NULL);
        DrawTextEx(font, TextFormat("%d shown", (int)filterPtrs.size()),
                   {fx, py + ph - 64}, 13, 1, GRAY);

        std::string picked;
        if (catActive >= 0 && catActive < (int)filterIds.size()) {
            picked = filterIds[catActive];
            showCatalog = false; catActive = -1;
        }
        if ((GuiButton((Rectangle){px + pw - 96, py + ph - 40, 80, 28}, "Close") ||
             IsKeyPressed(KEY_ESCAPE)) && picked.empty()) {
            showCatalog = false; catFilterEdit = false; catActive = -1;
        }
        return picked;
    };

    bool showEditor = false;                // station editor is a global resource (stays app-level)

    // ── Conjunction screening (closest approach between open tabs; app-level) ───
    bool  showConj   = false;
    float conjTimer  = 0.0f;
    struct ConjPair { int i, j; double rangeKm; double tcaUnix; };
    std::vector<ConjPair> conjPairs;

    // ── Hohmann transfer results (recomputed each frame for the active tab) ─────
    // The toggle + target altitude live per-tab on SatTab; these are just scratch.
    OrbitalMechanics::Hohmann hoh;          // last computed transfer
    bool    hohValid = false;
    Vector3 hohBurn1W{}, hohBurn2W{};       // burn-point world positions (for 2D labels)


    // Station/link editor field buffers + which one is being edited.
    char nameBuf[32] = {0}, latBuf[16] = {0}, lonBuf[16] = {0},
         altBuf[16] = {0}, maskBuf[16] = {0}, freqBuf[16] = {0};
    int  activeEditField = -1;          // -1 none; 0..5 = name/lat/lon/alt/mask/freq
    int  editorSynced    = -2;          // selectedStation last synced into buffers

    auto syncEditorBuffers = [&]() {
        const GroundStation& s = stations[selectedStation];
        std::snprintf(nameBuf, sizeof(nameBuf), "%s", s.name.c_str());
        std::snprintf(latBuf,  sizeof(latBuf),  "%.4f", s.lat);
        std::snprintf(lonBuf,  sizeof(lonBuf),  "%.4f", s.lon);
        std::snprintf(altBuf,  sizeof(altBuf),  "%.3f", s.altKm);
        std::snprintf(maskBuf, sizeof(maskBuf), "%.1f", s.maskDeg);
        std::snprintf(freqBuf, sizeof(freqBuf), "%.4f", downlinkHz / 1e6);
        editorSynced = selectedStation;
    };

    auto fmtDuration = [](double s) -> std::string {
        if (s < 0) s = 0;
        int t = (int)(s + 0.5);
        int h = t / 3600, m = (t % 3600) / 60, sec = t % 60;
        char buf[32];
        if (h > 0) std::snprintf(buf, sizeof(buf), "%dh%02dm%02ds", h, m, sec);
        else       std::snprintf(buf, sizeof(buf), "%02dm%02ds", m, sec);
        return buf;
    };

    auto exportPasses = [&]() {
        if (activeTab < 0) return;
        const auto& passes = tabs[activeTab].passes;
        if (passes.empty()) { exportStatus = "No passes to export"; return; }
        std::error_code ec;
        std::filesystem::create_directories("exports", ec);
        // Sanitize path components — station names like "0N/0E" contain characters
        // (e.g. '/') that would otherwise be read as directory separators.
        auto safe = [](std::string s) {
            for (char& c : s)
                if (!std::isalnum((unsigned char)c) && c != '-' && c != '.') c = '_';
            return s;
        };
        std::string fn = "exports/passes_" + safe(tabs[activeTab].id) + "_" +
                         safe(stations[selectedStation].name) + ".csv";
        std::ofstream o(fn);
        if (!o) { exportStatus = "Export failed: " + fn; return; }
        o << "aos_utc,los_utc,tca_utc,max_elevation_deg,duration_s,aos_az_deg,los_az_deg\n";
        char line[256];
        for (const auto& p : passes) {
            std::snprintf(line, sizeof(line), "%s,%s,%s,%.2f,%.0f,%.1f,%.1f\n",
                p.aos.ToString().substr(0, 19).c_str(),
                p.los.ToString().substr(0, 19).c_str(),
                p.tca.ToString().substr(0, 19).c_str(),
                p.maxElevationDeg, p.durationSec, p.aosAzimuthDeg, p.losAzimuthDeg);
            o << line;
        }
        exportStatus = "Exported " + std::to_string(passes.size()) + " passes -> " + fn;
    };

    // ─────────────────────────────────────────────────────────────────────────
    while (!WindowShouldClose())
    {
        if (IsWindowResized())
            rebuildTargets(GetRenderWidth(), GetRenderHeight());

        // The 2D UI is authored in logical-screen coords and drawn scaled up to the
        // (HiDPI) framebuffer, but GetMousePosition() reports framebuffer-space pixels.
        // Scale the mouse back into logical space so raygui hit-tests line up with the
        // controls the user actually sees (otherwise clicks miss every button).
        SetMouseScale((float)GetScreenWidth() / GetRenderWidth(),
                      (float)GetScreenHeight() / GetRenderHeight());

        // ── Async load: swap each satellite in once its worker thread finishes ──
        for (auto& t : tabs) {
            if (t.loadFuture.valid() &&
                t.loadFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                SatLoadResult res = t.loadFuture.get();
                t.loadStatus = res.status;
                if (res.ok) {
                    t.tle = res.tle; t.sgp4 = res.sgp4; t.tleFromCache = res.fromCache; t.id = res.id;
                    t.futureInterval = (float)(1440.0 / t.tle->MeanMotion()) / FUTURE_POINTS;
                    t.observedPath.clear(); t.lastRecordSim = -1e18;
                    t.futurePath.clear(); t.keplerPath.clear(); t.futureGeo.clear();
                    t.futureTimer = FUTURE_UPDATE_S;
                    t.forcePassRecompute = true;
                }
            }
        }

        // Catalog fetch: swap in entries once the worker finishes, then rebuild filter.
        if (catFuture.valid() &&
            catFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            CatalogResult cr = catFuture.get();
            catStatus = cr.status;
            catalog   = std::move(cr.entries);
            lastFilter = "\x01";                  // force rebuild below
        }
        if (showCatalog && lastFilter != catFilter) rebuildFilter();

        // Deferred tab-bar actions (applied after rendering so refs stay valid).
        int  reqActivate = -1, reqClose = -1;
        bool reqAdd = false;
        std::string reqAddId;            // NORAD id to open as a new tab

        bool textActive   = addEditMode || activeEditField != -1 || catFilterEdit ||
                            (activeTab >= 0 && tabs[activeTab].targetAltEdit);
        bool blockCamera  = textActive || showEditor || showAddOverlay || activeTab < 0;

        // ── Input: camera ──────────────────────────────────────────────────────
        Vector2 mouseDelta = GetMouseDelta();
        float   moveSpeed  = 0.15f;
        Vector3 movement   = {0.0f, 0.0f, 0.0f};
        if (!blockCamera) {
            if (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP))    movement.x =  moveSpeed;
            if (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN))  movement.x = -moveSpeed;
            if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT))  movement.y = -moveSpeed;
            if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) movement.y =  moveSpeed;
        }
        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && !blockCamera) {
            HideCursor();
            UpdateCameraPro(&camera, movement,
                (Vector3){ mouseDelta.x * 0.3f, mouseDelta.y * 0.3f, 0.0f }, 0.0f);
        } else {
            ShowCursor();
            if (!blockCamera)
                UpdateCameraPro(&camera, movement, (Vector3){0,0,0}, GetMouseWheelMove() * 2.0f);
        }

        // ── Input: hotkeys (suppressed while typing in a text field) ────────────
        if (!textActive) {
            if (IsKeyPressed(KEY_F)) ToggleFullscreen();
            if (IsKeyPressed(KEY_M) && activeTab >= 0) tabs[activeTab].showMap     = !tabs[activeTab].showMap;
            if (IsKeyPressed(KEY_H) && activeTab >= 0) tabs[activeTab].showHohmann = !tabs[activeTab].showHohmann;
            if (IsKeyPressed(KEY_X)) { showConj = !showConj; if (showConj) conjDirty = true; }
            if (IsKeyPressed(KEY_E)) { showEditor = !showEditor; if (showEditor) syncEditorBuffers(); }
            if (IsKeyPressed(KEY_C)) exportPasses();
            if (IsKeyPressed(KEY_SPACE)) clock.togglePause();
            if (IsKeyPressed(KEY_RIGHT_BRACKET)) clock.setSpeed(fminf(clock.speed() * 2.0, 100000.0));
            if (IsKeyPressed(KEY_LEFT_BRACKET))  clock.setSpeed(fmaxf(clock.speed() * 0.5, 0.25));
            if (IsKeyPressed(KEY_R)) { clock.resetToNow(); markAllPassRecompute(); markAllRecordBreak(); }
            if (IsKeyPressed(KEY_T)) {
                selectedStation = (selectedStation + 1) % (int)stations.size();
                markAllPassRecompute();
            }
            if (IsKeyPressed(KEY_N) && activeTab >= 0) {
                double simU = clock.unixSeconds();
                for (const auto& p : tabs[activeTab].passes) {
                    double aosU = SimClock::toUnixSeconds(p.aos);
                    if (aosU > simU + 1.0) {
                        clock.setUnixSeconds(aosU - 10.0);
                        clock.setSpeed(1.0);
                        markAllPassRecompute();
                        markAllRecordBreak();   // don't connect logs across the jump
                        break;
                    }
                }
            }
        }

        clock.update(GetFrameTime());
        libsgp4::DateTime simDt = clock.dateTime();
        double            jd    = clock.julianDate();

        // ── Simulation update ───────────────────────────────────────────────────
        SunSim::SunState sun = SunSim::GetSunState(jd, SUN_VISUAL_DISTANCE);
        Vector3 sunPos       = sun.position;

        float  earthRotation = (float)EarthSim::getRotationAngle(jd);
        Matrix spin  = MatrixRotateY((earthRotation + 112.5f) * DEG2RAD);
        Matrix tilt  = MatrixRotateZ(23.44f * DEG2RAD);
        earthModel.transform = MatrixMultiply(tilt, spin);

        const float LON_CALIB = 45.0f;
        auto geoToModel = [&](float lat_deg, float lon_deg, float altitude) -> Vector3 {
            float latR  = lat_deg * DEG2RAD;
            float alpha = (LON_CALIB - lon_deg) * DEG2RAD;
            float r     = EARTH_RADIUS + altitude;
            return { r * cosf(latR) * cosf(alpha),
                     r * sinf(latR),
                     r * cosf(latR) * sinf(alpha) };
        };
        auto geoToWorld = [&](float lat_deg, float lon_deg, float altitude) -> Vector3 {
            return Vector3Transform(geoToModel(lat_deg, lon_deg, altitude), earthModel.transform);
        };
        const float KM_TO_SCENE = EARTH_RADIUS / 6371.0f;

        // Inertial ECI (km) -> scene. Canonical mapping (X=-ECI_Y, Y=ECI_Z, Z=-ECI_X)
        // with Earth's axial tilt applied, but NOT the daily spin — orbits are inertial,
        // so they stay fixed while the textured Earth rotates beneath them.
        auto sceneFromEci = [&](const std::array<double, 3>& p) -> Vector3 {
            Vector3 v = { -(float)p[1] * KM_TO_SCENE,
                           (float)p[2] * KM_TO_SCENE,
                          -(float)p[0] * KM_TO_SCENE };
            return Vector3Transform(v, tilt);
        };

        std::vector<Vector3> stationPositions(stations.size());
        for (size_t i = 0; i < stations.size(); i++)
            stationPositions[i] = geoToWorld(stations[i].lat, stations[i].lon, 0.0f);

        // ── Simulate every tab (background sim keeps each history continuous) ──
        const GroundStation&   obs = stations[selectedStation];
        libsgp4::CoordGeodetic obsGeo(obs.lat, obs.lon, obs.altKm);
        const double RECORD_INTERVAL = 3.0;   // sim seconds between observed-track samples
        double simU = clock.unixSeconds();
        for (auto& t : tabs) {
            if (!t.sgp4) continue;
            try {
                auto sub = FetchTLE::getSubPoint(*t.sgp4, simDt);
                t.subLat = sub[0]; t.subLon = sub[1]; t.subAlt = sub[2] * KM_TO_SCENE;
            } catch (const std::exception& e) {
                TraceLog(LOG_WARNING, "SGP4 propagation error: %s", e.what());
            }

            // Record where the satellite has ACTUALLY been this session (a real trace
            // at whatever time speed, not a backward extrapolation). A jump (R/N) breaks
            // the segment so the line isn't connected across the gap.
            if (t.lastRecordSim <= -1e17 || std::fabs(simU - t.lastRecordSim) >= RECORD_INTERVAL) {
                if (t.recordBreak && !t.observedPath.empty())
                    t.observedPath.push_back({NAN, NAN, NAN});
                t.recordBreak = false;
                t.observedPath.push_back(geoToModel(t.subLat, t.subLon, t.subAlt));
                t.lastRecordSim = simU;
                if ((int)t.observedPath.size() > 10000) t.observedPath.erase(t.observedPath.begin());
            }

            t.futureTimer += GetFrameTime();
            if (t.futureTimer >= FUTURE_UPDATE_S) {
                t.futureTimer = 0.0f;
                t.futurePath.clear(); t.futureGeo.clear();
                auto futureSub = FetchTLE::getFutureSubPoints(*t.sgp4, FUTURE_POINTS, t.futureInterval, simDt);
                for (auto& s : futureSub) {
                    t.futurePath.push_back(geoToModel(s[0], s[1], s[2] * KM_TO_SCENE));
                    t.futureGeo.push_back({s[0], s[1]});
                }
                t.keplerPath.clear();
                t.kepEl = OrbitalMechanics::fromStateVector(FetchTLE::getStateVector(*t.sgp4, simDt));
                for (int i = 0; i < FUTURE_POINTS; i++) {
                    double dtSec = (double)i * t.futureInterval * 60.0;
                    auto p = OrbitalMechanics::propagate(t.kepEl, dtSec);
                    libsgp4::CoordGeodetic g =
                        libsgp4::Eci(simDt.AddSeconds(dtSec),
                                     libsgp4::Vector(p[0], p[1], p[2])).ToGeodetic();
                    t.keplerPath.push_back(geoToModel((float)(g.latitude  * RAD2DEG),
                                                      (float)(g.longitude * RAD2DEG),
                                                      (float)(g.altitude  * KM_TO_SCENE)));
                }
            }

            t.passRecomputeTimer += GetFrameTime();
            if (t.forcePassRecompute || t.passRecomputeTimer >= 2.0f) {
                t.forcePassRecompute = false;
                t.passRecomputeTimer = 0.0f;
                try {
                    t.passes = PassPredict::predictPasses(*t.sgp4, obsGeo, simDt, 24.0, obs.maskDeg);
                } catch (const std::exception& e) {
                    TraceLog(LOG_WARNING, "Pass prediction error: %s", e.what());
                }
            }
        }

        // ── Active-tab geometry (observer-relative; drives the globe + sidebar) ──
        Vector3 satPos = Vector3Zero();
        PassPredict::LookAngle look;
        double dopplerHz = 0.0;
        if (activeTab >= 0 && tabs[activeTab].sgp4) {
            SatTab& A = tabs[activeTab];
            satPos    = geoToWorld(A.subLat, A.subLon, A.subAlt);
            look      = PassPredict::lookAngle(*A.sgp4, obsGeo, simDt, obs.maskDeg);
            dopplerHz = PassPredict::dopplerShiftedHz(downlinkHz, look.rangeRateKmS) - downlinkHz;
        }

        // ── Conjunction screening: closest approach between every pair of loaded tabs ──
        conjTimer += GetFrameTime();
        if (showConj && (conjDirty || conjTimer >= 3.0f)) {
            conjDirty = false; conjTimer = 0.0f;
            conjPairs.clear();
            for (int i = 0; i < (int)tabs.size(); i++) {
                if (!tabs[i].sgp4) continue;
                for (int j = i + 1; j < (int)tabs.size(); j++) {
                    if (!tabs[j].sgp4) continue;
                    auto cj = PassPredict::closestApproach(*tabs[i].sgp4, *tabs[j].sgp4, simDt, 24.0);
                    if (cj.valid)
                        conjPairs.push_back({ i, j, cj.minRangeKm, SimClock::toUnixSeconds(cj.tca) });
                }
            }
            std::sort(conjPairs.begin(), conjPairs.end(),
                      [](const ConjPair& a, const ConjPair& b) { return a.rangeKm < b.rangeKm; });
        }

        // ── Empty state: no tabs ⇒ gray screen with icon + NORAD search ─────────
        if (activeTab < 0) {
            BeginDrawing();
            ClearBackground((Color){26, 28, 34, 255});
            // Author the UI in logical coords and scale up to the (HiDPI) framebuffer,
            // matching the main path so raygui hit-tests line up with the scaled mouse.
            rlPushMatrix();
            rlScalef((float)GetRenderWidth() / GetScreenWidth(),
                     (float)GetRenderHeight() / GetScreenHeight(), 1.0f);
            float cx = GetScreenWidth() * 0.5f, cy = GetScreenHeight() * 0.5f;
            drawSatQuestionIcon(cx, cy - 70.0f);
            const char* msg = "No satellites tracked";
            Vector2 mw = MeasureTextEx(font_bold, msg, 24, 1);
            DrawTextEx(font_bold, msg, {cx - mw.x * 0.5f, cy - 6.0f}, 24, 1, RAYWHITE);
            float bw = 200.0f, bh = 30.0f, bx = cx - (bw + 80.0f) * 0.5f, by = cy + 44.0f;
            DrawTextEx(font, "Add satellite by NORAD ID:", {bx, by - 22.0f}, 15, 1, LIGHTGRAY);
            if (GuiTextBox((Rectangle){bx, by, bw, bh}, addInput, sizeof(addInput), addEditMode))
                addEditMode = !addEditMode;
            if ((GuiButton((Rectangle){bx + bw + 8.0f, by, 72.0f, bh}, "Add") ||
                 (addEditMode && IsKeyPressed(KEY_ENTER))) && addInput[0]) {
                addEditMode = false; reqAddId = addInput; reqAdd = true;
            }
            if (GuiButton((Rectangle){bx, by + 40.0f, bw + 80.0f, bh}, "Browse catalog..."))
                openCatalog();
            std::string pick = drawCatalogModal();
            if (!pick.empty()) { reqAddId = pick; reqAdd = true; }
            rlPopMatrix();
            EndDrawing();
            if (reqAdd) { addTab(reqAddId); addInput[0] = '\0'; showCatalog = false; }
            continue;
        }

        // Bind the active tab's state so the existing render code reads unchanged.
        SatTab& A = tabs[activeTab];
        auto&  tle = A.tle;  auto& sgp4 = A.sgp4;
        auto&  observedPath = A.observedPath; auto& futurePath = A.futurePath;
        auto&  keplerPath   = A.keplerPath;   auto& futureGeo  = A.futureGeo;
        auto&  kepEl = A.kepEl; auto& passes = A.passes;
        float  subLat = A.subLat, subLon = A.subLon;
        bool   tleFromCache = A.tleFromCache;
        bool   loading = A.loadFuture.valid();
        // Per-tab view/planning state (so "what's open" is remembered per tab).
        auto&  showMap = A.showMap; auto& showHohmann = A.showHohmann;
        auto&  targetAltKm = A.targetAltKm; auto& targetAltBuf = A.targetAltBuf;
        auto&  targetAltEdit = A.targetAltEdit;

        SetShaderValue(shader,    lightPosLoc,    &sunPos,           SHADER_UNIFORM_VEC3);
        SetShaderValue(shader,    viewPosLoc,     &camera.position,  SHADER_UNIFORM_VEC3);
        SetShaderValue(satShader, satLightPosLoc, &sunPos,           SHADER_UNIFORM_VEC3);
        SetShaderValue(satShader, satViewPosLoc,  &camera.position,  SHADER_UNIFORM_VEC3);

        // ── Phase 1: Render 3D scene to texture ───────────────────────────────
        BeginTextureMode(sceneTarget);
        ClearBackground((Color){2, 2, 15, 255});
        BeginMode3D(camera);
        rlSetClipPlanes(0.1f, 20000.0f);

        rlDisableDepthMask();
        DrawModel(skybox, camera.position, -100.0f, WHITE);
        rlEnableDepthMask();

        DrawModel(earthModel, Vector3Zero(), EARTH_RADIUS, WHITE);

        // Observed path — the actual session record of where the satellite has been,
        // billboard quads, brightest near the satellite and fading into the past.
        // Co-rotates with the globe (model space); NaN entries break the line at jumps.
        int pastCount = showHohmann ? 0 : (int)observedPath.size();
        rlDisableBackfaceCulling();
        BeginBlendMode(BLEND_ADDITIVE);
        rlBegin(RL_QUADS);
        for (int i = 1; i < pastCount; i++) {
            if (std::isnan(observedPath[i].x) || std::isnan(observedPath[i - 1].x)) continue;
            float t     = (float)i / (float)pastCount;   // 0 = oldest, 1 = most recent
            float alpha = t * 200.0f;
            float width = 0.018f + 0.020f * t;
            Vector3 p0 = Vector3Transform(observedPath[i - 1], earthModel.transform);
            Vector3 p1 = Vector3Transform(observedPath[i],     earthModel.transform);
            Vector3 seg = Vector3Normalize(Vector3Subtract(p1, p0));
            Vector3 mid = Vector3Scale(Vector3Add(p0, p1), 0.5f);
            Vector3 cam = Vector3Normalize(Vector3Subtract(camera.position, mid));
            Vector3 perp = Vector3Normalize(Vector3CrossProduct(seg, cam));
            Vector3 off  = Vector3Scale(perp, width * 0.5f);
            rlColor4ub(120, 210, 255, (unsigned char)alpha);
            rlVertex3f(p0.x - off.x, p0.y - off.y, p0.z - off.z);
            rlVertex3f(p0.x + off.x, p0.y + off.y, p0.z + off.z);
            rlVertex3f(p1.x + off.x, p1.y + off.y, p1.z + off.z);
            rlVertex3f(p1.x - off.x, p1.y - off.y, p1.z - off.z);
        }
        rlEnd();
        EndBlendMode();
        rlEnableBackfaceCulling();

        // Future ground track — billboard quads, orange, fading
        int futureCount = showHohmann ? 0 : (int)futurePath.size();
        if (futureCount > 0) {
            Vector3 futureOrigin = Vector3Transform(futurePath[0], earthModel.transform);
            BeginBlendMode(BLEND_ALPHA);
            DrawLine3D(satPos, futureOrigin, {255, 160, 40, 200});
            EndBlendMode();
        }
        rlDisableBackfaceCulling();
        BeginBlendMode(BLEND_ADDITIVE);
        rlBegin(RL_QUADS);
        for (int i = 1; i < futureCount; i++) {
            float t     = (float)i / (float)FUTURE_POINTS;
            float alpha = (1.0f - t) * 180.0f;
            float width = 0.022f * (1.0f - t * 0.5f);
            Vector3 p0 = Vector3Transform(futurePath[i - 1], earthModel.transform);
            Vector3 p1 = Vector3Transform(futurePath[i],     earthModel.transform);
            Vector3 seg = Vector3Normalize(Vector3Subtract(p1, p0));
            Vector3 mid = Vector3Scale(Vector3Add(p0, p1), 0.5f);
            Vector3 cam = Vector3Normalize(Vector3Subtract(camera.position, mid));
            Vector3 perp = Vector3Normalize(Vector3CrossProduct(seg, cam));
            Vector3 off  = Vector3Scale(perp, width * 0.5f);
            rlColor4ub(255, 155, 35, (unsigned char)alpha);
            rlVertex3f(p0.x - off.x, p0.y - off.y, p0.z - off.z);
            rlVertex3f(p0.x + off.x, p0.y + off.y, p0.z + off.z);
            rlVertex3f(p1.x + off.x, p1.y + off.y, p1.z + off.z);
            rlVertex3f(p1.x - off.x, p1.y - off.y, p1.z - off.z);
        }
        rlEnd();
        EndBlendMode();
        rlEnableBackfaceCulling();

        // Two-body Kepler prediction — thin cyan line
        int keplerCount = showHohmann ? 0 : (int)keplerPath.size();
        BeginBlendMode(BLEND_ALPHA);
        for (int i = 1; i < keplerCount; i++) {
            Vector3 k0 = Vector3Transform(keplerPath[i - 1], earthModel.transform);
            Vector3 k1 = Vector3Transform(keplerPath[i],     earthModel.transform);
            unsigned char a = (unsigned char)((1.0f - (float)i / FUTURE_POINTS) * 220.0f);
            DrawLine3D(k0, k1, (Color){60, 230, 220, a});
        }
        EndBlendMode();

        // Satellite model + bright beacon (hidden in Hohmann mode, which uses its own
        // inertial orbit diagram + transfer-spacecraft marker instead).
        if (sgp4 && !showHohmann) {
            float satScale = 0.01f;
            DrawModelEx(satModel, satPos, (Vector3){0, 1, 0}, 0.0f,
                (Vector3){satScale, satScale, satScale}, WHITE);
            DrawSphere(satPos, 0.07f, {255, 235, 160, 255});
        }

        // ── Hohmann transfer visualization (inertial orbits) ──────────────────
        hohValid = false;
        if (showHohmann && kepEl.valid) {
            const double PI_D = 3.14159265358979323846, TWO_PI_D = 2.0 * PI_D;
            const double EARTH_R_KM = 6371.0;        // scene reference radius (matches KM_TO_SCENE)
            double r1 = kepEl.a;                      // initial circular radius (km)
            double r2 = EARTH_R_KM + targetAltKm;     // target circular radius (km)
            hoh = OrbitalMechanics::computeHohmann(r1, r2);
            hohValid = true;
            double inc = kepEl.i, raan = kepEl.raan;
            bool   raising = hoh.raising;

            // Burn 1 is at the current orbit radius r1 — the transfer's PERIapsis when
            // raising, but its APOapsis when lowering. Orient the ellipse so that apsis
            // lands at the satellite's current in-plane angle (argp + true anomaly).
            double nuBurn1 = raising ? 0.0 : PI_D;
            double nuBurn2 = raising ? PI_D : 0.0;
            double argpT   = kepEl.argp + kepEl.nu0 - nuBurn1;
            double pathN0  = nuBurn1;                  // coasting arc from burn 1 ...
            double pathN1  = raising ? PI_D : TWO_PI_D; // ... to burn 2

            auto orbitPt = [&](double a, double e, double ap, double nu) {
                return sceneFromEci(OrbitalMechanics::orbitPositionAt(a, e, inc, raan, ap, nu));
            };
            auto drawArc = [&](double a, double e, double ap, double n0, double n1, int seg, Color c) {
                Vector3 prev{};
                for (int k = 0; k <= seg; k++) {
                    double nu = n0 + (n1 - n0) * (double)k / seg;
                    Vector3 s = orbitPt(a, e, ap, nu);
                    if (k > 0) DrawLine3D(prev, s, c);
                    prev = s;
                }
            };
            // Initial orbit (blue), target orbit (green), transfer arc burn1->burn2 (orange).
            drawArc(r1, 0.0, argpT, 0.0, TWO_PI_D, 160, (Color){ 90, 180, 255, 170});
            drawArc(r2, 0.0, argpT, 0.0, TWO_PI_D, 160, (Color){ 80, 235, 140, 170});
            drawArc(hoh.aTransfer, hoh.eTransfer, argpT, pathN0, pathN1, 128, (Color){255, 150, 40, 255});

            // Burn points (also the transfer's apsides): burn 1 at r1, burn 2 at r2.
            hohBurn1W = orbitPt(hoh.aTransfer, hoh.eTransfer, argpT, nuBurn1);
            hohBurn2W = orbitPt(hoh.aTransfer, hoh.eTransfer, argpT, nuBurn2);
            DrawSphere(hohBurn1W, 0.13f, (Color){255, 180, 60, 255});
            DrawSphere(hohBurn2W, 0.13f, (Color){120, 255, 160, 255});

            // Thruster icons: an exhaust cone at each burn, pointing opposite the thrust
            // (thrust is prograde when raising, retrograde when lowering — same at both burns).
            auto thruster = [&](Vector3 at, double nu) {
                Vector3 ahead  = orbitPt(hoh.aTransfer, hoh.eTransfer, argpT, nu + 0.04);
                Vector3 behind = orbitPt(hoh.aTransfer, hoh.eTransfer, argpT, nu - 0.04);
                Vector3 prog   = Vector3Normalize(Vector3Subtract(ahead, behind));
                Vector3 thrust = raising ? prog : Vector3Negate(prog);
                Vector3 exhaust = Vector3Negate(thrust);     // plume opposite thrust
                rlDisableBackfaceCulling();
                DrawCylinderEx(at, Vector3Add(at, Vector3Scale(exhaust, 0.55f)), 0.10f, 0.0f, 10,
                               (Color){255, 150, 40, 255});
                BeginBlendMode(BLEND_ADDITIVE);
                DrawCylinderEx(at, Vector3Add(at, Vector3Scale(exhaust, 0.85f)), 0.05f, 0.0f, 10,
                               (Color){255, 230, 120, 150});
                EndBlendMode();
                rlEnableBackfaceCulling();
            };
            thruster(hohBurn1W, nuBurn1);
            thruster(hohBurn2W, nuBurn2);

            // Spacecraft coasting along the transfer (visual animation, loops burn1 -> burn2).
            double ph = fmod(GetTime() * 0.12, 1.0);
            DrawSphere(orbitPt(hoh.aTransfer, hoh.eTransfer, argpT, pathN0 + (pathN1 - pathN0) * ph),
                       0.10f, WHITE);
        }

        // Ground stations
        for (size_t i = 0; i < stations.size(); i++) {
            Vector3 tip = geoToWorld(stations[i].lat, stations[i].lon, 0.5f);
            DrawLine3D(stationPositions[i], tip, stations[i].color);
            DrawSphere(tip, 0.12f, stations[i].color);
            if ((int)i == selectedStation && look.visible)
                DrawLine3D(stationPositions[i], satPos, (Color){120, 255, 140, 200});
        }

        // Sun
        DrawSphere(sunPos, SUN_RADIUS, WHITE);
        rlDisableDepthMask();
        for (int i = 1; i <= 3; i++)
            DrawSphere(sunPos, SUN_RADIUS + (i * 28.0f), ColorAlpha(YELLOW, 0.18f / (float)i));
        rlEnableDepthMask();

        EndMode3D();
        EndTextureMode();

        // ── Phase 2: bright pass ──────────────────────────────────────────────
        BeginTextureMode(brightTarget);
        ClearBackground(BLACK);
        BeginShaderMode(brightPassShader);
        DrawTexturePro(sceneTarget.texture, fullSrc, halfDst, {0, 0}, 0.0f, WHITE);
        EndShaderMode();
        EndTextureMode();

        // ── Phase 3: Gaussian blur ping-pong ──────────────────────────────────
        int one = 1, zero = 0;
        for (int pass = 0; pass < 3; pass++) {
            Texture2D blurInput = (pass == 0) ? brightTarget.texture : blurTargetB.texture;
            BeginTextureMode(blurTargetA);
            ClearBackground(BLACK);
            SetShaderValue(blurShader, blurHorizontalLoc, &one, SHADER_UNIFORM_INT);
            BeginShaderMode(blurShader);
            DrawTexturePro(blurInput, halfSrc, halfDst, {0, 0}, 0.0f, WHITE);
            EndShaderMode();
            EndTextureMode();

            BeginTextureMode(blurTargetB);
            ClearBackground(BLACK);
            SetShaderValue(blurShader, blurHorizontalLoc, &zero, SHADER_UNIFORM_INT);
            BeginShaderMode(blurShader);
            DrawTexturePro(blurTargetA.texture, halfSrc, halfDst, {0, 0}, 0.0f, WHITE);
            EndShaderMode();
            EndTextureMode();
        }

        // ── Phase 4: composite + 2D UI ────────────────────────────────────────
        BeginDrawing();
        Rectangle fullDst = {0, 0, (float)GetRenderWidth(), (float)GetRenderHeight()};
        DrawTexturePro(sceneTarget.texture, fullSrc, fullDst, {0, 0}, 0.0f, WHITE);
        BeginBlendMode(BLEND_ADDITIVE);
        DrawTexturePro(blurTargetB.texture, halfSrc, fullDst, {0, 0}, 0.0f, ColorAlpha(WHITE, 0.85f));
        EndBlendMode();

        float screenW      = (float)GetScreenWidth();
        float screenH      = (float)GetScreenHeight();
        float sidebarWidth = 300.0f;
        rlPushMatrix();
        rlScalef((float)GetRenderWidth() / screenW, (float)GetRenderHeight() / screenH, 1.0f);

        // Floating station labels
        for (size_t i = 0; i < stations.size(); i++) {
            if (Vector3DotProduct(Vector3Normalize(stationPositions[i]),
                                  Vector3Normalize(camera.position)) > 0.0f) {
                Vector2 sp = GetWorldToScreen(stationPositions[i], camera);
                DrawTextEx(font, stations[i].name.c_str(),
                           {sp.x + 8.0f, sp.y - 8.0f}, 18, 1, stations[i].color);
            }
        }

        // Ground-station legend
        {
            const float panelX = 10.0f, rowH = 26.0f;
            int n = (int)stations.size();
            float panelY  = screenH - (n * rowH) - 34.0f;
            DrawRectangleRec((Rectangle){panelX - 4.0f, panelY - 22.0f, 220.0f, n * rowH + 28.0f},
                              ColorAlpha(BLACK, 0.5f));
            DrawTextEx(font, "GND STATIONS  [T] sel  [E] edit", (Vector2){panelX, panelY - 20.0f}, 14, 1, LIGHTGRAY);
            for (int i = 0; i < n; i++) {
                float y = panelY + i * rowH;
                DrawRectangleV((Vector2){panelX, y + 4.0f}, (Vector2){12.0f, 12.0f}, stations[i].color);
                DrawTextEx(font, TextFormat("%s%s", stations[i].name.c_str(),
                           i == selectedStation ? " *" : ""),
                           (Vector2){panelX + 18.0f, y}, 16, 1, stations[i].color);
            }
        }

        // Next-passes panel
        {
            const float px = 10.0f, py = 80.0f + TAB_BAR_H, pw = 330.0f;
            int rows = (int)passes.size(); if (rows > 6) rows = 6;
            DrawRectangleRec((Rectangle){px, py, pw, 64.0f + rows * 38.0f}, ColorAlpha(BLACK, 0.55f));
            DrawTextEx(font_bold, TextFormat("PASSES @ %s", stations[selectedStation].name.c_str()),
                       {px + 10, py + 8}, 18, 1.5f, (Color){120,255,140,255});
            DrawTextEx(font, TextFormat("min elev %.0f deg   [N] next  [C] export CSV",
                       stations[selectedStation].maskDeg), {px + 10, py + 30}, 13, 1, LIGHTGRAY);
            double simU = clock.unixSeconds();
            if (passes.empty())
                DrawTextEx(font, loading ? "Loading TLE..." : "No passes in next 24 h",
                           {px + 10, py + 52}, 15, 1, GRAY);
            for (int i = 0; i < rows; i++) {
                const auto& p = passes[i];
                double aosU = SimClock::toUnixSeconds(p.aos);
                double losU = SimClock::toUnixSeconds(p.los);
                float  ry   = py + 52.0f + i * 38.0f;
                bool   nowUp = simU >= aosU && simU <= losU;
                Color  c     = nowUp ? (Color){120,255,140,255} : RAYWHITE;
                std::string when = nowUp ? "NOW  -" + fmtDuration(losU - simU) + " to LOS"
                                         : "in " + fmtDuration(aosU - simU);
                DrawTextEx(font, p.aos.ToString().substr(11, 8).c_str(), {px + 10, ry}, 16, 1, c);
                DrawTextEx(font, TextFormat("max %.0f deg  %s", p.maxElevationDeg,
                           fmtDuration(p.durationSec).c_str()), {px + 92, ry}, 14, 1, c);
                DrawTextEx(font, when.c_str(), {px + 10, ry + 17}, 13, 1, LIGHTGRAY);
            }
        }

        // ── Conjunction panel (toggle X) — closest approach between open tabs ────
        if (showConj) {
            const float pw = 330.0f, px = screenW - sidebarWidth - pw - 10.0f, py = 80.0f + TAB_BAR_H;
            int rows = (int)conjPairs.size(); if (rows > 8) rows = 8;
            DrawRectangleRec((Rectangle){px, py, pw, 56.0f + rows * 34.0f}, ColorAlpha(BLACK, 0.6f));
            DrawTextEx(font_bold, "CONJUNCTIONS (24h)", {px + 10, py + 8}, 18, 1.5f, (Color){255,170,60,255});
            int loaded = 0; for (auto& t : tabs) if (t.sgp4) loaded++;
            double simU = clock.unixSeconds();
            if (loaded < 2) {
                DrawTextEx(font, "Need >= 2 loaded satellites", {px + 10, py + 34}, 14, 1, GRAY);
            } else {
                for (int r = 0; r < rows; r++) {
                    const ConjPair& cp = conjPairs[r];
                    float ry = py + 34.0f + r * 34.0f;
                    Color c = cp.rangeKm < 5.0   ? (Color){255,90,90,255}
                            : cp.rangeKm < 25.0  ? (Color){255,170,60,255}
                            :                      RAYWHITE;
                    const char* warn = cp.rangeKm < 5.0 ? "! " : "";
                    std::string names = TextFormat("%s%s  /  %s", warn,
                        (tabs[cp.i].tle ? tabs[cp.i].tle->Name() : tabs[cp.i].id).c_str(),
                        (tabs[cp.j].tle ? tabs[cp.j].tle->Name() : tabs[cp.j].id).c_str());
                    if (names.size() > 38) names = names.substr(0, 37) + "…";
                    DrawTextEx(font, names.c_str(), {px + 10, ry}, 14, 1, c);
                    DrawTextEx(font, TextFormat("min %.1f km   in %s", cp.rangeKm,
                               fmtDuration(cp.tcaUnix - simU).c_str()), {px + 10, ry + 16}, 13, 1, c);
                }
            }
        }

        // Clock readout (shifted below the tab bar)
        float clkY = TAB_BAR_H + 10.0f;
        DrawTextEx(font, simDt.ToString().substr(0, 19).c_str(), {10, clkY}, 20, 2, YELLOW);
        DrawTextEx(font, TextFormat("%s   x%g   Day %.2f",
                   clock.isPaused() ? "PAUSED" : "RUN", clock.speed(), sun.dayOfYear),
                   {10, clkY + 24.0f}, 16, 1, clock.isPaused() ? ORANGE : LIME);
        DrawTextEx(font, "SPACE pause  [ ] speed  R now  N next  T station  M map  E edit  C export  H hohmann  X conj",
                   {10, clkY + 44.0f}, 13, 1, ColorAlpha(RAYWHITE, 0.7f));
        if (!exportStatus.empty())
            DrawTextEx(font, exportStatus.c_str(), {10, screenH - 18.0f}, 13, 1, (Color){120,255,140,255});

        // ── Hohmann transfer panel + burn labels (toggle H) ─────────────────────
        if (showHohmann) {
            const float px = 360.0f, py = 80.0f + TAB_BAR_H, pw = 300.0f, ph = 212.0f;
            DrawRectangleRec((Rectangle){px, py, pw, ph}, ColorAlpha(BLACK, 0.6f));
            DrawTextEx(font_bold, "HOHMANN TRANSFER", {px + 12, py + 10}, 18, 1.5f, (Color){255, 170, 60, 255});
            DrawLine((int)(px + 12), (int)(py + 34), (int)(px + pw - 12), (int)(py + 34), GRAY);
            const char* a1 = hoh.raising ? "periapsis" : "apoapsis";
            const char* a2 = hoh.raising ? "apoapsis"  : "periapsis";
            if (hohValid) {
                auto row = [&](const char* s, float yy, Color c) { DrawTextEx(font, s, {px + 12, yy}, 15, 1, c); };
                row(TextFormat("Initial orbit:  %.0f km",   hoh.r1 - 6371.0),   py + 42,  (Color){ 90,180,255,255});
                row(TextFormat("Target orbit:   %.0f km",   targetAltKm),       py + 62,  (Color){ 80,235,140,255});
                row(TextFormat("dv1 (%s): %.0f m/s",  a1, hoh.dv1 * 1000.0),    py + 88,  RAYWHITE);
                row(TextFormat("dv2 (%s): %.0f m/s",  a2, hoh.dv2 * 1000.0),    py + 108, RAYWHITE);
                row(TextFormat("Total dv:        %.0f m/s", hoh.dvTotal*1000.0),py + 128, (Color){255,170,60,255});
                int tt = (int)hoh.transferTime;
                row(TextFormat("Transfer time:  %dh %02dm", tt / 3600, (tt % 3600) / 60), py + 148, RAYWHITE);
            } else {
                DrawTextEx(font, "Waiting for orbit data...", {px + 12, py + 44}, 15, 1, GRAY);
            }
            DrawTextEx(font, "Target alt (km):", {px + 12, py + 176}, 14, 1, LIGHTGRAY);
            if (GuiTextBox((Rectangle){px + 128, py + 174, 90, 26}, targetAltBuf, sizeof(targetAltBuf), targetAltEdit))
                targetAltEdit = !targetAltEdit;
            if ((GuiButton((Rectangle){px + 226, py + 174, 62, 26}, "Set") ||
                 (targetAltEdit && IsKeyPressed(KEY_ENTER)))) {
                targetAltEdit = false;
                double v = atof(targetAltBuf);
                if (v > 50.0) targetAltKm = v;
            }
            // 3D burn-point labels
            if (hohValid) {
                Vector2 sp1 = GetWorldToScreen(hohBurn1W, camera);
                Vector2 sp2 = GetWorldToScreen(hohBurn2W, camera);
                DrawTextEx(font, TextFormat("BURN 1  %.0f m/s", hoh.dv1 * 1000.0),
                           {sp1.x + 8, sp1.y - 8}, 14, 1, (Color){255, 180, 60, 255});
                DrawTextEx(font, a1, {sp1.x + 8, sp1.y + 8}, 12, 1, ColorAlpha(RAYWHITE, 0.8f));
                DrawTextEx(font, TextFormat("BURN 2  %.0f m/s", hoh.dv2 * 1000.0),
                           {sp2.x + 8, sp2.y - 8}, 14, 1, (Color){120, 255, 160, 255});
                DrawTextEx(font, a2, {sp2.x + 8, sp2.y + 8}, 12, 1, ColorAlpha(RAYWHITE, 0.8f));
            }
        }

        // ── 2D map (toggle M) ──────────────────────────────────────────────────
        if (showMap) {
            float mw = screenW - sidebarWidth - 40.0f;
            float mh = mw * 0.5f;
            if (mh > screenH * 0.45f) { mh = screenH * 0.45f; mw = mh * 2.0f; }
            float mx = 20.0f, my = screenH - mh - 50.0f;
            DrawRectangleRec((Rectangle){mx - 2, my - 2, mw + 4, mh + 4}, ColorAlpha(BLACK, 0.85f));
            DrawTexturePro(earthTexture,
                {0, 0, (float)earthTexture.width, (float)earthTexture.height},
                {mx, my, mw, mh}, {0, 0}, 0.0f, ColorAlpha(WHITE, 0.9f));
            DrawRectangleLinesEx((Rectangle){mx, my, mw, mh}, 1, GRAY);
            auto toMap = [&](float lat, float lon) -> Vector2 {
                return { mx + (lon + 180.0f) / 360.0f * mw, my + (90.0f - lat) / 180.0f * mh };
            };
            // Ground track (skip the seam where longitude wraps)
            for (size_t i = 1; i < futureGeo.size(); i++) {
                if (fabsf(futureGeo[i][1] - futureGeo[i - 1][1]) > 180.0f) continue;
                DrawLineEx(toMap(futureGeo[i - 1][0], futureGeo[i - 1][1]),
                           toMap(futureGeo[i][0], futureGeo[i][1]), 1.5f, (Color){255, 155, 35, 220});
            }
            for (size_t i = 0; i < stations.size(); i++) {
                Vector2 sp = toMap(stations[i].lat, stations[i].lon);
                DrawCircleV(sp, 4.0f, stations[i].color);
            }
            if (sgp4) {
                Vector2 ssp = toMap(subLat, subLon);
                DrawCircleV(ssp, 5.0f, (Color){255, 235, 160, 255});
                DrawCircleLinesV(ssp, 8.0f, WHITE);
            }
            DrawTextEx(font_bold, "GROUND TRACK (2D)  [M] close", {mx + 6, my + 4}, 14, 1, RAYWHITE);
        }

        // ── Right sidebar ───────────────────────────────────────────────────────
        Rectangle sidebarRect = {screenW - sidebarWidth, 0, sidebarWidth, screenH};
        DrawRectangleRec(sidebarRect, ColorAlpha(DARKGRAY, 0.7f));
        DrawLineEx({screenW - sidebarWidth, 0}, {screenW - sidebarWidth, screenH}, 2, GRAY);

        float sx = screenW - sidebarWidth + 20;
        DrawTextEx(font_bold, "ORBIT / TRACKING", {sx, 20}, 20, 2, SKYBLUE);
        DrawLine((int)sx, 45, (int)(screenW - 20), 45, RAYWHITE);

        int   startY  = 70, spacing = 26;
        float altitudeKm = (Vector3Length(satPos) - EARTH_RADIUS) * (6371.0f / 5.0f);

        if (tle) {
            DrawTextEx(font, TextFormat("ID: %s", tle->Name().c_str()),
                       {sx, (float)startY}, 18, 2, RAYWHITE);
            double ageDays = (libsgp4::DateTime::Now() - tle->Epoch()).TotalDays();
            Color ageColor = ageDays < 3.0 ? LIME : (ageDays < 14.0 ? ORANGE : (Color){255,90,90,255});
            DrawTextEx(font, TextFormat("TLE: %s  (%.1fd old%s)", tle->Epoch().ToString().substr(0,10).c_str(),
                       ageDays, tleFromCache ? ", cached" : ""),
                       {sx, (float)(startY + spacing)}, 15, 1, ageColor);
            DrawTextEx(font, TextFormat("Altitude: %.2f km", altitudeKm),
                       {sx, (float)(startY + spacing * 2)}, 18, 2, RAYWHITE);
            DrawTextEx(font, TextFormat("Inclination: %.4f deg", tle->Inclination(true)),
                       {sx, (float)(startY + spacing * 3)}, 18, 2, RAYWHITE);
            DrawTextEx(font, TextFormat("Orbital Vel: %.2f km/s",
                       (tle->MeanMotion() * 2 * PI * (6371.0f + altitudeKm)) / 86400.0),
                       {sx, (float)(startY + spacing * 4)}, 18, 2, RAYWHITE);
        } else {
            DrawTextEx(font, "Loading satellite...", {sx, (float)startY}, 18, 1, LIGHTGRAY);
        }

        // Observer-relative live data
        float oy = startY + spacing * 6.0f;
        DrawTextEx(font_bold, TextFormat("OBSERVER: %s", stations[selectedStation].name.c_str()),
                   {sx, oy}, 16, 1.5f, (Color){120,255,140,255});
        DrawLine((int)sx, (int)(oy + 20), (int)(screenW - 20), (int)(oy + 20), GRAY);
        Color liveC = look.visible ? (Color){120,255,140,255} : (Color){200,120,120,255};
        DrawTextEx(font, look.visible ? "ABOVE HORIZON" : "below horizon", {sx, oy + 26}, 16, 1, liveC);
        DrawTextEx(font, TextFormat("Azimuth:   %.1f deg", look.azimuthDeg),   {sx, oy + 48},  16, 1, RAYWHITE);
        DrawTextEx(font, TextFormat("Elevation: %.1f deg", look.elevationDeg), {sx, oy + 70},  16, 1, RAYWHITE);
        DrawTextEx(font, TextFormat("Range:     %.0f km",  look.rangeKm),      {sx, oy + 92},  16, 1, RAYWHITE);
        DrawTextEx(font, TextFormat("Doppler:   %+.0f Hz", dopplerHz),         {sx, oy + 114}, 16, 1, RAYWHITE);
        DrawTextEx(font, TextFormat("Downlink:  %.4f MHz", downlinkHz / 1e6),  {sx, oy + 136}, 14, 1, LIGHTGRAY);

        // Keplerian elements
        if (kepEl.valid) {
            float ky = oy + 170.0f;
            DrawTextEx(font_bold, "ORBITAL ELEMENTS (2-BODY)", {sx, ky}, 16, 1.5f, (Color){60,230,220,255});
            DrawLine((int)sx, (int)(ky + 22), (int)(screenW - 20), (int)(ky + 22), GRAY);
            float ry = ky + 30.0f;
            auto rowf = [&](const char* s) { DrawTextEx(font, s, {sx, ry}, 16, 1, RAYWHITE); ry += 22.0f; };
            rowf(TextFormat("Semi-major axis: %.1f km",  kepEl.a));
            rowf(TextFormat("Eccentricity:    %.5f",     kepEl.e));
            rowf(TextFormat("Period:          %.2f min", kepEl.period / 60.0));
            rowf(TextFormat("Apogee alt:      %.1f km",  kepEl.apogeeAltKm()));
            rowf(TextFormat("Perigee alt:     %.1f km",  kepEl.perigeeAltKm()));
            DrawTextEx(font, "cyan = 2-body  /  orange = SGP4", {sx, ry + 2.0f}, 13, 1, (Color){150,150,150,255});
        }

        // Active-tab load status + add/refresh controls
        {
            float by = screenH - 96.0f;
            if (!A.loadStatus.empty())
                DrawTextEx(font, A.loadStatus.c_str(), {sx, by}, 13, 1, LIGHTGRAY);
            if (GuiButton((Rectangle){sx, by + 18.0f, 150.0f, 26.0f}, "Browse catalog..."))
                openCatalog();
            if (GuiButton((Rectangle){sx, by + 48.0f, 150.0f, 26.0f}, "Refresh TLE") && !loading) {
                A.loadStatus = "Refreshing...";
                A.loadFuture = std::async(std::launch::async, loadSatelliteData, A.id);
            }
        }

        // ── Station / link editor (toggle E) ───────────────────────────────────
        if (showEditor) {
            if (editorSynced != selectedStation) syncEditorBuffers();
            float ew = 320.0f, eh = 360.0f;
            float ex = (screenW - ew) * 0.5f, ey = (screenH - eh) * 0.5f;
            DrawRectangleRec((Rectangle){ex, ey, ew, eh}, ColorAlpha((Color){20,24,32,255}, 0.97f));
            DrawRectangleLinesEx((Rectangle){ex, ey, ew, eh}, 2, SKYBLUE);
            DrawTextEx(font_bold, "EDIT STATION / LINK", {ex + 16, ey + 12}, 18, 1.5f, SKYBLUE);

            const char* labels[6] = {"Name", "Latitude (deg)", "Longitude (deg)",
                                     "Altitude (km)", "Elev mask (deg)", "Downlink (MHz)"};
            char* bufs[6] = {nameBuf, latBuf, lonBuf, altBuf, maskBuf, freqBuf};
            int   sizes[6] = {(int)sizeof(nameBuf), (int)sizeof(latBuf), (int)sizeof(lonBuf),
                              (int)sizeof(altBuf), (int)sizeof(maskBuf), (int)sizeof(freqBuf)};
            for (int f = 0; f < 6; f++) {
                float fy = ey + 48.0f + f * 40.0f;
                DrawTextEx(font, labels[f], {ex + 16, fy}, 14, 1, LIGHTGRAY);
                if (GuiTextBox((Rectangle){ex + 150, fy - 2, 150, 26}, bufs[f], sizes[f],
                               activeEditField == f))
                    activeEditField = (activeEditField == f) ? -1 : f;
            }

            float byrow = ey + eh - 84.0f;
            if (GuiButton((Rectangle){ex + 16, byrow, 88, 28}, "Apply")) {
                activeEditField = -1;
                stations[selectedStation].name    = nameBuf;
                stations[selectedStation].lat     = (float)atof(latBuf);
                stations[selectedStation].lon     = (float)atof(lonBuf);
                stations[selectedStation].altKm   = (float)atof(altBuf);
                stations[selectedStation].maskDeg = (float)atof(maskBuf);
                downlinkHz = atof(freqBuf) * 1e6;
                markAllPassRecompute();
            }
            if (GuiButton((Rectangle){ex + 112, byrow, 88, 28}, "Add")) {
                activeEditField = -1;
                stations.push_back({ "NEW", 0.0f, 0.0f, 0.0f, 5.0f,
                                     stationPalette((int)stations.size()) });
                selectedStation = (int)stations.size() - 1;
                syncEditorBuffers();
                markAllPassRecompute();
            }
            if (GuiButton((Rectangle){ex + 208, byrow, 96, 28}, "Delete") && stations.size() > 1) {
                activeEditField = -1;
                stations.erase(stations.begin() + selectedStation);
                if (selectedStation >= (int)stations.size()) selectedStation = (int)stations.size() - 1;
                syncEditorBuffers();
                markAllPassRecompute();
            }
            if (GuiButton((Rectangle){ex + 16, byrow + 36.0f, 288, 28}, "Close")) {
                showEditor = false; activeEditField = -1;
            }
            DrawTextEx(font, "Apply commits the fields. Add/Delete manage stations.",
                       {ex + 16, byrow + 70.0f}, 12, 1, GRAY);
        }

        // ── Tab bar (top strip, up to the sidebar) ──────────────────────────────
        {
            float barW = screenW - sidebarWidth;
            DrawRectangleRec((Rectangle){0, 0, barW, TAB_BAR_H}, (Color){18, 20, 26, 235});
            DrawLineEx({0, TAB_BAR_H}, {barW, TAB_BAR_H}, 1.0f, (Color){60, 60, 70, 255});
            Vector2 mp = GetMousePosition();
            const float TW = 150.0f, TH = TAB_BAR_H - 6.0f;
            float tx = 6.0f;
            for (int i = 0; i < (int)tabs.size(); i++) {
                Rectangle r = {tx, 3.0f, TW, TH};
                bool act = (i == activeTab);
                DrawRectangleRec(r, act ? (Color){44, 70, 110, 255} : (Color){32, 34, 42, 255});
                DrawRectangleLinesEx(r, 1.0f, act ? SKYBLUE : (Color){70, 70, 82, 255});
                std::string title = tabs[i].tle ? tabs[i].tle->Name() : ("Loading " + tabs[i].id);
                if (title.size() > 16) title = title.substr(0, 15) + "…";
                DrawTextEx(font, title.c_str(), {r.x + 8, r.y + 5}, 15, 1,
                           act ? RAYWHITE : (Color){190, 190, 200, 255});
                Rectangle xr = {r.x + TW - 19.0f, r.y + 4.0f, 15.0f, 15.0f};
                bool xhover = CheckCollisionPointRec(mp, xr);
                DrawTextEx(font_bold, "x", {xr.x + 3.0f, xr.y - 1.0f}, 16, 1, xhover ? RED : GRAY);
                if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                    if (xhover)                          reqClose    = i;
                    else if (CheckCollisionPointRec(mp, r)) reqActivate = i;
                }
                tx += TW + 4.0f;
            }
            if ((int)tabs.size() < MAX_TABS) {       // hide '+' once the 7-tab cap is reached
                Rectangle pr = {tx, 3.0f, TH, TH};
                bool phover = CheckCollisionPointRec(mp, pr);
                DrawRectangleRec(pr, phover ? (Color){44, 70, 110, 255} : (Color){32, 34, 42, 255});
                DrawRectangleLinesEx(pr, 1.0f, (Color){70, 70, 82, 255});
                DrawTextEx(font_bold, "+", {pr.x + 7.0f, pr.y + 2.0f}, 20, 1, RAYWHITE);
                if (phover && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) { showAddOverlay = true; addEditMode = true; }
            }
        }

        // ── Add-satellite overlay (opened by '+') ───────────────────────────────
        if (showAddOverlay) {
            DrawRectangleRec((Rectangle){0, 0, screenW, screenH}, ColorAlpha(BLACK, 0.55f));
            float cx = screenW * 0.5f, cy = screenH * 0.5f, pw = 360.0f, ph = 210.0f;
            DrawRectangleRec((Rectangle){cx - pw * 0.5f, cy - ph * 0.5f, pw, ph}, (Color){26, 28, 34, 255});
            DrawRectangleLinesEx((Rectangle){cx - pw * 0.5f, cy - ph * 0.5f, pw, ph}, 2, SKYBLUE);
            drawSatQuestionIcon(cx, cy - 52.0f);
            float bw = 200.0f, bh = 30.0f, bx = cx - (bw + 80.0f) * 0.5f, by = cy + 28.0f;
            DrawTextEx(font, "Add satellite by NORAD ID:", {bx, by - 22.0f}, 15, 1, LIGHTGRAY);
            if (GuiTextBox((Rectangle){bx, by, bw, bh}, addInput, sizeof(addInput), addEditMode))
                addEditMode = !addEditMode;
            if ((GuiButton((Rectangle){bx + bw + 8.0f, by, 72.0f, bh}, "Add") ||
                 (addEditMode && IsKeyPressed(KEY_ENTER))) && addInput[0]) {
                addEditMode = false; reqAddId = addInput; reqAdd = true;
            }
            if (GuiButton((Rectangle){cx - pw * 0.5f + 16.0f, by + 46.0f, 110.0f, 26.0f}, "Browse..."))
                openCatalog();
            if ((GuiButton((Rectangle){cx + pw * 0.5f - 96.0f, by + 46.0f, 80.0f, 26.0f}, "Cancel") ||
                 (IsKeyPressed(KEY_ESCAPE) && !showCatalog))) {
                showAddOverlay = false; addEditMode = false;
            }
        }

        // Browse-catalog modal (opened from the '+' overlay or the sidebar hint).
        {
            std::string pick = drawCatalogModal();
            if (!pick.empty()) { reqAddId = pick; reqAdd = true; showAddOverlay = false; }
        }

        rlPopMatrix();
        EndDrawing();

        // ── Apply deferred tab-bar actions (after rendering, so the bound active-
        //    tab references stayed valid for the whole frame) ────────────────────
        if (reqActivate >= 0 && reqActivate < (int)tabs.size()) activeTab = reqActivate;
        if (reqClose >= 0) closeTab(reqClose);
        if (reqAdd) { addTab(reqAddId); addInput[0] = '\0'; showAddOverlay = false; }
    }

    for (auto& t : tabs)                         // don't tear down while a load is in flight
        if (t.loadFuture.valid()) t.loadFuture.wait();
    if (targetsReady) {
        UnloadRenderTexture(sceneTarget);
        UnloadRenderTexture(brightTarget);
        UnloadRenderTexture(blurTargetA);
        UnloadRenderTexture(blurTargetB);
    }
    UnloadModel(earthModel);
    UnloadModel(satModel);
    UnloadModel(skybox);
    UnloadTexture(earthTexture);
    UnloadTexture(skyTexture);
    UnloadShader(shader);
    UnloadShader(satShader);
    UnloadShader(skyShader);
    UnloadShader(brightPassShader);
    UnloadShader(blurShader);
    CloseWindow();
    return 0;
}
