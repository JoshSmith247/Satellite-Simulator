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
#include <cstring>
#include <cmath>

int main()
{
    // ── Satellite loading (runtime-reloadable) ────────────────────────────────
    // Held by pointer so a different satellite can be loaded at runtime without
    // tearing down the window. SGP4 keeps its own copy of the elements, but we
    // keep the Tle alive too for its accessors (name, inclination, mean motion).
    std::unique_ptr<libsgp4::Tle>  tle;
    std::unique_ptr<libsgp4::SGP4> sgp4;
    bool        tleFromCache = false;
    std::string loadStatus;     // short message shown in the UI after a load attempt

    const int   FUTURE_POINTS   = 360;
    float       futureInterval  = 1.0f;   // minutes between ground-track samples (set on load)

    std::vector<Vector3> trail;
    std::vector<Vector3> futurePath;
    std::vector<Vector3> keplerPath;
    bool forcePassRecompute = false;

    auto loadSatellite = [&](const std::string& noradID) -> bool {
        bool fromCache = false;
        std::string raw = FetchTLE::fetchTLECached(noradID, &fromCache);
        if (!FetchTLE::validateTLE(raw)) {
            loadStatus = "Load failed: " + noradID;
            TraceLog(LOG_WARNING, "Failed to load TLE for NORAD ID: %s", noradID.c_str());
            return false;
        }
        try {
            auto newTle  = std::make_unique<libsgp4::Tle>(FetchTLE::buildTle(raw));
            auto newSgp4 = std::make_unique<libsgp4::SGP4>(*newTle);
            tle  = std::move(newTle);
            sgp4 = std::move(newSgp4);
        } catch (const std::exception& e) {
            loadStatus = std::string("Bad TLE: ") + e.what();
            TraceLog(LOG_WARNING, "Failed to parse TLE for %s: %s", noradID.c_str(), e.what());
            return false;
        }
        futureInterval     = (float)(1440.0 / tle->MeanMotion()) / FUTURE_POINTS;
        tleFromCache       = fromCache;
        loadStatus         = (fromCache ? "Loaded (cached): " : "Loaded: ") + tle->Name();
        forcePassRecompute = true;
        trail.clear();
        futurePath.clear();
        keplerPath.clear();
        return true;
    };

    std::string target_ID = "25544";  // ISS (Zarya) — default
    if (!loadSatellite(target_ID)) {
        TraceLog(LOG_ERROR, "Could not load initial satellite (no network and no cache).");
        return 1;
    }

    // ── Simulation clock ──────────────────────────────────────────────────────
    SimClock clock;

    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(0, 0, "Satellite Orbit Sim");
    ToggleFullscreen();

    // Determine fullscreen dimensions for render textures
    int mon    = GetCurrentMonitor();
    int sw     = GetMonitorWidth(mon);
    int sh     = GetMonitorHeight(mon);
    int bloomW = sw / 2;
    int bloomH = sh / 2;

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

    const float EARTH_RADIUS      = 5.0f;
    const float SUN_VISUAL_DISTANCE = 2000.0f;
    const float SUN_RADIUS        = EARTH_RADIUS * 109.0f / 200.0f;

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

    // Reduced ambient for dramatic day/night contrast
    float lightIntensity = 1.2f;
    Vector3 ambient      = Vector3Scale({0.08f, 0.08f, 0.10f}, lightIntensity);
    SetShaderValue(shader, ambientLoc, &ambient, SHADER_UNIFORM_VEC3);
    earthModel.materials[0].shader = shader;

    // ── Satellite shader (no atmospheric rim/city-lights) ────────────────────
    Shader satShader    = LoadShader("lighting.vs", "sat_lighting.fs");
    int satLightPosLoc  = GetShaderLocation(satShader, "lightPos");
    int satAmbientLoc   = GetShaderLocation(satShader, "ambient");
    int satViewPosLoc   = GetShaderLocation(satShader, "viewPos");
    SetShaderValue(satShader, satAmbientLoc, &ambient, SHADER_UNIFORM_VEC3);

    // ── Bloom pipeline ───────────────────────────────────────────────────────
    RenderTexture2D sceneTarget  = LoadRenderTexture(sw, sh);
    RenderTexture2D brightTarget = LoadRenderTexture(bloomW, bloomH);
    RenderTexture2D blurTargetA  = LoadRenderTexture(bloomW, bloomH);
    RenderTexture2D blurTargetB  = LoadRenderTexture(bloomW, bloomH);

    // NULL vertex shader → Raylib supplies a passthrough screen-quad shader
    Shader brightPassShader = LoadShader(0, "bloom_bright.fs");
    Shader blurShader       = LoadShader(0, "bloom_blur.fs");

    int bloomThresholdLoc = GetShaderLocation(brightPassShader, "threshold");
    int blurTexelSizeLoc  = GetShaderLocation(blurShader, "texelSize");
    int blurHorizontalLoc = GetShaderLocation(blurShader, "horizontal");

    float   bloomThreshold = 0.72f;
    Vector2 bloomTexelSize = {1.0f / (float)bloomW, 1.0f / (float)bloomH};
    SetShaderValue(brightPassShader, bloomThresholdLoc, &bloomThreshold, SHADER_UNIFORM_FLOAT);
    SetShaderValue(blurShader, blurTexelSizeLoc, &bloomTexelSize, SHADER_UNIFORM_VEC2);

    // Source rectangles with Y-flip for reading render textures
    Rectangle fullSrc = {0, 0, (float)sw,     -(float)sh};
    Rectangle halfSrc = {0, 0, (float)bloomW, -(float)bloomH};
    Rectangle fullDst = {0, 0, (float)sw,      (float)sh};
    Rectangle halfDst = {0, 0, (float)bloomW,  (float)bloomH};

    // ── Orbit data ───────────────────────────────────────────────────────────
    const int   TRAIL_LENGTH    = 200;
    const float FUTURE_UPDATE_S = 5.0f;
    trail.reserve(TRAIL_LENGTH);
    futurePath.reserve(FUTURE_POINTS);
    keplerPath.reserve(FUTURE_POINTS);
    float futureTimer = FUTURE_UPDATE_S;

    OrbitalMechanics::Elements kepEl;  // refreshed with the future track

    // ── Satellite model ──────────────────────────────────────────────────────
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

    // ── Ground stations ──────────────────────────────────────────────────────
    struct GroundStation { const char* name; float lat, lon, altKm, maskDeg; };
    GroundStation groundStations[] = {
        { "WUSat", 38.627f, -90.199f, 0.142f, 5.0f },
        { "0N/0E",  0.0f,    0.0f,    0.0f,   5.0f },
    };
    const int STATION_COUNT = 2;
    int       selectedStation = 0;     // observer used for pass prediction / telemetry

    // ── Radio link (for Doppler) ──────────────────────────────────────────────
    // ISS APRS/voice downlink default; editable concept for any comms CubeSat.
    double downlinkHz = 145.800e6;

    // ── Pass prediction state ─────────────────────────────────────────────────
    std::vector<PassPredict::Pass> passes;
    float passRecomputeTimer = 0.0f;   // wall-clock seconds since last prediction

    int  activeDropdown   = 0;
    bool dropDownEditMode = false;

    // NORAD ID text entry
    char noradInput[16];
    std::strncpy(noradInput, target_ID.c_str(), sizeof(noradInput) - 1);
    noradInput[sizeof(noradInput) - 1] = '\0';
    bool noradEditMode = false;

    // Format helpers
    auto fmtDuration = [](double s) -> std::string {
        if (s < 0) s = 0;
        int t = (int)(s + 0.5);
        int h = t / 3600; int m = (t % 3600) / 60; int sec = t % 60;
        char buf[32];
        if (h > 0) std::snprintf(buf, sizeof(buf), "%dh%02dm%02ds", h, m, sec);
        else       std::snprintf(buf, sizeof(buf), "%02dm%02ds", m, sec);
        return buf;
    };

    // Optional automated capture for verification: SATSIM_AUTOSHOT=N writes
    // satsim_verify.png after N rendered frames and exits. No effect when unset.
    int  autoShotFrame = -1;
    if (const char* s = getenv("SATSIM_AUTOSHOT")) autoShotFrame = atoi(s);
    int  frameCount = 0;

    // ─────────────────────────────────────────────────────────────────────────
    while (!WindowShouldClose())
    {
        bool uiCapturing = dropDownEditMode || noradEditMode;

        // ── Input: camera ──────────────────────────────────────────────────────
        Vector2 mouseDelta = GetMouseDelta();
        float   moveSpeed  = 0.15f;
        Vector3 movement   = {0.0f, 0.0f, 0.0f};
        if (!uiCapturing) {
            if (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP))    movement.x =  moveSpeed;
            if (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN))  movement.x = -moveSpeed;
            if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT))  movement.y = -moveSpeed;
            if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) movement.y =  moveSpeed;
        }

        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && !uiCapturing) {
            HideCursor();
            float rotationSpeed = 0.3f;
            UpdateCameraPro(&camera, movement,
                (Vector3){ mouseDelta.x * rotationSpeed, mouseDelta.y * rotationSpeed, 0.0f },
                0.0f);
        } else {
            ShowCursor();
            if (!uiCapturing)
                UpdateCameraPro(&camera, movement, (Vector3){0,0,0}, GetMouseWheelMove() * 2.0f);
        }

        // ── Input: time + simulation controls ───────────────────────────────────
        if (!uiCapturing) {
            if (IsKeyPressed(KEY_F)) ToggleFullscreen();
            if (IsKeyPressed(KEY_SPACE)) clock.togglePause();
            if (IsKeyPressed(KEY_RIGHT_BRACKET)) clock.setSpeed(fminf(clock.speed() * 2.0, 100000.0));
            if (IsKeyPressed(KEY_LEFT_BRACKET))  clock.setSpeed(fmaxf(clock.speed() * 0.5, 0.25));
            if (IsKeyPressed(KEY_R)) { clock.resetToNow(); forcePassRecompute = true; }
            if (IsKeyPressed(KEY_T)) {
                selectedStation = (selectedStation + 1) % STATION_COUNT;
                forcePassRecompute = true;
            }
            // Jump to the next upcoming pass (10 s before AOS).
            if (IsKeyPressed(KEY_N)) {
                double simU = clock.unixSeconds();
                for (const auto& p : passes) {
                    double aosU = SimClock::toUnixSeconds(p.aos);
                    if (aosU > simU + 1.0) {
                        clock.setUnixSeconds(aosU - 10.0);
                        clock.setSpeed(1.0);
                        forcePassRecompute = true;
                        break;
                    }
                }
            }
        }

        clock.update(GetFrameTime());
        libsgp4::DateTime simDt = clock.dateTime();
        double            jd    = clock.julianDate();

        // ── Simulation update ─────────────────────────────────────────────────
        SunSim::SunState sun = SunSim::GetSunState(jd, SUN_VISUAL_DISTANCE);
        Vector3 sunPos       = sun.position;

        float  earthRotation = (float)EarthSim::getRotationAngle(jd);
        Matrix spin  = MatrixRotateY((earthRotation + 112.5f) * DEG2RAD);
        Matrix tilt  = MatrixRotateZ(23.44f * DEG2RAD);
        earthModel.transform = MatrixMultiply(tilt, spin);

        // geo (lat,lon,alt) → model/world space (see notes preserved below).
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

        Vector3 stationPositions[STATION_COUNT];
        for (int i = 0; i < STATION_COUNT; i++)
            stationPositions[i] = geoToWorld(groundStations[i].lat, groundStations[i].lon, 0.0f);

        Vector3 satPos = trail.empty() ? Vector3Zero() : trail.back();
        try {
            auto sub = FetchTLE::getSubPoint(*sgp4, simDt);
            satPos = geoToWorld(sub[0], sub[1], sub[2] * KM_TO_SCENE);
        } catch (const std::exception& e) {
            TraceLog(LOG_WARNING, "SGP4 propagation error: %s", e.what());
        }

        if ((int)trail.size() >= TRAIL_LENGTH) trail.erase(trail.begin());
        trail.push_back(satPos);

        futureTimer += GetFrameTime();
        if (futureTimer >= FUTURE_UPDATE_S) {
            futureTimer = 0.0f;
            futurePath.clear();
            auto futureSub = FetchTLE::getFutureSubPoints(*sgp4, FUTURE_POINTS, futureInterval, simDt);
            for (auto& s : futureSub)
                futurePath.push_back(geoToModel(s[0], s[1], s[2] * KM_TO_SCENE));

            keplerPath.clear();
            kepEl = OrbitalMechanics::fromStateVector(FetchTLE::getStateVector(*sgp4, simDt));
            for (int i = 0; i < FUTURE_POINTS; i++) {
                double dtSec = (double)i * futureInterval * 60.0;
                auto p = OrbitalMechanics::propagate(kepEl, dtSec);
                libsgp4::CoordGeodetic g =
                    libsgp4::Eci(simDt.AddSeconds(dtSec),
                                 libsgp4::Vector(p[0], p[1], p[2])).ToGeodetic();
                keplerPath.push_back(geoToModel((float)(g.latitude  * RAD2DEG),
                                                (float)(g.longitude * RAD2DEG),
                                                (float)(g.altitude  * KM_TO_SCENE)));
            }
        }

        // ── Observer-relative geometry + pass prediction ───────────────────────
        const GroundStation& obs = groundStations[selectedStation];
        libsgp4::CoordGeodetic obsGeo(obs.lat, obs.lon, obs.altKm);
        PassPredict::LookAngle look =
            PassPredict::lookAngle(*sgp4, obsGeo, simDt, obs.maskDeg);
        double dopplerHz = PassPredict::dopplerShiftedHz(downlinkHz, look.rangeRateKmS) - downlinkHz;

        passRecomputeTimer += GetFrameTime();
        if (forcePassRecompute || passRecomputeTimer >= 2.0f) {
            forcePassRecompute = false;
            passRecomputeTimer = 0.0f;
            try {
                passes = PassPredict::predictPasses(*sgp4, obsGeo, simDt, 24.0, obs.maskDeg);
            } catch (const std::exception& e) {
                TraceLog(LOG_WARNING, "Pass prediction error: %s", e.what());
            }
        }

        // Per-frame shader uniforms
        SetShaderValue(shader,    lightPosLoc,    &sunPos,           SHADER_UNIFORM_VEC3);
        SetShaderValue(shader,    viewPosLoc,     &camera.position,  SHADER_UNIFORM_VEC3);
        SetShaderValue(satShader, satLightPosLoc, &sunPos,           SHADER_UNIFORM_VEC3);
        SetShaderValue(satShader, satViewPosLoc,  &camera.position,  SHADER_UNIFORM_VEC3);

        // ── Phase 1: Render 3D scene to texture ───────────────────────────────
        BeginTextureMode(sceneTarget);
        ClearBackground((Color){2, 2, 15, 255});
        BeginMode3D(camera);
        rlSetClipPlanes(0.1f, 20000.0f);

        // Skybox
        rlDisableDepthMask();
        DrawModel(skybox, camera.position, -100.0f, WHITE);
        rlEnableDepthMask();

        // Earth
        DrawModel(earthModel, Vector3Zero(), EARTH_RADIUS, WHITE);

        // Past trail — billboard quads, alpha-tapered, additive for glow
        int trailCount = (int)trail.size();
        rlDisableBackfaceCulling();
        BeginBlendMode(BLEND_ADDITIVE);
        rlBegin(RL_QUADS);
        for (int i = 1; i < trailCount; i++) {
            float t     = (float)i / (float)trailCount;
            float alpha = t * 200.0f;
            float width = 0.025f + 0.035f * t;     // taper: thin at oldest end

            Vector3 p0  = trail[i - 1];
            Vector3 p1  = trail[i];
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
        int futureCount = (int)futurePath.size();
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

            Vector3 p0  = Vector3Transform(futurePath[i - 1], earthModel.transform);
            Vector3 p1  = Vector3Transform(futurePath[i],     earthModel.transform);
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

        // Two-body Kepler prediction — thin cyan line, for comparison with SGP4 above
        BeginBlendMode(BLEND_ALPHA);
        for (int i = 1; i < (int)keplerPath.size(); i++) {
            Vector3 k0 = Vector3Transform(keplerPath[i - 1], earthModel.transform);
            Vector3 k1 = Vector3Transform(keplerPath[i],     earthModel.transform);
            unsigned char a = (unsigned char)((1.0f - (float)i / FUTURE_POINTS) * 220.0f);
            DrawLine3D(k0, k1, (Color){60, 230, 220, a});
        }
        EndBlendMode();

        // Satellite model + bright beacon (will bloom)
        float satScale = 0.01f;
        DrawModelEx(satModel, satPos,
            (Vector3){0, 1, 0}, 0.0f,
            (Vector3){satScale, satScale, satScale}, WHITE);
        DrawSphere(satPos, 0.07f, {255, 235, 160, 255});

        // Ground stations — per-station color; the selected observer is ringed.
        Color stationColors[STATION_COUNT] = {YELLOW, {80, 220, 255, 255}};
        for (int i = 0; i < STATION_COUNT; i++) {
            Vector3 tip = geoToWorld(groundStations[i].lat, groundStations[i].lon, 0.5f);
            DrawLine3D(stationPositions[i], tip, stationColors[i]);
            DrawSphere(tip, 0.12f, stationColors[i]);
            if (i == selectedStation) {
                // Line-of-sight to the satellite when it is above the horizon.
                if (look.visible)
                    DrawLine3D(stationPositions[i], satPos, (Color){120, 255, 140, 200});
            }
        }

        // Sun — bright core + minimal close-range rings (bloom handles the corona)
        DrawSphere(sunPos, SUN_RADIUS, WHITE);
        rlDisableDepthMask();
        for (int i = 1; i <= 3; i++) {
            float glowRadius = SUN_RADIUS + (i * 28.0f);
            float alpha      = 0.18f / (float)i;
            DrawSphere(sunPos, glowRadius, ColorAlpha(YELLOW, alpha));
        }
        rlEnableDepthMask();

        EndMode3D();
        EndTextureMode();

        // ── Phase 2: Extract bright pixels (half-res) ─────────────────────────
        BeginTextureMode(brightTarget);
        ClearBackground(BLACK);
        BeginShaderMode(brightPassShader);
        DrawTexturePro(sceneTarget.texture, fullSrc, halfDst, {0, 0}, 0.0f, WHITE);
        EndShaderMode();
        EndTextureMode();

        // ── Phase 3: Two-pass Gaussian blur (ping-pong) ───────────────────────
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

        // ── Phase 4: Composite scene + bloom + 2D UI ─────────────────────────
        BeginDrawing();

        fullDst = {0, 0, (float)GetRenderWidth(), (float)GetRenderHeight()};

        // Base scene
        DrawTexturePro(sceneTarget.texture, fullSrc, fullDst, {0, 0}, 0.0f, WHITE);

        // Bloom (additive — blooms in-place over the scene)
        BeginBlendMode(BLEND_ADDITIVE);
        DrawTexturePro(blurTargetB.texture, halfSrc, fullDst, {0, 0}, 0.0f,
                       ColorAlpha(WHITE, 0.85f));
        EndBlendMode();

        // ── 2D UI overlay (authored in logical coords, scaled to framebuffer) ──
        float screenW      = (float)GetScreenWidth();
        float screenH      = (float)GetScreenHeight();
        float sidebarWidth = 300.0f;
        rlPushMatrix();
        rlScalef((float)GetRenderWidth() / screenW, (float)GetRenderHeight() / screenH, 1.0f);

        // Floating station labels — follow the 3D marker so geography is unambiguous
        for (int i = 0; i < STATION_COUNT; i++) {
            if (Vector3DotProduct(Vector3Normalize(stationPositions[i]),
                                  Vector3Normalize(camera.position)) > 0.0f) {
                Vector2 sp = GetWorldToScreen(stationPositions[i], camera);
                DrawTextEx(font, groundStations[i].name,
                           {sp.x + 8.0f, sp.y - 8.0f}, 18, 1, stationColors[i]);
            }
        }

        // Fixed ground station legend — constant screen position, never moves
        {
            const float panelX = 10.0f;
            const float rowH   = 26.0f;
            float panelY  = screenH - (STATION_COUNT * rowH) - 34.0f;
            float panelW  = 200.0f;
            float panelHt = STATION_COUNT * rowH + 28.0f;
            DrawRectangleRec((Rectangle){panelX - 4.0f, panelY - 22.0f, panelW, panelHt},
                              ColorAlpha(BLACK, 0.5f));
            DrawTextEx(font, "GND STATIONS  [T] to select", (Vector2){panelX, panelY - 20.0f}, 14, 1, LIGHTGRAY);
            for (int i = 0; i < STATION_COUNT; i++) {
                float y = panelY + i * rowH;
                DrawRectangleV((Vector2){panelX, y + 4.0f}, (Vector2){12.0f, 12.0f},
                               stationColors[i]);
                const char* mark = (i == selectedStation) ? " *" : "";
                DrawTextEx(font, TextFormat("%s%s", groundStations[i].name, mark),
                           (Vector2){panelX + 18.0f, y}, 16, 1, stationColors[i]);
            }
        }

        // ── Next-passes panel (left side) ──────────────────────────────────────
        {
            const float px = 10.0f, py = 80.0f, pw = 330.0f;
            int rows = (int)passes.size(); if (rows > 6) rows = 6;
            float ph = 64.0f + rows * 38.0f;
            DrawRectangleRec((Rectangle){px, py, pw, ph}, ColorAlpha(BLACK, 0.55f));
            DrawTextEx(font_bold, TextFormat("PASSES @ %s", obs.name),
                       {px + 10, py + 8}, 18, 1.5f, (Color){120,255,140,255});
            DrawTextEx(font, TextFormat("min elev %.0f deg   [N] jump to next", obs.maskDeg),
                       {px + 10, py + 30}, 13, 1, LIGHTGRAY);
            double simU = clock.unixSeconds();
            if (passes.empty()) {
                DrawTextEx(font, "No passes in next 24 h", {px + 10, py + 52}, 15, 1, GRAY);
            }
            for (int i = 0; i < rows; i++) {
                const auto& p = passes[i];
                double aosU = SimClock::toUnixSeconds(p.aos);
                double losU = SimClock::toUnixSeconds(p.los);
                float  ry   = py + 52.0f + i * 38.0f;
                bool   nowUp = simU >= aosU && simU <= losU;
                Color  c    = nowUp ? (Color){120,255,140,255} : RAYWHITE;
                std::string when = nowUp
                    ? "NOW  -" + fmtDuration(losU - simU) + " to LOS"
                    : "in " + fmtDuration(aosU - simU);
                DrawTextEx(font, p.aos.ToString().substr(11, 8).c_str(), {px + 10, ry}, 16, 1, c);
                DrawTextEx(font, TextFormat("max %.0f deg  %s", p.maxElevationDeg, fmtDuration(p.durationSec).c_str()),
                           {px + 92, ry}, 14, 1, c);
                DrawTextEx(font, when.c_str(), {px + 10, ry + 17}, 13, 1, LIGHTGRAY);
            }
        }

        // ── Top-left clock readout ─────────────────────────────────────────────
        DrawTextEx(font, simDt.ToString().substr(0, 19).c_str(), {10, 10}, 20, 2, YELLOW);
        DrawTextEx(font,
            TextFormat("%s   x%g   Day %.2f",
                       clock.isPaused() ? "PAUSED" : "RUN",
                       clock.speed(), sun.dayOfYear),
            {10, 34}, 16, 1, clock.isPaused() ? ORANGE : LIME);
        DrawTextEx(font, "SPACE pause  [ ] speed  R now  N next-pass",
                   {10, 54}, 13, 1, ColorAlpha(RAYWHITE, 0.7f));

        // ── Right sidebar ──────────────────────────────────────────────────────
        Rectangle sidebarRect = {screenW - sidebarWidth, 0, sidebarWidth, screenH};
        DrawRectangleRec(sidebarRect, ColorAlpha(DARKGRAY, 0.7f));
        DrawLineEx({screenW - sidebarWidth, 0}, {screenW - sidebarWidth, screenH}, 2, GRAY);

        float sx = screenW - sidebarWidth + 20;
        DrawTextEx(font_bold, "SATELLITE TELEMETRY", {sx, 20}, 20, 2, SKYBLUE);
        DrawLine((int)sx, 45, (int)(screenW - 20), 45, RAYWHITE);

        int   startY  = 70;
        int   spacing = 26;
        float altitudeKm = (Vector3Length(satPos) - EARTH_RADIUS) * (6371.0f / 5.0f);

        DrawTextEx(font, TextFormat("ID: %s", tle->Name().c_str()),
                   {sx, (float)startY}, 18, 2, RAYWHITE);
        DrawTextEx(font, TextFormat("TLE epoch: %s", tle->Epoch().ToString().substr(0,10).c_str()),
                   {sx, (float)(startY + spacing)}, 15, 1, tleFromCache ? ORANGE : LIME);
        DrawTextEx(font, TextFormat("Altitude: %.2f km", altitudeKm),
                   {sx, (float)(startY + spacing * 2)}, 18, 2, RAYWHITE);
        DrawTextEx(font, TextFormat("Inclination: %.4f deg", tle->Inclination(true)),
                   {sx, (float)(startY + spacing * 3)}, 18, 2, RAYWHITE);
        DrawTextEx(font, TextFormat("Orbital Vel: %.2f km/s",
                   (tle->MeanMotion() * 2 * PI * (6371.0f + altitudeKm)) / 86400.0),
                   {sx, (float)(startY + spacing * 4)}, 18, 2, RAYWHITE);

        // Observer-relative live data (the operational core)
        float oy = startY + spacing * 6.0f;
        DrawTextEx(font_bold, TextFormat("OBSERVER: %s", obs.name), {sx, oy}, 16, 1.5f,
                   (Color){120,255,140,255});
        DrawLine((int)sx, (int)(oy + 20), (int)(screenW - 20), (int)(oy + 20), GRAY);
        Color liveC = look.visible ? (Color){120,255,140,255} : (Color){200,120,120,255};
        DrawTextEx(font, look.visible ? "ABOVE HORIZON" : "below horizon",
                   {sx, oy + 26}, 16, 1, liveC);
        DrawTextEx(font, TextFormat("Azimuth:   %.1f deg", look.azimuthDeg),
                   {sx, oy + 48}, 16, 1, RAYWHITE);
        DrawTextEx(font, TextFormat("Elevation: %.1f deg", look.elevationDeg),
                   {sx, oy + 70}, 16, 1, RAYWHITE);
        DrawTextEx(font, TextFormat("Range:     %.0f km", look.rangeKm),
                   {sx, oy + 92}, 16, 1, RAYWHITE);
        DrawTextEx(font, TextFormat("Doppler:   %+.0f Hz", dopplerHz),
                   {sx, oy + 114}, 16, 1, RAYWHITE);
        DrawTextEx(font, TextFormat("Downlink:  %.4f MHz", downlinkHz / 1e6),
                   {sx, oy + 136}, 14, 1, LIGHTGRAY);

        // Keplerian (two-body) orbital elements derived from the live state vector
        if (kepEl.valid) {
            float kx = sx;
            float ky = oy + 170.0f;
            DrawTextEx(font_bold, "ORBITAL ELEMENTS (2-BODY)", {kx, ky}, 16, 1.5f,
                       (Color){60, 230, 220, 255});
            DrawLine((int)kx, (int)(ky + 22), (int)(screenW - 20), (int)(ky + 22), GRAY);
            float ry = ky + 30.0f;
            auto row = [&](const char* s) {
                DrawTextEx(font, s, {kx, ry}, 16, 1, RAYWHITE); ry += 22.0f;
            };
            row(TextFormat("Semi-major axis: %.1f km",  kepEl.a));
            row(TextFormat("Eccentricity:    %.5f",     kepEl.e));
            row(TextFormat("Period:          %.2f min", kepEl.period / 60.0));
            row(TextFormat("Apogee alt:      %.1f km",  kepEl.apogeeAltKm()));
            row(TextFormat("Perigee alt:     %.1f km",  kepEl.perigeeAltKm()));
            DrawTextEx(font, "cyan = 2-body  /  orange = SGP4",
                       {kx, ry + 2.0f}, 13, 1, (Color){150,150,150,255});
        }

        // NORAD ID entry — load any catalogued satellite at runtime.
        {
            float by = screenH - 80.0f;
            DrawTextEx(font, "Load NORAD ID:", {sx, by - 22}, 15, 1, RAYWHITE);
            Rectangle tb = {sx, by, 110, 28};
            if (GuiTextBox(tb, noradInput, sizeof(noradInput), noradEditMode))
                noradEditMode = !noradEditMode;
            if (GuiButton((Rectangle){sx + 120, by, 70, 28}, "Load") ||
                (noradEditMode && IsKeyPressed(KEY_ENTER))) {
                noradEditMode = false;
                if (loadSatellite(noradInput)) target_ID = noradInput;
            }
            if (!loadStatus.empty())
                DrawTextEx(font, loadStatus.c_str(), {sx, by + 32}, 13, 1, LIGHTGRAY);
        }

        rlPopMatrix();  // end UI scale-to-framebuffer

        EndDrawing();

        if (autoShotFrame >= 0 && ++frameCount >= autoShotFrame) {
            TakeScreenshot("satsim_verify.png");
            TraceLog(LOG_INFO, "AUTOSHOT: wrote satsim_verify.png after %d frames", frameCount);
            break;
        }
    }

    UnloadRenderTexture(sceneTarget);
    UnloadRenderTexture(brightTarget);
    UnloadRenderTexture(blurTargetA);
    UnloadRenderTexture(blurTargetB);
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
