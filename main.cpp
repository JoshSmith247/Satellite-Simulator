#include "raylib.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#include "raymath.h"
#include "rlgl.h"
#include "EarthMath.hpp"
#include "SunMath.hpp"
#include "fetchTLE.hpp"
#include "SGP4.h"
#include "Tle.h"
#include "Eci.h"
#include "CoordGeodetic.h"
#include "DateTime.h"
#include <exception>
#include <vector>
#include <cmath>

int main()
{
    std::string target_ID = "25544";
    std::string TLE_data = FetchTLE::fetchTLE(target_ID);
    if (!FetchTLE::validateTLE(TLE_data)) {
        TraceLog(LOG_ERROR, "Failed to fetch valid TLE for NORAD ID: %s", target_ID.c_str());
        return 1;
    }
    libsgp4::Tle tle = FetchTLE::buildTle(TLE_data);
    libsgp4::SGP4 sgp4(tle);

    // TEMP DIAG: print propagated geodetic sub-point for cross-check
    {
        libsgp4::DateTime now = libsgp4::DateTime::Now(true);
        libsgp4::Eci eci = sgp4.FindPosition(now);
        libsgp4::CoordGeodetic geo = eci.ToGeodetic();
        libsgp4::Vector p = eci.Position();
        TraceLog(LOG_WARNING, "DIAG UTC %s", now.ToString().c_str());
        TraceLog(LOG_WARNING, "DIAG ECI_km %.3f %.3f %.3f", p.x, p.y, p.z);
        TraceLog(LOG_WARNING, "DIAG GEO lat=%.4f lon=%.4f alt_km=%.3f",
                 geo.latitude * RAD2DEG, geo.longitude * RAD2DEG, geo.altitude);
    }

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
    Vector3 ambient      = Vector3Scale({1.0f, 1.0f, 1.0f}, lightIntensity); // TEMP DIAG
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
    const int   FUTURE_POINTS   = 360;
    const float FUTURE_INTERVAL = (float)(1440.0 / tle.MeanMotion()) / FUTURE_POINTS;
    const float FUTURE_UPDATE_S = 5.0f;

    std::vector<Vector3> trail;
    trail.reserve(TRAIL_LENGTH);

    std::vector<Vector3> futurePath;
    futurePath.reserve(FUTURE_POINTS);
    float futureTimer = FUTURE_UPDATE_S;

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
        SetTextureFilter(font.texture,      TEXTURE_FILTER_BILINEAR);
        SetTextureFilter(font_bold.texture, TEXTURE_FILTER_BILINEAR);
    }

    // ── Ground stations ──────────────────────────────────────────────────────
    struct GroundStation { const char* name; float lat, lon; };
    GroundStation groundStations[] = {
        { "WUSat", 38.627f, -90.199f },
        { "0N/0E",  0.0f,    0.0f   },
    };
    const int STATION_COUNT = 2;

    int  activeDropdown   = 0;
    bool dropDownEditMode = false;

    // ─────────────────────────────────────────────────────────────────────────
    while (!WindowShouldClose())
    {
        // ── Input ─────────────────────────────────────────────────────────────
        Vector2 mouseDelta = GetMouseDelta();
        float   moveSpeed  = 0.15f;
        Vector3 movement   = {0.0f, 0.0f, 0.0f};
        if (!dropDownEditMode) {
            if (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP))    movement.x =  moveSpeed;
            if (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN))  movement.x = -moveSpeed;
            if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT))  movement.y = -moveSpeed;
            if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) movement.y =  moveSpeed;
        }

        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && !dropDownEditMode) {
            HideCursor();
            float rotationSpeed = 0.3f;
            UpdateCameraPro(&camera, movement,
                (Vector3){ mouseDelta.x * rotationSpeed, mouseDelta.y * rotationSpeed, 0.0f },
                0.0f);
        } else {
            ShowCursor();
            if (!dropDownEditMode)
                UpdateCameraPro(&camera, movement, (Vector3){0,0,0}, GetMouseWheelMove() * 2.0f);
        }

        if (IsKeyPressed(KEY_F)) ToggleFullscreen();

        // ── Simulation update ─────────────────────────────────────────────────
        SunSim::SunState sun = SunSim::GetCurrentSunState(SUN_VISUAL_DISTANCE);
        Vector3 sunPos       = sun.position;

        float  earthRotation = (float)EarthSim::getCurrentRotationAngle();
        Matrix spin  = MatrixRotateY((earthRotation + 112.5f) * DEG2RAD);
        Matrix tilt  = MatrixRotateZ(23.44f * DEG2RAD);
        earthModel.transform = MatrixMultiply(tilt, spin);

        // Convert geographic (lat_deg, lon_deg, altitude) to world-space position.
        // altitude is in scene units above EARTH_RADIUS (0 = surface).
        // LON_CALIB: model-space azimuth offset for longitude 0. Empirically calibrated
        // against the rendered continents (verified to ~1°); the OBJ-UV-implied 22.5° lands
        // ~22.5° too far east. Nudge this one number if the markers need a final tweak
        // (larger = markers move west).
        const float LON_CALIB = 45.0f;
        auto geoToWorld = [&](float lat_deg, float lon_deg, float altitude) -> Vector3 {
            float latR  = lat_deg * DEG2RAD;
            float alpha = (LON_CALIB - lon_deg) * DEG2RAD;
            float r     = EARTH_RADIUS + altitude;
            Vector3 modelPos = {
                r * cosf(latR) * cosf(alpha),
                r * sinf(latR),
                r * cosf(latR) * sinf(alpha)
            };
            // Use the exact same transform as the textured globe so markers pin to the texture.
            return Vector3Transform(modelPos, earthModel.transform);
        };

        Vector3 stationPositions[STATION_COUNT];
        for (int i = 0; i < STATION_COUNT; i++)
            stationPositions[i] = geoToWorld(groundStations[i].lat, groundStations[i].lon, 0.0f);

        Vector3 satPos = trail.empty() ? Vector3Zero() : trail.back();
        try {
            auto rawPos = FetchTLE::getScenePosition(sgp4);
            satPos = Vector3Transform(
                {rawPos[0], rawPos[1], rawPos[2]},
                MatrixRotateZ(23.44f * DEG2RAD));
        } catch (const std::exception& e) {
            TraceLog(LOG_WARNING, "SGP4 propagation error: %s", e.what());
        }

        if ((int)trail.size() >= TRAIL_LENGTH) trail.erase(trail.begin());
        trail.push_back(satPos);

        // TEMP DIAG: park camera over the ISS dot to read the texture beneath it
        static int _diagFrame = 0; _diagFrame++;
        camera.position = Vector3Scale(Vector3Normalize(satPos), 13.0f);
        camera.target = Vector3Zero(); camera.up = (Vector3){0,1,0};

        futureTimer += GetFrameTime();
        if (futureTimer >= FUTURE_UPDATE_S) {
            futureTimer = 0.0f;
            futurePath.clear();
            double eraBase = EarthSim::getCurrentRotationAngle();
            const double ERA_RATE_DEG_PER_MIN = 360.0 * 1.00273781191135448 / 1440.0;
            auto rawFuture = FetchTLE::getFuturePositions(sgp4, FUTURE_POINTS, FUTURE_INTERVAL);
            for (int i = 0; i < (int)rawFuture.size(); i++) {
                float rx = rawFuture[i][0], ry = rawFuture[i][1], rz = rawFuture[i][2];
                float r       = sqrtf(rx*rx + ry*ry + rz*rz);
                float latR    = asinf(ry / r);
                float eclonR  = atan2f(-rx, -rz);
                double eraI   = eraBase + ERA_RATE_DEG_PER_MIN * i * (double)FUTURE_INTERVAL;
                float glonR   = eclonR - (float)(eraI * DEG2RAD);
                float alphaI  = 22.5f * DEG2RAD - glonR;
                futurePath.push_back({
                    r * cosf(latR) * cosf(alphaI),
                    r * sinf(latR),
                    r * cosf(latR) * sinf(alphaI)
                });
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
            // Connector from satellite to first future point
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

        // Satellite model + bright beacon (will bloom)
        float satScale = 0.01f;
        DrawModelEx(satModel, satPos,
            (Vector3){0, 1, 0}, 0.0f,
            (Vector3){satScale, satScale, satScale}, WHITE);
        DrawSphere(satPos, 0.07f, {255, 235, 160, 255});

        // Ground stations — per-station color matches the fixed 2D legend
        Color stationColors[STATION_COUNT] = {YELLOW, {80, 220, 255, 255}};
        for (int i = 0; i < STATION_COUNT; i++) {
            Vector3 tip = geoToWorld(groundStations[i].lat, groundStations[i].lon, 0.5f);
            DrawLine3D(stationPositions[i], tip, stationColors[i]);
            DrawSphere(tip, 0.12f, stationColors[i]);
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

        // Base scene
        DrawTexturePro(sceneTarget.texture, fullSrc, fullDst, {0, 0}, 0.0f, WHITE);

        // Bloom (additive — blooms in-place over the scene)
        BeginBlendMode(BLEND_ADDITIVE);
        DrawTexturePro(blurTargetB.texture, halfSrc, fullDst, {0, 0}, 0.0f,
                       ColorAlpha(WHITE, 0.85f));
        EndBlendMode();

        // ── 2D UI overlay ─────────────────────────────────────────────────────
        float screenW      = (float)GetScreenWidth();
        float screenH      = (float)GetScreenHeight();
        float sidebarWidth = 300.0f;

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
            float panelW  = 160.0f;
            float panelHt = STATION_COUNT * rowH + 28.0f;
            DrawRectangleRec((Rectangle){panelX - 4.0f, panelY - 22.0f, panelW, panelHt},
                              ColorAlpha(BLACK, 0.5f));
            DrawTextEx(font, "GND STATIONS", (Vector2){panelX, panelY - 20.0f}, 14, 1, LIGHTGRAY);
            for (int i = 0; i < STATION_COUNT; i++) {
                float y = panelY + i * rowH;
                DrawRectangleV((Vector2){panelX, y + 4.0f}, (Vector2){12.0f, 12.0f},
                               stationColors[i]);
                DrawTextEx(font, groundStations[i].name, (Vector2){panelX + 18.0f, y}, 16, 1,
                           stationColors[i]);
            }
        }
        Rectangle sidebarRect = {screenW - sidebarWidth, 0, sidebarWidth, screenH};
        DrawRectangleRec(sidebarRect, ColorAlpha(DARKGRAY, 0.7f));
        DrawLineEx({screenW - sidebarWidth, 0}, {screenW - sidebarWidth, screenH}, 2, GRAY);

        DrawTextEx(font_bold, "SATELLITE TELEMETRY",
                   {screenW - sidebarWidth + 20, 20}, 20, 2, SKYBLUE);
        DrawLine((int)(screenW - sidebarWidth + 20), 45, (int)(screenW - 20), 45, RAYWHITE);

        int   startY  = 70;
        int   spacing = 30;
        float altitudeKm = (Vector3Length(satPos) - EARTH_RADIUS) * (6371.0f / 5.0f);

        DrawTextEx(font, TextFormat("ID: %s", tle.Name().c_str()),
                   {screenW - sidebarWidth + 20, (float)startY}, 18, 2, RAYWHITE);
        DrawTextEx(font, TextFormat("Status: %s", "ACTIVE"),
                   {screenW - sidebarWidth + 20, (float)(startY + spacing)}, 18, 2, LIME);
        DrawTextEx(font, TextFormat("Altitude: %.2f km", altitudeKm),
                   {screenW - sidebarWidth + 20, (float)(startY + spacing * 2)}, 18, 2, RAYWHITE);
        DrawTextEx(font, TextFormat("Inclination: %.4f deg", tle.Inclination(true)),
                   {screenW - sidebarWidth + 20, (float)(startY + spacing * 3)}, 18, 2, RAYWHITE);
        DrawTextEx(font, TextFormat("Orbital Vel: %.2f km/s",
                   (tle.MeanMotion() * 2 * PI * (6371.0f + altitudeKm)) / 86400.0),
                   {screenW - sidebarWidth + 20, (float)(startY + spacing * 4)}, 18, 2, RAYWHITE);

        DrawTextEx(font, TextFormat("Day of Year: %.2f", sun.dayOfYear),
                   {10, 10}, 20, 2, YELLOW);

        if (GuiDropdownBox(
                (Rectangle){screenW - sidebarWidth + 80, (float)(startY + spacing * 6), 100, 30},
                "Option 1;Option 2;Option 3", &activeDropdown, dropDownEditMode)) {
            dropDownEditMode = !dropDownEditMode;
            if (!dropDownEditMode)
                TraceLog(LOG_INFO, "Selected Option Index: %d", activeDropdown);
        }

        if (_diagFrame == 90) TakeScreenshot("satdiag.png");
        EndDrawing();
        if (_diagFrame >= 91) break;
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
