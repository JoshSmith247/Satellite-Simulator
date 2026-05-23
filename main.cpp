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

    const int screenWidth = 0;  // 1200;
    const int screenHeight = 0; // 800;
    // SetConfigFlags(FLAG_FULLSCREEN_MODE);
    InitWindow(screenWidth, screenHeight, "Satellite Orbit Sim");
    ToggleFullscreen();

    Camera3D camera = {0};
    camera.position = (Vector3){20.0f, 20.0f, 20.0f};
    camera.target = (Vector3){0.0f, 0.0f, 0.0f};
    camera.up = (Vector3){0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    SetTargetFPS(60);

    Image earthImage = LoadImage("earth_tx.jpg");
    if (earthImage.data == NULL)
    {
        TraceLog(LOG_ERROR, "Failed to load earth_tx.jpg");
    }
    Texture2D earthTexture = LoadTextureFromImage(earthImage);
    SetTextureFilter(earthTexture, TEXTURE_FILTER_TRILINEAR);
    SetTextureWrap(earthTexture, TEXTURE_WRAP_CLAMP);
    UnloadImage(earthImage);

    Model earthModel = LoadModel("earth_sphere.obj");
    if (earthModel.meshCount == 0)
    {
        TraceLog(LOG_ERROR, "Failed to load earth_sphere.obj");
    }
    earthModel.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = earthTexture;

    const float EARTH_RADIUS = 5.0f;
    const float SUN_VISUAL_DISTANCE = 2000.0f;
    const float SUN_RADIUS = EARTH_RADIUS * 109.0f / 200; // 200 scale down

    Texture2D skyTexture = LoadTexture("stars_tx.jpg");
    SetTextureFilter(skyTexture, TEXTURE_FILTER_BILINEAR);

    Mesh skySphere = GenMeshSphere(1.0f, 96, 96);
    Model skybox = LoadModelFromMesh(skySphere);
    skybox.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = skyTexture;

    Shader skyShader = LoadShader("skybox.vs", "skybox.fs");
    skybox.materials[0].shader = skyShader;

    Shader shader = LoadShader("lighting.vs", "lighting.fs");
    int lightPosLoc = GetShaderLocation(shader, "lightPos");
    int ambientLoc = GetShaderLocation(shader, "ambient");

    float lightIntensity = 5.0;
    Vector3 ambient = Vector3Scale({0.1f, 0.1f, 0.12f}, lightIntensity);
    SetShaderValue(shader, ambientLoc, &ambient, SHADER_UNIFORM_VEC3);
    earthModel.materials[0].shader = shader;

    const int TRAIL_LENGTH = 200;

    std::vector<Vector3> trail;
    trail.reserve(TRAIL_LENGTH);

    const int   FUTURE_POINTS   = 360;   // 12 hours at 2-min intervals
    const float FUTURE_INTERVAL = 2.0f;  // minutes between samples
    const float FUTURE_UPDATE_S = 60.0f; // recompute every 60 seconds

    std::vector<Vector3> futurePath;
    futurePath.reserve(FUTURE_POINTS);
    float futureTimer = FUTURE_UPDATE_S; // trigger immediate first build

    // Mesh  satMesh  = GenMeshSphere(0.25f, 12, 12);
    // Model satModel = LoadModelFromMesh(satMesh);

    Model satModel = LoadModel("satellite.obj");

    satModel.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = GRAY;
    // Add Earth lighting shader:
    satModel.materials[0].shader = shader;

    Font font = LoadFontEx("Roboto-Med.ttf", 96, 0, 0);
    Font font_bold = LoadFontEx("Montserrat-Bold.ttf", 96, 0, 0);

    if (font.texture.id == 0 || font_bold.texture.id == 0) {
        TraceLog(LOG_ERROR, "Failed to load Montserrat.ttf! Using default font.");
        font = GetFontDefault();
        font_bold = GetFontDefault();
    } else {
        // Load the fonts
        SetTextureFilter(font.texture, TEXTURE_FILTER_BILINEAR);
        SetTextureFilter(font_bold.texture, TEXTURE_FILTER_BILINEAR);
    }

    int activeDropdown = 0;
    bool dropDownEditMode = false;

    while (!WindowShouldClose())
    {
        Vector2 mouseDelta = GetMouseDelta();

        float moveSpeed = 0.15f;
        Vector3 movement = { 0.0f, 0.0f, 0.0f };
        if (!dropDownEditMode)
        {
            if (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP))    movement.x =  moveSpeed;
            if (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN))  movement.x = -moveSpeed;
            if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT))  movement.y = -moveSpeed;
            if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) movement.y =  moveSpeed;
        }

        // Right-click drag to rotate; mouse wheel always zooms
        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && !dropDownEditMode)
        {
            HideCursor();
            float rotationSpeed = 0.3f;
            UpdateCameraPro(&camera,
                movement,
                (Vector3){ mouseDelta.x * rotationSpeed, mouseDelta.y * rotationSpeed, 0.0f },
                0.0f
            );
        }
        else
        {
            ShowCursor();
            if (!dropDownEditMode)
            {
                UpdateCameraPro(&camera, movement, (Vector3){ 0.0f, 0.0f, 0.0f }, GetMouseWheelMove() * 2.0f);
            }
        }

        if (IsKeyPressed(KEY_F))
        {
            ToggleFullscreen();
        }

        SunSim::SunState sun = SunSim::GetCurrentSunState(SUN_VISUAL_DISTANCE);
        Vector3 sunPos = sun.position;

        float earthRotation = (float)(EarthSim::getCurrentRotationAngle());

        Matrix spin = MatrixRotateY((earthRotation + 112.5f) * DEG2RAD);
        Matrix tilt = MatrixRotateZ(23.44f * DEG2RAD);
        earthModel.transform = MatrixMultiply(tilt, spin);

        Vector3 satPos = trail.empty() ? Vector3Zero() : trail.back();
        try {
            auto rawPos = FetchTLE::getScenePosition(sgp4);
            satPos = Vector3Transform(
                {rawPos[0], rawPos[1], rawPos[2]},
                MatrixRotateZ(23.44f * DEG2RAD)
            );
        } catch (const std::exception& e) {
            TraceLog(LOG_WARNING, "SGP4 propagation error: %s", e.what());
        }

        if ((int)trail.size() >= TRAIL_LENGTH)
            trail.erase(trail.begin());
        trail.push_back(satPos);

        // Rebuild future path every FUTURE_UPDATE_S seconds
        futureTimer += GetFrameTime();
        if (futureTimer >= FUTURE_UPDATE_S) {
            futureTimer = 0.0f;
            futurePath.clear();
            Matrix futureTilt = MatrixRotateZ(23.44f * DEG2RAD);
            auto rawFuture = FetchTLE::getFuturePositions(sgp4, FUTURE_POINTS, FUTURE_INTERVAL);
            for (auto& p : rawFuture) {
                futurePath.push_back(Vector3Transform({p[0], p[1], p[2]}, futureTilt));
            }
        }

        SetShaderValue(shader, lightPosLoc, &sunPos, SHADER_UNIFORM_VEC3);

        BeginDrawing();
        ClearBackground((Color){2, 2, 15, 255});

        BeginMode3D(camera);
        rlSetClipPlanes(0.1f, 20000.0f);

        rlDisableDepthMask();
        DrawModel(skybox, camera.position, -100.0f, WHITE);
        rlEnableDepthMask();

        DrawModel(earthModel, Vector3Zero(), EARTH_RADIUS, WHITE);

        int trailCount = (int)trail.size();
        for (int i = 1; i < trailCount; i++)
        {
            float t = (float)i / (float)TRAIL_LENGTH;
            unsigned char alpha = (unsigned char)(t * 255);
            Color trailColor = {100, 200, 255, alpha};
            DrawLine3D(trail[i - 1], trail[i], trailColor);
        }

        // 12-hour extrapolated path: amber fading to transparent
        int futureCount = (int)futurePath.size();
        for (int i = 1; i < futureCount; i++)
        {
            float t = (float)i / (float)FUTURE_POINTS;
            unsigned char alpha = (unsigned char)((1.0f - t) * 200);
            Color futureColor = {255, 160, 40, alpha};
            DrawLine3D(futurePath[i - 1], futurePath[i], futureColor);
        }

        float satScale = 0.01f;
        DrawModelEx(satModel, satPos, (Vector3){0, 1, 0}, 0.0f, (Vector3){satScale, satScale, satScale}, WHITE);
        DrawSphere(satPos, 0.12f, RED);

        DrawSphere(sunPos, SUN_RADIUS, WHITE);
        rlDisableDepthMask();
        for (int i = 1; i <= 8; i++)
        {
            float glowRadius = SUN_RADIUS + (i * 25.0f);
            float alpha = 0.15f / (float)i;
            DrawSphere(sunPos, glowRadius, ColorAlpha(YELLOW, alpha));
        }
        rlEnableDepthMask();

        Vector3 visualSunLineEnd = Vector3Scale(Vector3Normalize(sunPos), 15.0f);
        DrawLine3D(Vector3Zero(), visualSunLineEnd, YELLOW);

        DrawGrid(20, 1.0f);
        EndMode3D();

        // Central UI Design

        int monitor = GetCurrentMonitor();
        float sw = IsWindowFullscreen() ? (float)GetMonitorWidth(monitor) : (float)GetScreenWidth();
        float sh = IsWindowFullscreen() ? (float)GetMonitorHeight(monitor) : (float)GetScreenHeight();

        float sidebarWidth = 300.0f;
        Rectangle sidebarRect = {sw - sidebarWidth, 0, sidebarWidth, sh};

        DrawRectangleRec(sidebarRect, ColorAlpha(DARKGRAY, 0.7f));
        DrawLineEx((Vector2){sw - sidebarWidth, 0}, (Vector2){sw - sidebarWidth, sh}, 2, GRAY);

        DrawTextEx(font_bold, "SATELLITE TELEMETRY", Vector2{sw - sidebarWidth + 20, 20}, 20, 2, SKYBLUE);
        DrawLine(sw - sidebarWidth + 20, 45, sw - 20, 45, RAYWHITE);

        int startY = 70;
        int spacing = 30;

        float altitudeKm = (Vector3Length(satPos) - EARTH_RADIUS) * (6371.0f / 5.0f);
        DrawTextEx(font, TextFormat("ID: %s", tle.Name().c_str()), Vector2{sw - sidebarWidth + 20, (float)startY}, 18, 2, RAYWHITE);
        DrawTextEx(font, TextFormat("Status: %s", "ACTIVE"), Vector2{sw - sidebarWidth + 20, (float) startY + spacing}, 18, 2, LIME);
        DrawTextEx(font, TextFormat("Altitude: %.2f km", altitudeKm), Vector2{sw - sidebarWidth + 20, (float) startY + spacing * 2}, 18, 2, RAYWHITE);
        DrawTextEx(font, TextFormat("Inclination: %.4f deg", tle.Inclination(true)), Vector2{sw - sidebarWidth + 20, (float) startY + spacing * 3}, 18, 2, RAYWHITE);
        DrawTextEx(font, TextFormat("Orbital Vel: %.2f km/s", (tle.MeanMotion() * 2 * PI * (6371.0f + altitudeKm)) / 86400.0), Vector2{sw - sidebarWidth + 20, (float) startY + spacing * 4}, 18, 2, RAYWHITE);

        DrawTextEx(font, TextFormat("Day of Year: %.2f", sun.dayOfYear), Vector2{10, 10}, 20, 2, YELLOW);

        if (GuiDropdownBox((Rectangle){ sw - sidebarWidth + 80, (float)startY + spacing * 6, 100, 30 }, "Option 1;Option 2;Option 3", &activeDropdown, dropDownEditMode)) {
            dropDownEditMode = !dropDownEditMode;

            if (!dropDownEditMode) {
                TraceLog(LOG_INFO, "Selected Option Index: %d", activeDropdown);
            }
        }

        EndDrawing();
    }

    UnloadModel(earthModel);
    UnloadModel(satModel);
    UnloadModel(skybox);
    UnloadTexture(earthTexture);
    UnloadTexture(skyTexture);
    UnloadShader(shader);
    UnloadShader(skyShader);
    CloseWindow();
    return 0;
}