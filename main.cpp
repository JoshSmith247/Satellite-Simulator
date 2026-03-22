#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include "EarthMath.hpp"
#include "SunMath.hpp"
#include "fetchTLE.hpp"
#include "SGP4.h"
#include "Tle.h"
#include <vector>
#include <cmath>

int main()
{
    const int screenWidth = 0;  // 1200;
    const int screenHeight = 0; // 800;
    SetConfigFlags(FLAG_FULLSCREEN_MODE);
    InitWindow(screenWidth, screenHeight, "Satellite Orbit Sim");

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

    const float ORBIT_RADIUS = 8.0f;
    const float ORBIT_SPEED = 0.8f;
    const float ORBIT_INCLINATION = 30.0f;
    const int TRAIL_LENGTH = 200;

    float orbitAngle = 0.0f;
    std::vector<Vector3> trail;
    trail.reserve(TRAIL_LENGTH);

    // Mesh  satMesh  = GenMeshSphere(0.25f, 12, 12);
    // Model satModel = LoadModelFromMesh(satMesh);

    Model satModel = LoadModel("satellite.obj");

    satModel.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = GRAY;
    // Add Earth lighting shader:
    satModel.materials[0].shader = shader;

    std::string target_ID = "25544";
    std::string TLE_data = FetchTLE::fetchTLE(target_ID);
    TLEData parsed = FetchTLE::parseTLE(TLE_data);

    while (!WindowShouldClose())
    {
        UpdateCamera(&camera, CAMERA_FREE);

        if (IsKeyPressed(KEY_F))
        {
            ToggleFullscreen();
        }

        SunSim::SunState sun = SunSim::GetCurrentSunState(SUN_VISUAL_DISTANCE);
        Vector3 sunPos = sun.position;

        float offset = 45.0f;
        float earthRotation = (float)(EarthSim::getCurrentRotationAngle() + offset);

        Matrix spin = MatrixRotateY(earthRotation * DEG2RAD);
        Matrix tilt = MatrixRotateZ(23.44f * DEG2RAD);
        earthModel.transform = MatrixMultiply(spin, tilt);

        Vector3 satPos = FetchTLE::getScenePosition(TLE_data);

        if ((int)trail.size() >= TRAIL_LENGTH)
            trail.erase(trail.begin());
        trail.push_back(satPos);

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

        float satScale = 0.01f;
        DrawModelEx(satModel, satPos, (Vector3){0, 1, 0}, 0.0f, (Vector3){satScale, satScale, satScale}, WHITE);

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

        Vector2 screenPos = GetWorldToScreen(satPos, camera);
        DrawText("SCALAR", (int)screenPos.x - 20, (int)screenPos.y - 40, 20, RAYWHITE);
        DrawCircle((int)screenPos.x, (int)screenPos.y, 4, RED);

        // Central UI Design

        float sw = (float)GetScreenWidth();
        float sh = (float)GetScreenHeight();

        float sidebarWidth = 300.0f;
        Rectangle sidebarRect = {sw - sidebarWidth, 0, sidebarWidth, sh};

        DrawRectangleRec(sidebarRect, ColorAlpha(DARKGRAY, 0.7f));
        DrawLineEx((Vector2){sw - sidebarWidth, 0}, (Vector2){sw - sidebarWidth, sh}, 2, GRAY);

        DrawText("SATELLITE TELEMETRY", sw - sidebarWidth + 20, 20, 20, SKYBLUE);
        DrawLine(sw - sidebarWidth + 20, 45, sw - 20, 45, RAYWHITE);

        int startY = 70;
        int spacing = 30;

        DrawText(TextFormat("ID: %s", parsed.name.c_str()), sw - sidebarWidth + 20, startY, 18, RAYWHITE);
        DrawText(TextFormat("Status: %s", "ACTIVE"), sw - sidebarWidth + 20, startY + spacing, 18, LIME);
        DrawText(TextFormat("Altitude: %.2f km", (ORBIT_RADIUS - EARTH_RADIUS) * 1274.2f), sw - sidebarWidth + 20, startY + spacing * 2, 18, RAYWHITE);
        DrawText(TextFormat("Inclination: %.4f deg", parsed.inclination), sw - sidebarWidth + 20, startY + spacing * 3, 18, RAYWHITE);
        DrawText(TextFormat("Orbital Vel: %.2f km/s", (parsed.meanMotion * 2 * PI * 6371.0) / 86400.0), sw - sidebarWidth + 20, startY + spacing * 4, 18, RAYWHITE);

        DrawText(TextFormat("Orbit angle: %.1f deg", orbitAngle), 10, 10, 20, RAYWHITE);
        DrawText(TextFormat("Day of Year: %.2f", sun.dayOfYear), 10, 35, 20, YELLOW);
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