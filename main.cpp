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
#include <vector>
#include <cmath>

int main()
{
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

    // Mesh  satMesh  = GenMeshSphere(0.25f, 12, 12);
    // Model satModel = LoadModelFromMesh(satMesh);

    Model satModel = LoadModel("satellite.obj");

    satModel.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = GRAY;
    // Add Earth lighting shader:
    satModel.materials[0].shader = shader;

    std::string target_ID = "25544";
    std::string TLE_data = FetchTLE::fetchTLE(target_ID);
    libsgp4::Tle tle = FetchTLE::buildTle(TLE_data);
    libsgp4::SGP4 sgp4(tle);

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
                GetMouseWheelMove() * 2.0f
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

        Matrix spin = MatrixRotateY(-earthRotation * DEG2RAD);
        Matrix tilt = MatrixRotateZ(23.44f * DEG2RAD);
        earthModel.transform = MatrixMultiply(spin, tilt);

        auto rawPos = FetchTLE::getScenePosition(sgp4);
        Vector3 satPos = {rawPos[0], rawPos[1], rawPos[2]};

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

        // Only draw the satellite label when it's in front of the camera AND not occluded by Earth.
        Vector3 camForward = Vector3Subtract(camera.target, camera.position);
        Vector3 toSat = Vector3Subtract(satPos, camera.position);
        bool inFront = Vector3DotProduct(toSat, camForward) > 0.0f;

        // Ray-sphere intersection: does the ray from camera to satellite hit the Earth?
        bool occluded = false;
        if (inFront)
        {
            Vector3 rayDir = Vector3Normalize(toSat);
            float satDist = Vector3Length(toSat);
            // oc = ray origin relative to sphere center (Earth is at origin)
            Vector3 oc = camera.position;
            float b = Vector3DotProduct(oc, rayDir);
            float c = Vector3DotProduct(oc, oc) - EARTH_RADIUS * EARTH_RADIUS;
            float discriminant = b * b - c;
            if (discriminant >= 0.0f)
            {
                float t = -b - sqrtf(discriminant);
                occluded = (t > 0.0f && t < satDist);
            }
        }

        if (inFront && !occluded)
        {
            Vector2 screenPos = GetWorldToScreen(satPos, camera);
            DrawTextEx(font, "SCALAR", Vector2{screenPos.x - 20, screenPos.y - 40}, 20.0f, 2.0f, RAYWHITE);
            DrawCircle((int)screenPos.x, (int)screenPos.y, 4, RED);
        }

        // Central UI Design

        float sw = (float)GetScreenWidth();
        float sh = (float)GetScreenHeight();

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
        DrawTextEx(font, TextFormat("Orbital Vel: %.2f km/s", (tle.MeanMotion() * 2 * PI * 6371.0) / 86400.0), Vector2{sw - sidebarWidth + 20, (float) startY + spacing * 4}, 18, 2, RAYWHITE);

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