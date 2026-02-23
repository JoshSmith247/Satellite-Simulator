#include "raylib.h"
#include "raymath.h"
#include <vector>
#include <cmath>

int main() {
    const int screenWidth = 1200;
    const int screenHeight = 800;
    InitWindow(screenWidth, screenHeight, "Satellite Orbit Sim");

    Camera3D camera = { 0 };
    camera.position   = (Vector3){ 20.0f, 20.0f, 20.0f };
    camera.target     = (Vector3){ 0.0f, 0.0f, 0.0f };
    camera.up         = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy       = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    SetTargetFPS(60);

    // ── Texture ──
    Image earthImage = LoadImage("earth_tx.jpg");
    if (earthImage.data == NULL) {
        TraceLog(LOG_ERROR, "Failed to load earth_tx.jpg — check the file is next to your .exe");
    }
    Texture2D earthTexture = LoadTextureFromImage(earthImage);
    SetTextureFilter(earthTexture, TEXTURE_FILTER_TRILINEAR);
    SetTextureWrap(earthTexture, TEXTURE_WRAP_CLAMP);
    UnloadImage(earthImage);

    // ── Earth Model (proper UV sphere from Blender) ─-
    Model earthModel = LoadModel("earth_sphere.obj");
    if (earthModel.meshCount == 0) {
        TraceLog(LOG_ERROR, "Failed to load earth_sphere.obj — check the file is next to your .exe");
    }
    earthModel.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = earthTexture;

    const float EARTH_RADIUS = 5.0f;

    float earthRotation = 0.0f;

    // ── Orbit Parameters ──
    const float ORBIT_RADIUS      = 8.0f;
    const float ORBIT_SPEED       = 0.8f;
    const float ORBIT_INCLINATION = 30.0f;
    const int   TRAIL_LENGTH      = 200;

    float orbitAngle = 0.0f;
    std::vector<Vector3> trail;
    trail.reserve(TRAIL_LENGTH);

    Mesh  satMesh  = GenMeshSphere(0.25f, 12, 12);
    Model satModel = LoadModelFromMesh(satMesh);

    while (!WindowShouldClose()) {

        // ── UPDATE LOOP  ──
        UpdateCamera(&camera, CAMERA_FREE);

        earthRotation += 0.1f;
        Matrix tilt = MatrixRotateX(23.5f * DEG2RAD);
        Matrix spin = MatrixRotateY(earthRotation * DEG2RAD);
        earthModel.transform = MatrixMultiply(tilt, spin);

        orbitAngle += ORBIT_SPEED;
        if (orbitAngle >= 360.0f) orbitAngle -= 360.0f;

        float rad    = orbitAngle * DEG2RAD;
        float incRad = ORBIT_INCLINATION * DEG2RAD;

        Vector3 flatPos = {
            ORBIT_RADIUS * cosf(rad),
            0.0f,
            ORBIT_RADIUS * sinf(rad)
        };
        Vector3 satPos = {
            flatPos.x,
            flatPos.z * sinf(incRad),
            flatPos.z * cosf(incRad)
        };

        if ((int)trail.size() >= TRAIL_LENGTH)
            trail.erase(trail.begin());
        trail.push_back(satPos);

        // ── DRAW ──
        BeginDrawing();
            ClearBackground((Color){ 2, 2, 15, 255 });

            BeginMode3D(camera);

                // EARTH_RADIUS used as scale since OBJ sphere has radius 1
                DrawModel(earthModel, Vector3Zero(), EARTH_RADIUS, WHITE);

                // Orbit trail
                int trailCount = (int)trail.size();
                for (int i = 1; i < trailCount; i++) {
                    float t = (float)i / (float)TRAIL_LENGTH;
                    unsigned char alpha = (unsigned char)(t * 255);
                    Color trailColor = { 100, 200, 255, alpha };
                    DrawLine3D(trail[i - 1], trail[i], trailColor);
                }

                // Satellite
                DrawModel(satModel, satPos, 1.0f, RED);

                DrawGrid(20, 1.0f);

            EndMode3D();

            DrawText(TextFormat("Orbit angle: %.1f deg", orbitAngle), 10, 10, 20, RAYWHITE);
            DrawText("WASD + mouse to move camera", 10, 35, 18, GRAY);

        EndDrawing();
    }

    UnloadModel(earthModel);
    UnloadModel(satModel);
    UnloadTexture(earthTexture);
    CloseWindow();
    return 0;
}