// External
#include <raylib.h>
#include <raymath.h>

// Local Includes
#include "Body.h"

int main()
{
    const int screenWidth = 800;
    const int screenHeight = 450;
    InitWindow(screenWidth, screenHeight, "Physics Engine");

    Camera camera = { 0 };
    camera.position = { 0.0f, 0.0f, 10.0f };
    camera.target = { 0.0f, 0.0f, 0.0f };
    camera.up = { 0.0f, 1.0f, 0.0f };
    camera.fovy = 60.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    Body *SphereA;
    const char* error;
    if (!Body::CreateSphereBody({ -2.5, 0, 0 }, 5, true, &SphereA, &error))
    {
        TraceLog(LOG_ERROR, error);
    }

    Body* SphereB;
    if (!Body::CreateSphereBody({ 2.5, 0, 0 }, 5, true, &SphereB, &error))
    {
        TraceLog(LOG_ERROR, error);
    }

    Body* Box;
    if (!Body::CreateBoxBody({ 0, 8, 0 }, { 3, 20, 3 }, true, &Box, &error))
    {
        TraceLog(LOG_ERROR, error);
    }

    bool showCursor = false;
    DisableCursor();
    SetTargetFPS(60);

    while (!WindowShouldClose())
    {
        if (IsKeyPressed(KEY_LEFT_SHIFT))
        {
            if (!showCursor)
            {
                showCursor = true;
                EnableCursor();
            }
            else
            {
                showCursor = false;
                DisableCursor();
            }
        }

        UpdateCamera(&camera, CAMERA_FREE);
        BeginDrawing();
        ClearBackground(BLACK);
        BeginMode3D(camera);
        DrawModel(SphereA->Mesh, SphereA->Position(), SphereA->Radius, WHITE);
        DrawModel(SphereB->Mesh, SphereB->Position(), SphereB->Radius, WHITE);
        DrawModel(Box->Mesh, Box->Position(), Vector3Length(Box->Size), WHITE);
        EndMode3D();
        DrawFPS(10, 10);
        EndDrawing();
    }
    CloseWindow();        // Close window and OpenGL context

    return 0;
}