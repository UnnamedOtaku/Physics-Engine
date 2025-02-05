// External Includes
#include <raylib.h>
#include <raymath.h>
#include <malloc.h>

// Local Includes
#include "Body.h"

int main()
{
    const int screenWidth = 800;
    const int screenHeight = 450;
    InitWindow(screenWidth, screenHeight, "Physics Engine");

    // Initialize the camera
    Camera camera = { 0 };
    camera.position = { 0.0f, 0.0f, 50.0f };
    camera.target = { 0.0f, 0.0f, 0.0f };
    camera.up = { 0.0f, 1.0f, 0.0f };
    camera.fovy = 60.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    const int bodyCount = 1000;
    Body* bodyList = (Body*)malloc(bodyCount * sizeof(Body));
    if (bodyList == NULL)
    {
        TraceLog(LOG_ERROR, "Failed to allocate memory for bodyList.");
        return -1;
    }

    Body* body;
    const char* error;
    Vector3 pos;
    float rad;
    unsigned char R, G, B;

    for (int i = 0; i < bodyCount; i++)
    {
        pos = { (float)GetRandomValue(-25, 25), (float)GetRandomValue(-25, 25), (float)GetRandomValue(-25, 25) };
        rad = (float)GetRandomValue(1, 5);
        R = GetRandomValue(0, 255);
        G = GetRandomValue(0, 255);
        B = GetRandomValue(0, 255);
        if (!Body::CreateSphereBody(pos, rad, true, { R, G, B, 255 }, &body, &error))
            TraceLog(LOG_ERROR, error);

        bodyList[i] = *body;
    }

    bool showCursor = false;
    DisableCursor();
    SetTargetFPS(60);

    // Main game loop
    while (!WindowShouldClose())
    {
        // Toggle cursor visibility with LEFT SHIFT key
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

        // Update camera
        UpdateCamera(&camera, CAMERA_FREE);

        // Draw everything
        BeginDrawing();
        ClearBackground(BLACK);
        BeginMode3D(camera);
        for (int i = 0; i < bodyCount; i++)
        {
            Body* body = &bodyList[i];
            if (body->shapeType == Sphere)
                DrawModel(body->Mesh, body->Position(), body->Radius, body->color);
            else if (body->shapeType == Box)
                DrawModel(body->Mesh, body->Position(), Vector3Length(body->Size), body->color);
        }
        EndMode3D();
        DrawFPS(10, 10);
        EndDrawing();
    }

    // Close window and OpenGL context
    free(bodyList);
    CloseWindow();

    return 0;
}
