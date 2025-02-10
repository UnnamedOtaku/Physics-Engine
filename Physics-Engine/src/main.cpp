// External Includes
#include <raylib.h>
#include <raymath.h>
#include <vector>

// Local Includes
//#include "Body.h"
#include "World.h"

int main()
{
    const int screenWidth = 800;
    const int screenHeight = 600;
    InitWindow(screenWidth, screenHeight, "Physics Engine");

    // Initialize the camera
    Camera camera = { 0 };
    camera.position = { 0.0f, 0.0f, -1.0f };
    camera.target = { 0.0f, 0.0f, 0.0f };
    camera.up = { 0.0f, 1.0f, 0.0f };
    camera.fovy = 60.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    const int bodyCount = 1000;

    World world;
    //Body body;
    const char* error;
    Vector3 pos;
    float rad;
    unsigned char R, G, B;

    for (int i = 0; i < bodyCount - 1; i++)
    {
        Body body;
        rad = (float)GetRandomValue(1, 5);
        pos = { (float)GetRandomValue(-50, 50), (float)GetRandomValue(rad + 50, 100), (float)GetRandomValue(-50, 50) };
        R = GetRandomValue(0, 255);
        G = GetRandomValue(0, 255);
        B = GetRandomValue(0, 255);
        if (!Body::CreateSphereBody(pos, rad, 15.0f, false, 0.5f, { R, G, B, 255 }, &body, &error))
            TraceLog(LOG_ERROR, error);

        world.AddBody(body);
    }

    Body body;
    if (!Body::CreateSphereBody({ 0, -102, 0 }, 100, 60.0f, true, 0.5f, GREEN, &body, &error))
        TraceLog(LOG_ERROR, error);
    world.AddBody(body);

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
        Vector3 dir = Vector3Normalize(Vector3Subtract(camera.position, camera.target));

        world.Step(GetFrameTime(), 2);

        for (int i = 0; i < world.BodyCount(); i++)
            if (Vector3Distance(camera.position, world.GetBody(i)->Position()) > 2500)
            world.RemoveBody(i);

        // Draw everything
        BeginDrawing();
        ClearBackground(BLACK);
        BeginMode3D(camera);
        for (int i = 0; i < world.BodyCount(); i++)
        {
            Body *body = world.GetBody(i);
            if (body == nullptr) continue;

            if (body->shapeType == Sphere)
            {
                if (Vector3DotProduct(dir, Vector3Normalize(Vector3Subtract(camera.position, body->Position()))) < -0.2) continue;
                DrawModel(body->Mesh, body->Position(), body->Radius, body->color);
            }
            else if (body->shapeType == Box)
                DrawModel(body->Mesh, body->Position(), Vector3Length(body->Size), body->color);
        }
        EndMode3D();
        DrawFPS(10, 10);
        EndDrawing();
    }

    // Close window and OpenGL context
    CloseWindow();
    for (int i = 0; i < world.BodyCount(); i++)
        world.RemoveBody(i);

    return 0;
}
