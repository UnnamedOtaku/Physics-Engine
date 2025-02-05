#include <raylib.h>

int main()
{
    const int screenWidth = 800;
    const int screenHeight = 450;
    InitWindow(screenWidth, screenHeight, "Physics Engine");

    Camera camera = { 0 };
    camera.position = { 0.0f, 0.0f, 150.0f };
    camera.target = { 0.0f, 0.0f, 0.0f };
    camera.up = { 0.0f, 1.0f, 0.0f };
    camera.fovy = 60.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    bool showCursor = false;
    DisableCursor();
    SetTargetFPS(60);

    while (!WindowShouldClose())
    {
        if (IsKeyPressed(KEY_SPACE))
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

        UpdateCamera(&camera, CAMERA_FIRST_PERSON);
        BeginDrawing();
        ClearBackground(BLACK);
        BeginMode3D(camera);
        DrawCube({ 0, 0, 0 }, 10, 10, 10, WHITE);
        EndMode3D();
        DrawFPS(10, 10);
        EndDrawing();
    }
    CloseWindow();        // Close window and OpenGL context

    return 0;
}