// External Includes
#include <raylib.h>
#include <raymath.h>
#include <vector>

// Local Includes
#include "World.h"
#define RLIGHTS_IMPLEMENTATION
#include "RLights.h"

#if defined(PLATFORM_DESKTOP)
#define GLSL_VERSION            330
#else   // PLATFORM_ANDROID, PLATFORM_WEB
#define GLSL_VERSION            100
#endif

int main()
{
    const int screenWidth = 800;
    const int screenHeight = 600;
    SetConfigFlags(FLAG_MSAA_4X_HINT);  // Enable Multi Sampling Anti Aliasing 4x (if available)
    InitWindow(screenWidth, screenHeight, "Physics Engine");

    // Initialize the camera
    Camera camera = { 0 };
    camera.position = { 0.0f, -102.0f, -400.0f };
    camera.target = { 0.0f, 0.0f, 0.0f };
    camera.up = { 0.0f, 1.0f, 0.0f };
    camera.fovy = 60.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    // Load basic lighting shader
    Shader shader = LoadShader(TextFormat("resources/shaders/glsl%i/lighting.vert", GLSL_VERSION),
        TextFormat("resources/shaders/glsl%i/lighting.frag", GLSL_VERSION));
    // Get some required shader locations
    shader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(shader, "viewPos");
    // NOTE: "matModel" location name is automatically assigned on shader loading, 
    // no need to get the location again if using that uniform name
    //shader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocation(shader, "matModel");

    // Ambient light level (some basic lighting)
    int ambientLoc = GetShaderLocation(shader, "ambient");
    float ambient[4] = { 0.1f, 0.1f, 0.1f, 1.0f };
    SetShaderValue(shader, ambientLoc, ambient, SHADER_UNIFORM_VEC4);

    // Create lights
    Light lights[MAX_LIGHTS] = { 0 };
    lights[0] = CreateLight(LIGHT_POINT, { -2, 300, -2 }, Vector3Zero(), YELLOW, shader);
    lights[1] = CreateLight(LIGHT_POINT, { -2, 300, -2 }, Vector3Zero(), RED, shader);
    lights[2] = CreateLight(LIGHT_POINT, { -2, 300, -2 }, Vector3Zero(), GREEN, shader);
    lights[3] = CreateLight(LIGHT_POINT, { -2, 300, -2 }, Vector3Zero(), BLUE, shader);

    const int bodyCount = 1000;

    World world;
    Body body;
    const char* error;
    Vector3 pos;
    float rad;
    unsigned char R, G, B;

    for (int i = 0; i < bodyCount - 1; i++)
    {
        //Body body;
        rad = (float)GetRandomValue(1, 5);
        pos = { (float)GetRandomValue(-1000, 1000), (float)GetRandomValue(-1000, 1000), (float)GetRandomValue(-1000, 1000) };
        R = GetRandomValue(0, 255);
        G = GetRandomValue(0, 255);
        B = GetRandomValue(0, 255);
        if (!Body::CreateSphereBody(pos, rad, 15.0f, false, 0.5f, { R, G, B, 255 }, &body, &error))
            TraceLog(LOG_ERROR, error);

        body.Mesh.materials[0].shader = shader;
        world.AddBody(body);
    }

    //Body body;
    if (!Body::CreateSphereBody({ 0, -102, 0 }, 150.0f, 1e10f, false, 0.5f, GREEN, &body, &error))
        TraceLog(LOG_ERROR, error);

    body.Mesh.materials[0].shader = shader;
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
        camera.target = world.GetBody(world.BodyCount() - 1)->Position();
        UpdateCamera(&camera, CAMERA_THIRD_PERSON);
        Vector3 dir = Vector3Normalize(Vector3Subtract(camera.position, camera.target));

        // Update the shader with the camera view vector (points towards { 0.0f, 0.0f, 0.0f })
        float cameraPos[3] = { camera.position.x, camera.position.y, camera.position.z };
        SetShaderValue(shader, shader.locs[SHADER_LOC_VECTOR_VIEW], cameraPos, SHADER_UNIFORM_VEC3);

        // Check key inputs to enable/disable lights
        if (IsKeyPressed(KEY_Y)) { lights[0].enabled = !lights[0].enabled; }
        if (IsKeyPressed(KEY_R)) { lights[1].enabled = !lights[1].enabled; }
        if (IsKeyPressed(KEY_G)) { lights[2].enabled = !lights[2].enabled; }
        if (IsKeyPressed(KEY_B)) { lights[3].enabled = !lights[3].enabled; }

        // Update light values (actually, only enable/disable them)
        for (int i = 0; i < MAX_LIGHTS; i++) UpdateLightValues(shader, lights[i]);

        world.Step(GetFrameTime(), 2);

        for (int i = 0; i < world.BodyCount(); i++)
            if (Vector3Distance(camera.position, world.GetBody(i)->Position()) > 5000)
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

            // Draw spheres to show where the lights are
            for (int i = 0; i < MAX_LIGHTS; i++)
            {
                if (Vector3DotProduct(dir, Vector3Normalize(Vector3Subtract(camera.position, lights[i].position))) < -0.2) continue;
                if (lights[i].enabled) DrawModel(lights[i].model, lights[i].position, 1, lights[i].color);
                else DrawModelWires(lights[i].model, lights[i].position, 1, lights[i].color);
            }
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
