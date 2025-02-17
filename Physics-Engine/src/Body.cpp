#include "Body.h"
#include "World.h"

// Constructor for the Body class
Body::Body(
    Vector3 position,
    Vector3 size,
    float radius,
    float density,
    float mass,
    float restitution,
    float volume,
    bool isStatic,
    ShapeType shapeType,
    Color color
) {
    this->_Position = position;
    this->_LinearVelocity = Vector3Zero();
    //this->rotation = 0.0f;
    //this->rotationalVelocity = 0.0f;

    this->force = Vector3Zero();

    this->Density = density;
    this->Mass = mass;
    this->Restitution = restitution;
    this->Volume = volume;

    this->IsStatic = isStatic;
    this->Radius = radius;
    this->Size = size;
    this->shapeType = shapeType;
    this->color = color;

    if (!this->IsStatic)
    {
        this->InvMass = 1.f / this->Mass;
    }
    else
    {
        this->InvMass = 0.f;
    }

    if (this->shapeType == Box)
    {
        this->vertices = Body::CreateBoxVertices(this->Size);
        this->Triangles = Body::CreateBoxTriangles();
        this->transformedVertices.resize(this->vertices.size());
    }

    this->transformUpdateRequired = true;
    this->aabbUpdateRequired = true;
}

Body::~Body() {
    //printf("deleting\n");
}

std::vector<Vector3> Body::CreateBoxVertices(Vector3 size)
{
    float left = -size.x / 2.f;
    float right = left + size.x;
    float bottom = -size.y / 2.f;
    float top = bottom + size.y;
    float front = -size.z / 2.f;
    float back = front + size.z;

    std::vector<Vector3> vertices;
    vertices.resize(23);

    // Vértices en orden antihorario (mirando desde afuera) para cada cara:
    vertices[0] = { left, top, front };      // Cara frontal
    vertices[1] = { right, top, front };
    vertices[2] = { right, bottom, front };
    vertices[3] = { left, bottom, front };

    vertices[4] = { left, bottom, back };      // Cara trasera
    vertices[5] = { right, bottom, back };
    vertices[6] = { right, top, back };
    vertices[7] = { left, top, back };

    vertices[8] = { left, top, back };        // Cara izquierda
    vertices[9] = { left, top, front };
    vertices[10] = { left, bottom, front };
    vertices[11] = { left, bottom, back };

    vertices[12] = { right, top, front };       // Cara derecha
    vertices[13] = { right, top, back };
    vertices[14] = { right, bottom, back };
    vertices[15] = { right, bottom, front };

    vertices[16] = { left, top, back };        // Cara superior
    vertices[17] = { right, top, back };
    vertices[18] = { right, top, front };
    vertices[19] = { left, top, front };

    vertices[20] = { left, bottom, front };     // Cara inferior
    vertices[21] = { right, bottom, front };
    vertices[22] = { right, bottom, back };
    vertices[23] = { left, bottom, back };

    return vertices;
}

std::vector<int> Body::CreateBoxTriangles() {
    return { 0, 1, 2, 0, 2, 3 }; // Inicialización directa
}

std::vector<Vector3> Body::GetTransformedVertices()
{
    if (this->transformUpdateRequired)
    {
        for (int i = 0; i < vertices.size(); i++)
        {
            Vector3 v = this->vertices[i];
            this->transformedVertices[i] = Vector3Transform(v, GetTransformation({ 1, 1, 1 }, { 0, 0, 0 }, this->_Position));
            //this->transformedVertices[i] = v;
        }
    }

    this->transformUpdateRequired = false;
    return this->transformedVertices;
}

AABB Body::GetAABB()
{
    if (this->aabbUpdateRequired)
    {
        float minX = 1e10;
        float minY = 1e10;
        float minZ = 1e10;
        float maxX = -1e10;
        float maxY = -1e10;
        float maxZ = -1e10;

        if (this->shapeType == Box)
        {
            std::vector<Vector3> vertices = this->GetTransformedVertices();

            for (int i = 0; i < vertices.size(); i++)
            {
                Vector3 v = vertices[i];

                if (v.x < minX) { minX = v.x; }
                if (v.x > maxX) { maxX = v.x; }
                if (v.y < minY) { minY = v.y; }
                if (v.y > maxY) { maxY = v.y; }
                if (v.z < minZ) { minZ = v.z; }
                if (v.z > maxZ) { maxZ = v.z; }
            }
        }
        else if (this->shapeType == Sphere)
        {
            minX = this->_Position.x - this->Radius;
            minY = this->_Position.y - this->Radius;
            minZ = this->_Position.z - this->Radius;
            maxX = this->_Position.x + this->Radius;
            maxY = this->_Position.y + this->Radius;
            maxZ = this->_Position.z + this->Radius;
        }
        else
        {
            TraceLog(LOG_ERROR, "Unknown ShapeType.");
        }

        this->aabb = AABB(minX, minY, minZ, maxX, maxY, maxZ);
    }

    this->aabbUpdateRequired = false;
    return this->aabb;
}

void Body::Step(float time, int iterations)
{
    if (this->IsStatic)
    {
        return;
    }

    time /= (float)iterations;

    // force = mass * acc
    // acc = force / mass;

    Vector3 acceleration = Vector3Scale(this->force, 1 / this->Mass);
    //Vector3 acceleration = this->force;
    this->_LinearVelocity = Vector3Add(this->_LinearVelocity, Vector3Scale(acceleration, time));

    this->_Position = Vector3Add(this->_Position, Vector3Scale(this->_LinearVelocity, time));

    //this->_Rotation = Vector3Add(this->_Rotation, Vector3Scale(this->_RotationalVelocity, time));

    this->force = Vector3Zero();
    this->transformUpdateRequired = true;
    this->aabbUpdateRequired = true;
}

// Method to move the body by a specific amount
void Body::Move(Vector3 amount)
{
    this->_Position = Vector3Add(this->_Position, amount); // Update the position of the body
    this->transformUpdateRequired = true;
    this->aabbUpdateRequired = true;
}

// Method to move the body to a specific position
void Body::MoveTo(Vector3 pos)
{
    this->_Position = pos; // Set the new position of the body
    this->transformUpdateRequired = true;
    this->aabbUpdateRequired = true;
}

void Body::AddForce(Vector3 amount)
{
    this->force = Vector3Add(this->force, amount);
}

// Static method to create a spherical body
bool Body::CreateSphereBody(Vector3 position, float radius, float density, bool isStatic, float restitution, Color color, Body* body, const char** error)
{
    *error = "";

    float volume = radius * radius * radius * PI * 4 / 3;
    restitution = Clamp(restitution, 0.0f, 1.0f);
    float mass = volume * density;

    // Create a new instance of Body with a sphere shape
    *body = Body(position, { 0, 0, 0 }, radius, density, mass, restitution, volume, isStatic, Sphere, color);
    // Load the mesh model for the sphere
    body->Mesh = LoadModelFromMesh(GenMeshSphere(1, 20, 20));
    //body->Transformation = body->GetTransformation({ 1, 1, 1 }, { 0, 0, 0 }, { 0, 0, 0 });
    //body->Mesh.transform = body->Transformation;
    return true;
}

// Static method to create a box-shaped body
bool Body::CreateBoxBody(Vector3 position, Vector3 size, float density, bool isStatic, float restitution, Color color, Body* body, const char** error)
{
    *error = "";

    float volume = size.x * size.y * size.z;
    restitution = Clamp(restitution, 0.0f, 1.0f);
    float mass = volume * density;

    // Create a new instance of Body with a box shape
    *body = Body(position, size, 0.f, density, mass, restitution, volume, isStatic, Box, color);
    // Normalize the size of the box
    Vector3 SizeN = Vector3Normalize(size);
    // Load the mesh model for the box
    body->Mesh = LoadModelFromMesh(GenMeshCube(SizeN.x, SizeN.y, SizeN.z));
    //(*body)->Transformation = (*body)->GetTransformation({ 1, 1, 1 }, { 0, 0, 0 }, { 0, 0, 0 });
    //(*body)->Mesh.transform = (*body)->Transformation;
    return true;
}

Matrix Body::GetTransformation(Vector3 scale, Vector3 rotation, Vector3 position)
{
    Matrix transform = MatrixIdentity();
    transform = MatrixMultiply(transform, MatrixScale(scale.x, scale.y, scale.z));
    transform = MatrixRotateXYZ(rotation);
    transform = MatrixMultiply(transform, MatrixTranslate(position.x, position.y, position.z));
    return transform;
}
