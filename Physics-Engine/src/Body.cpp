#include "Body.h"

// Constructor for the Body class
Body::Body(Vector3 position, Vector3 size, float radius, bool isStatic, ShapeType shapeType, Color color)
{
    this->_Position = position; // Initialize the position of the body

    this->Mesh = Model(); // Initialize the mesh model

    this->IsStatic = isStatic; // Indicate if the body is static
    this->Radius = radius; // Set the radius of the body
    this->Size = size; // Set the size of the body
    this->shapeType = shapeType; // Set the shape type of the body
    this->color = color;
}

// Method to move the body by a specific amount
void Body::Move(Vector3 amount)
{
    this->_Position = Vector3Add(this->_Position, amount); // Update the position of the body
}

// Method to move the body to a specific position
void Body::MoveTo(Vector3 position)
{
    this->_Position = position; // Set the new position of the body
}

// Static method to create a spherical body
bool Body::CreateSphereBody(Vector3 position, float radius, bool isStatic, Color color, Body** body, const char** error)
{
    *body = nullptr;
    *error = "";

    // Create a new instance of Body with a sphere shape
    *body = new Body(position, { 0, 0, 0 }, radius, isStatic, Sphere, color);
    // Load the mesh model for the sphere
    (*body)->Mesh = LoadModelFromMesh(GenMeshSphere(0.5, 20, 20));
    return true;
}

// Static method to create a box-shaped body
bool Body::CreateBoxBody(Vector3 position, Vector3 size, bool isStatic, Color color, Body** body, const char** error)
{
    *body = nullptr;
    *error = "";

    // Create a new instance of Body with a box shape
    *body = new Body(position, size, 0.f, isStatic, Box, color);
    // Normalize the size of the box
    Vector3 SizeN = Vector3Normalize(size);
    // Load the mesh model for the box
    (*body)->Mesh = LoadModelFromMesh(GenMeshCube(SizeN.x, SizeN.y, SizeN.z));
    return true;
}
