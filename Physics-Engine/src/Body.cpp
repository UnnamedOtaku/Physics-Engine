#include "Body.h"

Body::Body(Vector3 position, Vector3 size, float radius, bool isStatic, ShapeType shapeType)
{
    this->_Position = position;

    this->IsStatic = isStatic;
    this->Radius = radius;
    this->Size = size;
    this->shapeType = shapeType;
}

void Body::Move(Vector3 amount)
{
    this->_Position = Vector3Add(this->_Position, amount);
}

void Body::MoveTo(Vector3 position)
{
    this->_Position = position;
}

bool Body::CreateSphereBody(Vector3 position, float radius, bool isStatic, Body** body, const char** error)
{
    *body = nullptr;
    *error = "";

    *body = new Body(position, { 0, 0, 0 }, radius, isStatic, Sphere);
    (*body)->Mesh = LoadModelFromMesh(GenMeshSphere(0.5, 20, 20));
    return true;
}

bool Body::CreateBoxBody(Vector3 position, Vector3 size, bool isStatic, Body** body, const char** error)
{
    *body = nullptr;
    *error = "";

    *body = new Body(position, size, 0.f, isStatic, Box);
    Vector3 SizeN = Vector3Normalize(size);
    (*body)->Mesh = LoadModelFromMesh(GenMeshCube(SizeN.x, SizeN.y, SizeN.z));
    return true;
}