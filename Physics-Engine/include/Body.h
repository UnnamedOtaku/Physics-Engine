#pragma once
#include <raylib.h>
#include <raymath.h>

enum ShapeType
{
	Sphere = 0,
	Box
};

class Body
{
private:
	Vector3 _Position;

public:
    Model Mesh;
    bool IsStatic;
    float Radius;
    Vector3 Size;
    ShapeType shapeType;

    Vector3 Position() const
    {
        return _Position;
    }

    void Position(Vector3 position)
    {
        this->_Position = position;
    }

private:
    Body(
        Vector3 position,
        Vector3 size,
        float radius,
        bool isStatic,
        ShapeType shapeType
    );

public:
    void Move(Vector3 amount);
    void MoveTo(Vector3 position);
    static bool CreateSphereBody(Vector3 position, float radius, bool isStatic, Body** body, const char** error);
    static bool CreateBoxBody(Vector3 position, Vector3 size, bool isStatic, Body** body, const char** error);
};