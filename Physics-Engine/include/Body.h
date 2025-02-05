#pragma once
#include <raylib.h>
#include <raymath.h>

// Enumeration for the type of shape
enum ShapeType
{
    Sphere = 0,
    Box
};

// Class representing a physical body
class Body
{
private:
    Vector3 _Position; // Position of the body

public:
    Model Mesh; // Mesh model of the body
    Color color;
    bool IsStatic; // Indicates if the body is static
    float Radius; // Radius of the body (for spherical shapes)
    Vector3 Size; // Size of the body (for box shapes)
    ShapeType shapeType; // Type of the shape (Sphere or Box)

    // Getter for the position of the body
    Vector3 Position() const
    {
        return _Position;
    }

    // Setter for the position of the body
    void Position(Vector3 position)
    {
        this->_Position = position;
    }

private:
    // Private constructor for the Body class
    Body(
        Vector3 position,
        Vector3 size,
        float radius,
        bool isStatic,
        ShapeType shapeType,
        Color color
    );

public:
    // Method to move the body by a specific amount
    void Move(Vector3 amount);
    // Method to move the body to a specific position
    void MoveTo(Vector3 position);
    // Static method to create a spherical body
    static bool CreateSphereBody(Vector3 position, float radius, bool isStatic, Color color, Body** body, const char** error);
    // Static method to create a box-shaped body
    static bool CreateBoxBody(Vector3 position, Vector3 size, bool isStatic, Color color, Body** body, const char** error);
};
