#pragma once
#include <raylib.h>
#include <raymath.h>
#include <vector>

class World;

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
    Vector3 _LinearVelocity;

    Vector3 force;

    std::vector<Vector3> vertices;
    std::vector<int> Triangles;
    std::vector<Vector3> transformedVertices;
    bool transformUpdateRequired = true;

public:
    Model Mesh; // Mesh model of the body
    bool DrawMesh = true;
    Matrix Transformation = MatrixIdentity();
    Color color;

    float Density;
    float Mass;
    float InvMass;
    float Restitution;
    float Volume;

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

    Vector3 LinearVelocity() const
    {
        return this->_LinearVelocity;
    }

    void LinearVelocity(Vector3 vel)
    {
        this->_LinearVelocity = vel;
    }

    bool operator==(const Body &otro) const {
        // Aquí defines la lógica de comparación. Por ejemplo:
        return this == &otro;
    }

private:
    // Private constructor for the Body class
    Body(
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
    );

    static std::vector<Vector3> CreateBoxVertices(Vector3 size);
    static std::vector<int> CreateBoxTriangles();

public:
    Body() {}
    std::vector<Vector3> GetTransformedVertices();
    void Step(float time, Vector3 gravity, int iterations);
    // Method to move the body by a specific amount
    void Move(Vector3 amount);
    // Method to move the body to a specific position
    void MoveTo(Vector3 position);
    // Static method to create a spherical body
    static bool CreateSphereBody(Vector3 position, float radius, float density, bool isStatic, float restitution, Color color, Body* body, const char** error);
    // Static method to create a box-shaped body
    static bool CreateBoxBody(Vector3 position, Vector3 size, float density, bool isStatic, float restitution, Color color, Body* body, const char** error);
    Matrix GetTransformation(Vector3 scale, Vector3 rotation, Vector3 position);
};
