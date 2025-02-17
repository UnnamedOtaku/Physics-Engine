#ifndef COLLISIONS_H
#define COLLISIONS_H

#include "raylib.h"
#include "raymath.h"
#include "AABB.h"
#include "Body.h"
#include <vector>
#include <cmath>

class Collisions {
public:
    // Distancia entre un punto y un segmento en 3D
    static void PointSegmentDistance(const Vector3& p, const Vector3& a, const Vector3& b,
        float& distanceSquared, Vector3& closestPoint);

    // Intersección entre dos AABBs en 3D
    static bool IntersectAABBs(const AABB& a, const AABB& b);

    // Encuentra puntos de contacto entre dos cuerpos en 3D
    static void FindContactPoints(Body& bodyA, Body& bodyB,
        Vector3& contact1, Vector3& contact2, int& contactCount);

    // Colisión entre dos cuerpos en 3D
    static bool Collide(Body& bodyA, Body& bodyB,
        Vector3& normal, float& depth);

    // Colisión entre una esfera y un polígono en 3D
    static bool IntersectSpherePolygon(const Vector3& sphereCenter, float sphereRadius,
        const Vector3& polygonCenter, const std::vector<Vector3>& vertices,
        Vector3& normal, float& depth);

    // Colisión entre dos polígonos en 3D
    static bool IntersectPolygons(const Vector3& centerA, const std::vector<Vector3>& verticesA,
        const Vector3& centerB, const std::vector<Vector3>& verticesB,
        Vector3& normal, float& depth);

    // Colisión entre dos esferas en 3D
    static bool IntersectSpheres(const Vector3& centerA, float radiusA,
        const Vector3& centerB, float radiusB,
        Vector3& normal, float& depth);

private:
    // Proyecta vértices en un eje en 3D
    static void ProjectVertices(const std::vector<Vector3>& vertices, const Vector3& axis,
        float& min, float& max);

    // Proyecta una esfera en un eje en 3D
    static void ProjectSphere(const Vector3& center, float radius, const Vector3& axis,
        float& min, float& max);

    // Encuentra el punto más cercano en un polígono a una esfera en 3D
    static int FindClosestPointOnPolygon(const Vector3& sphereCenter, const std::vector<Vector3>& vertices);
};

#endif // COLLISIONS_H