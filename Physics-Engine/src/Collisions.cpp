#include "Collisions.h"
#include <limits>

void Collisions::PointSegmentDistance(const Vector3& p, const Vector3& a, const Vector3& b,
    float& distanceSquared, Vector3& closestPoint) {
    Vector3 ab = Vector3Subtract(b, a);
    Vector3 ap = Vector3Subtract(p, a);

    float proj = Vector3DotProduct(ap, ab);
    float abLenSq = Vector3DotProduct(ab, ab);
    float d = proj / abLenSq;

    if (d <= 0.0f) {
        closestPoint = a;
    }
    else if (d >= 1.0f) {
        closestPoint = b;
    }
    else {
        closestPoint = Vector3Add(a, Vector3Scale(ab, d));
    }

    distanceSquared = Vector3DotProduct(Vector3Subtract(p, closestPoint), Vector3Subtract(p, closestPoint));
}

bool Collisions::IntersectAABBs(const AABB& a, const AABB& b) {
    return !(a.Max.x <= b.Min.x || b.Max.x <= a.Min.x ||
        a.Max.y <= b.Min.y || b.Max.y <= a.Min.y ||
        a.Max.z <= b.Min.z || b.Max.z <= a.Min.z);
}

bool Collisions::IntersectSpheres(const Vector3& centerA, float radiusA,
    const Vector3& centerB, float radiusB,
    Vector3& normal, float& depth) {
    Vector3 delta = Vector3Subtract(centerB, centerA);
    float distance = Vector3Length(delta);
    float radii = radiusA + radiusB;

    if (distance >= radii) {
        return false;
    }

    normal = Vector3Normalize(delta);
    depth = radii - distance;
    return true;
}

void Collisions::FindContactPoints(Body& bodyA, Body& bodyB,
    Vector3& contact1, Vector3& contact2, int& contactCount) {
    contact1 = Vector3Zero();
    contact2 = Vector3Zero();
    contactCount = 0;

    // Obtener los vértices transformados de los cuerpos
    std::vector<Vector3> verticesA = bodyA.GetTransformedVertices();
    std::vector<Vector3> verticesB = bodyB.GetTransformedVertices();

    float minDistSq = std::numeric_limits<float>::max();

    // Verificar los vértices de A contra los bordes de B
    for (size_t i = 0; i < verticesA.size(); i++) {
        Vector3 p = verticesA[i];

        for (size_t j = 0; j < verticesB.size(); j++) {
            Vector3 va = verticesB[j];
            Vector3 vb = verticesB[(j + 1) % verticesB.size()];

            float distSq;
            Vector3 cp;
            PointSegmentDistance(p, va, vb, distSq, cp);

            if (std::abs(distSq - minDistSq) < 1e-6f) {
                if (!Vector3Equals(cp, contact1) && !Vector3Equals(cp, contact2)) {
                    contact2 = cp;
                    contactCount = 2;
                }
            }
            else if (distSq < minDistSq) {
                minDistSq = distSq;
                contactCount = 1;
                contact1 = cp;
            }
        }
    }

    // Verificar los vértices de B contra los bordes de A
    for (size_t i = 0; i < verticesB.size(); i++) {
        Vector3 p = verticesB[i];

        for (size_t j = 0; j < verticesA.size(); j++) {
            Vector3 va = verticesA[j];
            Vector3 vb = verticesA[(j + 1) % verticesA.size()];

            float distSq;
            Vector3 cp;
            PointSegmentDistance(p, va, vb, distSq, cp);

            if (std::abs(distSq - minDistSq) < 1e-6f) {
                if (!Vector3Equals(cp, contact1) && !Vector3Equals(cp, contact2)) {
                    contact2 = cp;
                    contactCount = 2;
                }
            }
            else if (distSq < minDistSq) {
                minDistSq = distSq;
                contactCount = 1;
                contact1 = cp;
            }
        }
    }
}

bool Collisions::Collide(Body& bodyA, Body& bodyB,
    Vector3& normal, float& depth) {
    normal = Vector3Zero();
    depth = 0.0f;

    // Obtener los vértices transformados de los cuerpos
    std::vector<Vector3> verticesA = bodyA.GetTransformedVertices();
    std::vector<Vector3> verticesB = bodyB.GetTransformedVertices();

    // Verificar colisión entre polígonos
    if (bodyA.shapeType == Box && bodyB.shapeType == Box) {
        return IntersectPolygons(bodyA.Position(), verticesA, bodyB.Position(), verticesB, normal, depth);
    }

    // Verificar colisión entre esfera y polígono
    if (bodyA.shapeType == Sphere && bodyB.shapeType == Box) {
        return IntersectSpherePolygon(bodyA.Position(), bodyA.Radius, bodyB.Position(), verticesB, normal, depth);
    }

    if (bodyA.shapeType == Box && bodyB.shapeType == Sphere) {
        bool result = IntersectSpherePolygon(bodyB.Position(), bodyB.Radius, bodyA.Position(), verticesA, normal, depth);
        normal = Vector3Negate(normal); // Invertir la normal
        return result;
    }

    // Verificar colisión entre esferas
    if (bodyA.shapeType == Sphere && bodyB.shapeType == Sphere) {
        return IntersectSpheres(bodyA.Position(), bodyA.Radius, bodyB.Position(), bodyB.Radius, normal, depth);
    }

    return false;
}

bool Collisions::IntersectSpherePolygon(const Vector3& sphereCenter, float sphereRadius,
    const Vector3& polygonCenter, const std::vector<Vector3>& vertices,
    Vector3& normal, float& depth) {
    normal = Vector3Zero();
    depth = std::numeric_limits<float>::max();

    // Proyectar la esfera y el polígono en cada eje de las aristas del polígono
    for (size_t i = 0; i < vertices.size(); i++) {
        Vector3 va = vertices[i];
        Vector3 vb = vertices[(i + 1) % vertices.size()];

        Vector3 edge = Vector3Subtract(vb, va);
        Vector3 axis = Vector3Normalize({ -edge.y, edge.x, 0 });

        float minA, maxA, minB, maxB;
        ProjectVertices(vertices, axis, minA, maxA);
        ProjectSphere(sphereCenter, sphereRadius, axis, minB, maxB);

        if (minA >= maxB || minB >= maxA) {
            return false;
        }

        float axisDepth = std::min(maxB - minA, maxA - minB);
        if (axisDepth < depth) {
            depth = axisDepth;
            normal = axis;
        }
    }

    // Verificar el eje desde el centro de la esfera al punto más cercano del polígono
    int cpIndex = FindClosestPointOnPolygon(sphereCenter, vertices);
    Vector3 cp = vertices[cpIndex];

    Vector3 axis = Vector3Normalize(Vector3Subtract(cp, sphereCenter));
    float minA, maxA, minB, maxB;
    ProjectVertices(vertices, axis, minA, maxA);
    ProjectSphere(sphereCenter, sphereRadius, axis, minB, maxB);

    if (minA >= maxB || minB >= maxA) {
        return false;
    }

    float axisDepth = std::min(maxB - minA, maxA - minB);
    if (axisDepth < depth) {
        depth = axisDepth;
        normal = axis;
    }

    // Asegurar que la normal apunte desde el polígono hacia la esfera
    Vector3 direction = Vector3Subtract(polygonCenter, sphereCenter);
    if (Vector3DotProduct(direction, normal) < 0.0f) {
        normal = Vector3Negate(normal);
    }

    return true;
}

bool Collisions::IntersectPolygons(const Vector3& centerA, const std::vector<Vector3>& verticesA,
    const Vector3& centerB, const std::vector<Vector3>& verticesB,
    Vector3& normal, float& depth) {
    normal = Vector3Zero();
    depth = std::numeric_limits<float>::max();

    // Verificar las aristas del polígono A
    for (size_t i = 0; i < verticesA.size(); i++) {
        Vector3 va = verticesA[i];
        Vector3 vb = verticesA[(i + 1) % verticesA.size()];

        Vector3 edge = Vector3Subtract(vb, va);
        Vector3 axis = Vector3Normalize({ -edge.y, edge.x, 0 });

        float minA, maxA, minB, maxB;
        ProjectVertices(verticesA, axis, minA, maxA);
        ProjectVertices(verticesB, axis, minB, maxB);

        if (minA >= maxB || minB >= maxA) {
            return false;
        }

        float axisDepth = std::min(maxB - minA, maxA - minB);
        if (axisDepth < depth) {
            depth = axisDepth;
            normal = axis;
        }
    }

    // Verificar las aristas del polígono B
    for (size_t i = 0; i < verticesB.size(); i++) {
        Vector3 va = verticesB[i];
        Vector3 vb = verticesB[(i + 1) % verticesB.size()];

        Vector3 edge = Vector3Subtract(vb, va);
        Vector3 axis = Vector3Normalize({ -edge.y, edge.x, 0 });

        float minA, maxA, minB, maxB;
        ProjectVertices(verticesA, axis, minA, maxA);
        ProjectVertices(verticesB, axis, minB, maxB);

        if (minA >= maxB || minB >= maxA) {
            return false;
        }

        float axisDepth = std::min(maxB - minA, maxA - minB);
        if (axisDepth < depth) {
            depth = axisDepth;
            normal = axis;
        }
    }

    // Asegurar que la normal apunte desde A hacia B
    Vector3 direction = Vector3Subtract(centerB, centerA);
    if (Vector3DotProduct(direction, normal) < 0.0f) {
        normal = Vector3Negate(normal);
    }

    return true;
}

void Collisions::ProjectVertices(const std::vector<Vector3>& vertices, const Vector3& axis,
    float& min, float& max) {
    min = std::numeric_limits<float>::max();
    max = std::numeric_limits<float>::lowest();

    for (const auto& vertex : vertices) {
        float proj = Vector3DotProduct(vertex, axis);
        if (proj < min) min = proj;
        if (proj > max) max = proj;
    }
}

void Collisions::ProjectSphere(const Vector3& center, float radius, const Vector3& axis,
    float& min, float& max) {
    Vector3 direction = Vector3Normalize(axis);
    Vector3 directionAndRadius = Vector3Scale(direction, radius);

    Vector3 p1 = Vector3Add(center, directionAndRadius);
    Vector3 p2 = Vector3Subtract(center, directionAndRadius);

    min = Vector3DotProduct(p1, axis);
    max = Vector3DotProduct(p2, axis);

    if (min > max) {
        std::swap(min, max);
    }
}

int Collisions::FindClosestPointOnPolygon(const Vector3& sphereCenter, const std::vector<Vector3>& vertices) {
    int closestIndex = -1;
    float minDistance = std::numeric_limits<float>::max();

    for (size_t i = 0; i < vertices.size(); i++) {
        float distance = Vector3DotProduct(Vector3Subtract(sphereCenter, vertices[i]), Vector3Subtract(sphereCenter, vertices[i]));
        if (distance < minDistance) {
            minDistance = distance;
            closestIndex = i;
        }
    }

    return closestIndex;
}