#pragma once
#include <raylib.h>
#include <raymath.h>

class Collisions
{
public:
    static bool IntersectCirclePolygon(Vector3 sphereCenter, float sphereRadius,
        const std::vector<Vector3>& vertices, // Centro del cubo y vértices
        Vector3* normal, float* depth)
    {
        *normal = Vector3Zero();
        *depth = 1e10f;

        // 1. Transformar los vértices del cubo al espacio mundial
        std::vector<Vector3> worldVertices(vertices.size());
        for (size_t i = 0; i < vertices.size(); ++i)
        {
            worldVertices[i] = vertices[i];
        }

        // 2. Colisión contra las caras del cubo

        for (int i = 0; i < 6; ++i) // 6 caras en un cubo
        {
            // Obtener los vértices de la cara actual (en orden antihorario)
            int faceIndex[4] = {
                i * 4, i * 4 + 1, i * 4 + 2, i * 4 + 3
            };

            Vector3 v1 = worldVertices[faceIndex[0]];
            Vector3 v2 = worldVertices[faceIndex[1]];
            Vector3 v3 = worldVertices[faceIndex[2]];

            Vector3 edge1 = Vector3Subtract(v2, v1);
            Vector3 edge2 = Vector3Subtract(v3, v1);

            Vector3 faceNormal = Vector3CrossProduct(edge1, edge2); // Normal de la cara
            faceNormal = Vector3Normalize(faceNormal);

            // Proyectamos el centro de la esfera sobre el plano de la cara
            float projection = Vector3DotProduct(Vector3Subtract(sphereCenter, v1), faceNormal);
            Vector3 projectedCenter = Vector3Subtract(sphereCenter, Vector3Scale(faceNormal, projection));

            // Verificamos si el punto proyectado está dentro del cubo (usando el método barycentric)
            if (IsPointInCubeFace(projectedCenter, worldVertices, faceIndex))
            {
                float distance = std::abs(projection); // Distancia al plano de la cara

                if (distance < sphereRadius)
                {
                    *depth = distance - sphereRadius;
                    *normal = faceNormal;
                    return true; // ¡Colisión!
                }
            }
        }

        // 3. Colisión contra los vértices del cubo (si no hubo colisión con las caras)

        for (const auto& vertex : worldVertices)
        {
            float distance = Vector3Distance(sphereCenter, vertex);
            if (distance < sphereRadius)
            {
                *depth = distance - sphereRadius;
                *normal = Vector3Normalize(Vector3Subtract(sphereCenter, vertex));
                return true; // ¡Colisión!
            }
        }

        return false; // No hay colisión
    }

    // Función auxiliar para verificar si un punto está dentro de una cara de un cubo
    static bool IsPointInCubeFace(Vector3 point, const std::vector<Vector3>& vertices, const int faceIndex[4])
    {
        // Obtener los vértices de la cara
        Vector3 v1 = vertices[faceIndex[0]];
        Vector3 v2 = vertices[faceIndex[1]];
        Vector3 v3 = vertices[faceIndex[2]];
        Vector3 v4 = vertices[faceIndex[3]];

        // Calcular las áreas de los triángulos formados por el punto y los vértices de la cara
        float areaTotal = Vector3Length(Vector3CrossProduct(Vector3Subtract(v2, v1), Vector3Subtract(v3, v1))) * 0.5f +
            Vector3Length(Vector3CrossProduct(Vector3Subtract(v3, v1), Vector3Subtract(v4, v1))) * 0.5f;

        float area1 = Vector3Length(Vector3CrossProduct(Vector3Subtract(v2, v1), Vector3Subtract(point, v1))) * 0.5f;
        float area2 = Vector3Length(Vector3CrossProduct(Vector3Subtract(v3, v2), Vector3Subtract(point, v2))) * 0.5f;
        float area3 = Vector3Length(Vector3CrossProduct(Vector3Subtract(v4, v3), Vector3Subtract(point, v3))) * 0.5f;
        float area4 = Vector3Length(Vector3CrossProduct(Vector3Subtract(v1, v4), Vector3Subtract(point, v4))) * 0.5f;

        // Verificar si la suma de las áreas de los triángulos es igual al área total
        return std::abs(area1 + area2 + area3 + area4 - areaTotal) < 0.001f; // Margen de error para comparar floats
    }

private:
    static int FindClosestPointOnPolygon(Vector3 circleCenter, const std::vector<Vector3>& vertices)
    {
        int result = -1;
        float minDistance = 1e10f; // Sufijo f para float
        size_t vertlen = vertices.size();

        for (size_t i = 0; i < vertlen; i++)
        {
            Vector3 v = vertices[i];
            float distance = Vector3Distance(v, circleCenter);

            if (distance < minDistance)
            {
                minDistance = distance;
                result = i;
            }
        }

        return result;
    }

    static void ProjectCircle(Vector3 center, float radius, Vector3 axis, float* min, float* max)
    {
        Vector3 direction = Vector3Normalize(axis);
        Vector3 directionAndRadius = Vector3Scale(direction, radius);

        Vector3 p1 = Vector3Add(center, directionAndRadius);
        Vector3 p2 = Vector3Subtract(center, directionAndRadius);

        *min = Vector3DotProduct(p1, axis);
        *max = Vector3DotProduct(p2, axis);

        if (min > max)
        {
            // swap the min and max values.
            float t = *min;
            *min = *max;
            *max = t;
        }
    }

public:
    static bool IntersectPolygons(Vector3 centerA, const std::vector<Vector3>& verticesA,
        Vector3 centerB, const std::vector<Vector3>& verticesB,
        Vector3* normal, float* depth)
    {
        *normal = Vector3Zero();
        *depth = 1e10f; // Sufijo f para float

        size_t vertalen = verticesA.size();
        size_t vertblen = verticesB.size();

        // Prueba ejes del polígono A
        for (size_t i = 0; i < vertalen; i++)
        {
            Vector3 va = verticesA[i];
            Vector3 vb = verticesA[(i + 1) % vertalen];

            Vector3 edge = Vector3Subtract(vb, va);
            Vector3 axis = Vector3CrossProduct(edge, Vector3{ 0.0f, 0.0f, 1.0f }); // Asume polígonos en XY
            if (Vector3LengthSqr(axis) == 0.0f) continue; // Evitar división por cero
            axis = Vector3Normalize(axis);

            float minA, maxA, minB, maxB;
            ProjectVertices(verticesA, axis, &minA, &maxA); // Sin Collisions::
            ProjectVertices(verticesB, axis, &minB, &maxB); // Sin Collisions::

            if (minA >= maxB || minB >= maxA) return false;

            float axisDepth = std::min(maxB - minA, maxA - minB); // Usar std::min
            if (axisDepth < *depth)
            {
                *depth = axisDepth;
                *normal = axis;
            }
        }

        // Prueba ejes del polígono B
        for (size_t i = 0; i < vertblen; i++)
        {
            Vector3 va = verticesB[i];
            Vector3 vb = verticesB[(i + 1) % vertblen];

            Vector3 edge = Vector3Subtract(vb, va);
            Vector3 axis = Vector3CrossProduct(edge, Vector3{ 0.0f, 0.0f, 1.0f }); // Asume polígonos en XY
            if (Vector3LengthSqr(axis) == 0.0f) continue; // Evitar división por cero
            axis = Vector3Normalize(axis);

            float minA, maxA, minB, maxB;
            ProjectVertices(verticesA, axis, &minA, &maxA); // Sin Collisions::
            ProjectVertices(verticesB, axis, &minB, &maxB); // Sin Collisions::

            if (minA >= maxB || minB >= maxA) return false;

            float axisDepth = std::min(maxB - minA, maxA - minB); // Usar std::min
            if (axisDepth < *depth)
            {
                *depth = axisDepth;
                *normal = axis;
            }
        }

        Vector3 direction = Vector3Subtract(centerB, centerA);

        if (Vector3DotProduct(direction, *normal) < 0.0f)
        {
            *normal = Vector3Negate(*normal);
        }

        return true;
    }

    static bool IntersectPolygons(std::vector<Vector3> verticesA, std::vector<Vector3> verticesB, Vector3* normal, float* depth)
    {
        *normal = Vector3Zero();
        *depth = 1e10f; // Sufijo f para float

        size_t vertalen = verticesA.size();
        size_t vertblen = verticesB.size();

        // Prueba ejes del polígono A
        for (size_t i = 0; i < vertalen; i++)
        {
            Vector3 va = verticesA[i];
            Vector3 vb = verticesA[(i + 1) % vertalen];

            Vector3 edge = Vector3Subtract(vb, va);
            Vector3 axis = Vector3CrossProduct(edge, Vector3{ 0.0f, 0.0f, 1.0f }); // Asume polígonos en XY
            if (Vector3LengthSqr(axis) == 0.0f) continue; // Evitar división por cero
            axis = Vector3Normalize(axis);

            float minA, maxA, minB, maxB;
            ProjectVertices(verticesA, axis, &minA, &maxA); // Sin Collisions::
            ProjectVertices(verticesB, axis, &minB, &maxB); // Sin Collisions::

            if (minA >= maxB || minB >= maxA) return false;

            float axisDepth = std::min(maxB - minA, maxA - minB); // Usar std::min
            if (axisDepth < *depth)
            {
                *depth = axisDepth;
                *normal = axis;
            }
        }

        // Prueba ejes del polígono B
        for (size_t i = 0; i < vertblen; i++)
        {
            Vector3 va = verticesB[i];
            Vector3 vb = verticesB[(i + 1) % vertblen];

            Vector3 edge = Vector3Subtract(vb, va);
            Vector3 axis = Vector3CrossProduct(edge, Vector3{ 0.0f, 0.0f, 1.0f }); // Asume polígonos en XY
            if (Vector3LengthSqr(axis) == 0.0f) continue; // Evitar división por cero
            axis = Vector3Normalize(axis);

            float minA, maxA, minB, maxB;
            ProjectVertices(verticesA, axis, &minA, &maxA); // Sin Collisions::
            ProjectVertices(verticesB, axis, &minB, &maxB); // Sin Collisions::

            if (minA >= maxB || minB >= maxA) return false;

            float axisDepth = std::min(maxB - minA, maxA - minB); // Usar std::min
            if (axisDepth < *depth)
            {
                *depth = axisDepth;
                *normal = axis;
            }
        }

        Vector3 centerA = FindArithmeticMean(verticesA); // Sin Collisions::
        Vector3 centerB = FindArithmeticMean(verticesB); // Sin Collisions::

        Vector3 direction = Vector3Subtract(centerB, centerA);

        if (Vector3DotProduct(direction, *normal) < 0.0f)
        {
            *normal = Vector3Negate(*normal);
        }

        return true;
    }

private:
    static Vector3 FindArithmeticMean(const std::vector<Vector3>& vertices)
    {
        Vector3 sum = Vector3Zero(); // Inicializar la suma como Vector3Zero()
        size_t vertlen = vertices.size();

        for (const auto& v : vertices) // Bucle for basado en rango (más limpio)
        {
            sum = Vector3Add(sum, v); // Sumar directamente los vectores
        }

        return Vector3Scale(sum, 1.0f / (float)vertlen); // Escalar al final para eficiencia
    }

    static void ProjectVertices(const std::vector<Vector3>& vertices, Vector3 axis, float* min, float* max)
    {
        *min = 1e10f; // Sufijo f para float
        *max = -1e10f; // Sufijo f para float

        for (const auto& v : vertices) // Bucle for basado en rango
        {
            float proj = Vector3DotProduct(v, axis);
            *min = std::min(*min, proj); // Usar std::min
            *max = std::max(*max, proj); // Usar std::max
        }
    }

public:
    static bool IntersectCircles(
        Vector3 centerA, float radiusA,
        Vector3 centerB, float radiusB,
        Vector3 *normal, float *depth)
    {
        *normal = Vector3Zero();
        *depth = 1e10f;

        float distance = Vector3Distance(centerA, centerB);
        float radii = radiusA + radiusB;

        if (distance >= radii)
        {
            return false;
        }

        *normal = Vector3Normalize(Vector3Subtract(centerB, centerA));
        *depth = radii - distance;

        return true;
    }

};