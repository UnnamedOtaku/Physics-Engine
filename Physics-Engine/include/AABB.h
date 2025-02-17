#ifndef FLATAABB3_H
#define FLATAABB3_H

#include "raylib.h"
#include "raymath.h"

struct AABB {
    Vector3 Min;
    Vector3 Max;

    // Constructor por defecto
    AABB() : Min(Vector3Zero()), Max(Vector3Zero()) {}

    // Constructor con valores mínimos y máximos
    AABB(const Vector3& min, const Vector3& max) : Min(min), Max(max) {}

    // Constructor con valores individuales
    AABB(float minX, float minY, float minZ, float maxX, float maxY, float maxZ)
        : Min({ minX, minY, minZ }), Max({ maxX, maxY, maxZ }) {
    }

    // Verifica si dos AABBs se intersectan
    bool Intersects(const AABB& other) const {
        return !(Max.x <= other.Min.x || Min.x >= other.Max.x ||
            Max.y <= other.Min.y || Min.y >= other.Max.y ||
            Max.z <= other.Min.z || Min.z >= other.Max.z);
    }

    // Obtiene el centro del AABB
    Vector3 GetCenter() const {
        return {
            (Min.x + Max.x) * 0.5f,
            (Min.y + Max.y) * 0.5f,
            (Min.z + Max.z) * 0.5f
        };
    }

    // Obtiene las dimensiones del AABB
    Vector3 GetSize() const {
        return {
            Max.x - Min.x,
            Max.y - Min.y,
            Max.z - Min.z
        };
    }

    // Expande el AABB para incluir un punto
    void ExpandToInclude(const Vector3& point) {
        Min.x = fminf(Min.x, point.x);
        Min.y = fminf(Min.y, point.y);
        Min.z = fminf(Min.z, point.z);

        Max.x = fmaxf(Max.x, point.x);
        Max.y = fmaxf(Max.y, point.y);
        Max.z = fmaxf(Max.z, point.z);
    }

    // Expande el AABB para incluir otro AABB
    void ExpandToInclude(const AABB& other) {
        Min.x = fminf(Min.x, other.Min.x);
        Min.y = fminf(Min.y, other.Min.y);
        Min.z = fminf(Min.z, other.Min.z);

        Max.x = fmaxf(Max.x, other.Max.x);
        Max.y = fmaxf(Max.y, other.Max.y);
        Max.z = fmaxf(Max.z, other.Max.z);
    }
};

#endif // FLATAABB3_H