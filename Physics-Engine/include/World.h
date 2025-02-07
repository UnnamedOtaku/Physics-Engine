#pragma once
#include <raylib.h>
#include <raymath.h>
#include <vector>

#include "Body.h"
#include "Manifold.h"
#include "Collisions.h"

class World
{
public:
    static int TransformCount;
    static int NoTransformCount;

    static void AddTransformCount() {
        TransformCount++;
    }
    static void AddNoTransformCount() {
        NoTransformCount++;
    }

    static constexpr float MinBodySize = 0.01f * 0.01f * 0.01f;
    static constexpr float MaxBodySize = 64.0f * 64.0f * 64.0f;

    static constexpr float MinDensity = 0.5f;     // g/cm^3
    static constexpr float MaxDensity = 21.4f;

    static constexpr int MinIterations = 1;
    static constexpr int MaxIterations = 128;

private:
    Vector3 gravity;
    float bodyCount = 0;
    std::vector<Body> bodyList;
    std::vector<Manifold> contactList;

public:
    int BodyCount() const
    {
        return this->bodyCount;
    }

    World();
    std::vector<Body>* BodyList() { return &bodyList; }
    void AddBody(Body body);
    bool RemoveBody(int index);
    Body *GetBody(int index);
    void Step(float time, int iterations);
    void ResolveCollision(Manifold contact);
    bool Collide(Body bodyA, Body bodyB, Vector3* normal, float* depth);
};