#pragma once
#include <raylib.h>
#include <raymath.h>
#include <map>
#include <vector>

#include "Body.h"
#include "Manifold.h"
#include "Collisions.h"

enum BroadPhase
{
    BruteForce = 0,
    Grid
};

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

    static constexpr float MinNodeSize = 1;
    static constexpr float MaxNodeSize = 32;

private:
    float G;
    float bodyCount = 0;
    std::vector<Body> bodyList;
    std::vector<Manifold> contactList;
    std::vector<Vector3> ContactPointsList;
    std::map<int, std::vector<int>> grid;
    BroadPhase broadPhase;
    float gridNodeSize;

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
    void ResolveCollision(Manifold* contact);

private:
    void CollisionStepBruteForce();
    void BuildNodeGrid(int* columns);
    void CollisionStepGrid(int columns);
    bool ContactListContainsPair(Body* bodyA, Body* bodyB);
};