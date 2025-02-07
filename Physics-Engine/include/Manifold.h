#pragma once
#include <raylib.h>
#include <raymath.h>

#include "Body.h"

struct Manifold
{
public:
    Body *BodyA;
    Body *BodyB;
    Vector3 Normal;
    float Depth;
    Vector3 Contact1;
    Vector3 Contact2;
    int ContactCount;

    Manifold(Body *bodyA, Body *bodyB, Vector3 normal, float depth, Vector3 contact1, Vector3 contact2, int contactCount)
    {
        this->BodyA = bodyA;
        this->BodyB = bodyB;
        this->Normal = normal;
        this->Depth = depth;
        this->Contact1 = contact1;
        this->Contact2 = contact2;
        this->ContactCount = contactCount;
    }
};