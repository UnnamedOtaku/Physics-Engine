#include "World.h"

World::World()
{
    this->gravity = { 0.0f, -9.81f, 0.0f };
}

int World::TransformCount = 0;  // Definición e inicialización
int World::NoTransformCount = 0; // Definición e inicialización

void World::AddBody(Body body)
{
    this->bodyList.push_back(body);
    this->bodyCount += 1;
}

bool World::RemoveBody(int index)
{
    if (index < 0 || index >= bodyCount) {
        return false;
    }
    UnloadModel(bodyList[index].Mesh);
    bodyList.erase(bodyList.begin() + index);
    bodyCount--;
    return true;
}

Body *World::GetBody(int index) {
    if (index < 0 || index >= bodyCount) {
        return nullptr; // O lanza una excepción si lo prefieres
    }
    return &bodyList[index]; // Retorna una referencia
}

void World::Step(float time, int iterations)
{
    iterations = Clamp(iterations, World::MinIterations, World::MaxIterations);

    for (int it = 0; it < iterations; it++)
    {
        // Movement step
        for (int i = 0; i < this->bodyCount; i++)
        {
            this->bodyList[i].Step(time, this->gravity, iterations);
        }

        this->contactList.clear();

        // collision step
        for (int i = 0; i < this->bodyCount - 1; i++)
        {
            Body *bodyA = &this->bodyList[i];

            for (int j = i + 1; j < this->bodyCount; j++)
            {
                Body *bodyB = &this->bodyList[j];

                if (bodyA->IsStatic && bodyB->IsStatic)
                {
                    continue;
                }

                Vector3 normal = Vector3Zero();
                float depth = 0;
                if (this->Collide(*bodyA, *bodyB, &normal, &depth))
                {
                    //printf("has collided\n");
                    if (bodyA->IsStatic)
                    {
                        bodyB->Move(Vector3Scale(normal, depth));
                    }
                    else if (bodyB->IsStatic)
                    {
                        bodyA->Move(Vector3Scale(normal, -depth));
                    }
                    else
                    {
                        bodyA->Move(Vector3Scale(normal, -depth / 2.0f));
                        bodyB->Move(Vector3Scale(normal, depth / 2.0f));
                    }

                    Manifold contact = Manifold(bodyA, bodyB, normal, depth, Vector3Zero(), Vector3Zero(), 0);
                    this->contactList.push_back(contact);
                }
            }
        }

        for (int i = 0; i < this->contactList.size(); i++)
        {
            this->ResolveCollision(this->contactList[i]);
        }
    }
}

void World::ResolveCollision(Manifold contact)
{
    Body *bodyA = contact.BodyA;
    Body *bodyB = contact.BodyB;
    Vector3 normal = contact.Normal;
    float depth = contact.Depth;

    Vector3 relativeVelocity = Vector3Subtract(bodyB->LinearVelocity(), bodyA->LinearVelocity());

    if (Vector3DotProduct(relativeVelocity, normal) > 0.0f)
    {
        return;
    }

    float e = fminf(bodyA->Restitution, bodyB->Restitution);

    float j = -(1.0f + e) * Vector3DotProduct(relativeVelocity, normal);
    j /= bodyA->InvMass + bodyB->InvMass;

    Vector3 impulse = Vector3Scale(normal, j);

    bodyA->LinearVelocity(Vector3Subtract(bodyA->LinearVelocity(), Vector3Scale(impulse, bodyA->InvMass)));
    bodyB->LinearVelocity(Vector3Subtract(bodyB->LinearVelocity(), Vector3Scale(impulse, bodyB->InvMass)));
}

bool World::Collide(Body bodyA, Body bodyB, Vector3* normal, float* depth)
{
    *normal = Vector3Zero();
    *depth = 0.0f;

    ShapeType shapeTypeA = bodyA.shapeType;
    ShapeType shapeTypeB = bodyB.shapeType;

    if (shapeTypeA == Box)
    {
        if (shapeTypeB == Box)
        {
            bool result = Collisions::IntersectPolygons(
                bodyA.Position(), bodyA.GetTransformedVertices(),
                bodyB.Position(), bodyB.GetTransformedVertices(),
                normal, depth);
            return result;
        }
        else if (shapeTypeB == Sphere)
        {
            bool result = Collisions::IntersectCirclePolygon(
                bodyB.Position(), bodyB.Radius,
                bodyA.GetTransformedVertices(),
                normal, depth);

            *normal = Vector3Negate(*normal);
            return result;
        }
    }
    else if (shapeTypeA == Sphere)
    {
        if (shapeTypeB == Box)
        {
            bool result = Collisions::IntersectCirclePolygon(
                bodyA.Position(), bodyA.Radius,
                bodyB.GetTransformedVertices(),
                normal, depth);
            return result;
        }
        else if (shapeTypeB == Sphere)
        {
            bool result = Collisions::IntersectCircles(
                bodyA.Position(), bodyA.Radius,
                bodyB.Position(), bodyB.Radius,
                normal, depth);
            return result;
        }
    }

    return false;
}