#include "World.h"

World::World()
{
    this->G = 6.674e-11;
    this->broadPhase = Grid;
    this->gridNodeSize = 400;
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
    Vector3 dir;
    iterations = Clamp(iterations, World::MinIterations, World::MaxIterations);
    this->ContactPointsList.clear();
    int columns = 0;

    if (this->broadPhase == Grid)
    {
        this->BuildNodeGrid(&columns);
    }

    for (int it = 0; it < iterations; it++)
    {
        // Movement step
        for (int i = 0; i < this->bodyCount; i++)
        {
            for (int j = 0; j < this->bodyCount; j++)
            {
                if (i == j) continue;
                dir = Vector3Normalize(Vector3Subtract(this->bodyList[j].Position(), this->bodyList[i].Position()));
                float distanceSqr = Vector3DistanceSqr(bodyList[i].Position(), bodyList[j].Position());

                bodyList[i].AddForce(Vector3Scale(dir, G * bodyList[i].Mass * bodyList[j].Mass / distanceSqr));
            }
            this->bodyList[i].Step(time, iterations);
        }

        if (this->broadPhase == BruteForce)
        {
            this->CollisionStepBruteForce();
        }
        else if (this->broadPhase == Grid)
        {
            this->CollisionStepGrid(columns);
        }
    }
}

void World::CollisionStepBruteForce()
{
    this->contactList.clear();
    this->grid.clear();

    // collision step
    for (int i = 0; i < this->bodyList.size() - 1; i++)
    {
        Body& bodyA = this->bodyList[i];
        AABB bodyA_aabb = bodyA.GetAABB();

        for (int j = i + 1; j < this->bodyList.size(); j++)
        {
            Body& bodyB = this->bodyList[j];
            AABB bodyB_aabb = bodyB.GetAABB();

            if (bodyA.IsStatic && bodyB.IsStatic)
            {
                continue;
            }

            if (!Collisions::IntersectAABBs(bodyA_aabb, bodyB_aabb))
            {
                continue;
            }

            Vector3 normal;
            float depth;

            if (Collisions::Collide(bodyA, bodyB, normal, depth))
            {
                if (bodyA.IsStatic)
                {
                    bodyB.Move(Vector3Scale(normal, depth));
                }
                else if (bodyB.IsStatic)
                {
                    bodyA.Move(Vector3Scale(normal, -depth));
                }
                else
                {
                    bodyA.Move(Vector3Scale(normal, -depth / 2.0f));
                    bodyB.Move(Vector3Scale(normal, depth / 2.0f));
                }
                Vector3 contact1, contact2;
                int contactCount;

                Collisions::FindContactPoints(bodyA, bodyB, contact1, contact2, contactCount);
                Manifold contact = Manifold(&bodyA, &bodyB, normal, depth, contact1, contact2, contactCount);
                this->contactList.push_back(contact);
            }
        }
    }

    for (int i = 0; i < this->contactList.size(); i++)
    {
        Manifold *contact = &this->contactList[i];
        this->ResolveCollision(contact);

        if (contact->ContactCount > 0)
        {
            this->ContactPointsList.push_back(contact->Contact1);

            if (contact->ContactCount > 1)
            {
                this->ContactPointsList.push_back(contact->Contact2);
            }
        }
    }
}

void World::BuildNodeGrid(int* columns)
{
    float minX = 1e10f;
    //float minY = 1e10f;
    float maxX = -1e10f;
    //float maxY = -1e10f;

    for (int i = 0; i < this->bodyList.size(); i++)
    {
        Body *body = &this->bodyList[i];
        AABB aabb = body->GetAABB();

        if (aabb.Min.x < minX) { minX = aabb.Min.x; }
        if (aabb.Max.x > maxX) { maxX = aabb.Max.x; }
        //if (aabb.Min.y < minY) { minY = aabb.Min.y; }
        //if (aabb.Max.y > maxY) { maxY = aabb.Max.y; }
    }

    float width = maxX - minX;
    //float height = maxY - minY;

    *columns = ceil(width / this->gridNodeSize);
    //*rows = ceil(height / this->gridNodeSize);
}

void World::CollisionStepGrid(int columns)
{
    this->contactList.clear();

    for (auto& pair : this->grid)
    {
        pair.second.clear();
    }

    for (int i = 0; i < this->bodyCount; i++)
    {
        Body* body = &this->bodyList[i];
        AABB aabb = body->GetAABB();

        int left = (int)floor(aabb.Min.x / this->gridNodeSize);
        int right = (int)ceil(aabb.Max.x / this->gridNodeSize);
        int bottom = (int)floor(aabb.Min.y / this->gridNodeSize);
        int top = (int)ceil(aabb.Max.y / this->gridNodeSize);

        for (int y = bottom; y <= top; y++)
        for (int x = left; x <= right; x++)
        {
            int key = x + y * columns;

            if (grid.find(key) == grid.end())
            {
                this->grid.insert({ key, std::vector<int>() });
            }

            std::vector<int>& node = grid[key];

            if (std::find(node.begin(), node.end(), i) == node.end())
            {
                node.push_back(i);
            }
        }
    }

    for (auto& pair : this->grid)
    {
        std::vector<int>& node = pair.second;
        if (node.size() < 2) continue;

        for (int i = 0; i < node.size() - 1; i++)
        {
            Body& bodyA = this->bodyList[node[i]];
            AABB bodyA_aabb = bodyA.GetAABB();

            for (int j = i + 1; j < node.size(); j++)
            {
                Body& bodyB = this->bodyList[node[j]];
                AABB bodyB_aabb = bodyB.GetAABB();

                if (bodyA.IsStatic && bodyB.IsStatic)
                {
                    continue;
                }

                if (!Collisions::IntersectAABBs(bodyA_aabb, bodyB_aabb))
                {
                    continue;
                }

                if (this->ContactListContainsPair(&bodyA, &bodyB))
                {
                    continue;
                }

                Vector3 normal;
                float depth;

                if (Collisions::Collide(bodyA, bodyB, normal, depth))
                {
                    if (bodyA.IsStatic)
                    {
                        bodyB.Move(Vector3Scale(normal, depth));
                    }
                    else if (bodyB.IsStatic)
                    {
                        bodyA.Move(Vector3Scale(normal, -depth));
                    }
                    else
                    {
                        bodyA.Move(Vector3Scale(normal, -depth / 2.0f));
                        bodyB.Move(Vector3Scale(normal, depth / 2.0f));
                    }
                    Vector3 contact1, contact2;
                    int contactCount;

                    Collisions::FindContactPoints(bodyA, bodyB, contact1, contact2, contactCount);
                    Manifold contact = Manifold(&bodyA, &bodyB, normal, depth, contact1, contact2, contactCount);
                    this->contactList.push_back(contact);
                }
            }
        }
    }

    for (int i = 0; i < this->contactList.size(); i++)
    {
        Manifold* contact = &this->contactList[i];
        this->ResolveCollision(contact);

        if (contact->ContactCount > 0)
        {
            this->ContactPointsList.push_back(contact->Contact1);

            if (contact->ContactCount > 1)
            {
                this->ContactPointsList.push_back(contact->Contact2);
            }
        }
    }
}

bool World::ContactListContainsPair(Body* bodyA, Body* bodyB)
{
    for (int i = 0; i < this->contactList.size(); i++)
    {
        Manifold* manifold = &this->contactList[i];

        if ((manifold->BodyA == bodyA && manifold->BodyB == bodyB) ||
            (manifold->BodyB == bodyA && manifold->BodyA == bodyB))
        {
            return true;
        }
    }

    return false;
}

void World::ResolveCollision(Manifold* contact)
{
    Body* bodyA = contact->BodyA;
    Body* bodyB = contact->BodyB;
    Vector3 normal = contact->Normal;
    float depth = contact->Depth;

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
    bodyB->LinearVelocity(Vector3Add(bodyB->LinearVelocity(), Vector3Scale(impulse, bodyB->InvMass)));
}