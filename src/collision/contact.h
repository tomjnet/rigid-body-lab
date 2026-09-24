#pragma once
#include "../math/math.h"
#include "../core/rigid_body.h"

namespace mc {

struct ContactPoint {
    Vec3 position;       // world-space contact point
    Vec3 localA, localB; // anchors in each body's local frame (for warm-start matching)
    float penetration = 0.0f;

    // Accumulated impulses, persisted across steps (warm starting).
    float normalImpulse = 0.0f;
    float tangentImpulse[2] = {0.0f, 0.0f};

    // Solver scratch, filled in prestep:
    Vec3 rA, rB;
    float normalMass = 0.0f;
    float tangentMass[2] = {0.0f, 0.0f};
    float velocityBias = 0.0f;
};

struct Manifold {
    BodyId a = kInvalidBody, b = kInvalidBody;
    Vec3 normal;         // world-space, points from A to B
    ContactPoint points[4];
    int count = 0;
    float friction = 0.0f;    // combined
    float restitution = 0.0f; // combined
    Vec3 tangent[2];          // friction basis, rebuilt in prestep
};

// Key for the persistent manifold map. Ordered so (a,b) == (b,a) never occurs:
// callers always store with a < b.
struct PairKey {
    BodyId a, b;
    bool operator==(const PairKey& o) const { return a == o.a && b == o.b; }
};
struct PairKeyHash {
    size_t operator()(const PairKey& k) const {
        return (static_cast<size_t>(static_cast<uint32_t>(k.a)) << 32) | static_cast<uint32_t>(k.b);
    }
};

} // namespace mc
