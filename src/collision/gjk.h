#pragma once
// GJK distance query between convex shapes, with witness points, plus EPA for
// penetration depth when the shapes overlap. Operates on support mappings so
// it works for any convex shape (here: box, sphere, capsule, and the capsule's
// inner segment used by the capsule-vs-box contact path).

#include "../math/math.h"
#include "shapes.h"

namespace mc {

// A support-mapped convex object in world space. `margin` lets callers run GJK
// on a shrunk core shape (e.g. a capsule's inner segment) and add the radius back.
struct SupportShape {
    const Shape* shape = nullptr;
    Transform xf;
    bool coreOnly = false; // capsule: use inner segment (ignore radius)

    Vec3 support(const Vec3& dirWorld) const {
        Vec3 dLocal = rotateInv(xf.orientation, dirWorld);
        Vec3 p;
        if (coreOnly && shape->type == ShapeType::Capsule)
            p = Vec3{0, dLocal.y >= 0 ? shape->halfHeight : -shape->halfHeight, 0};
        else
            p = shape->localSupport(dLocal);
        return xf.toWorld(p);
    }
};

struct GjkResult {
    bool intersecting = false;
    float distance = 0.0f; // valid when not intersecting
    Vec3 pointA, pointB;   // closest points on A and B (valid when not intersecting)
};

GjkResult gjkDistance(const SupportShape& a, const SupportShape& b);

struct EpaResult {
    bool valid = false;
    Vec3 normal;       // points from A to B (push B along +normal to separate)
    float depth = 0.0f;
    Vec3 contactOnB;   // deepest point estimate on B's surface
};

// Requires the shapes to actually overlap (call after gjkDistance says so).
EpaResult epaPenetration(const SupportShape& a, const SupportShape& b);

} // namespace mc
