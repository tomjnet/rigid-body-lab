#pragma once
#include "../math/math.h"

namespace mc {

enum class ShapeType : uint8_t { Sphere, Box, Capsule };

// One POD shape struct instead of a class hierarchy: bodies are iterated in
// hot loops and virtual dispatch per shape is a needless indirection.
struct Shape {
    ShapeType type = ShapeType::Sphere;
    Vec3 halfExtents{0.5f, 0.5f, 0.5f}; // Box
    float radius = 0.5f;                // Sphere / Capsule
    float halfHeight = 0.5f;            // Capsule: half the segment length (excludes caps)

    static Shape sphere(float r) {
        Shape s; s.type = ShapeType::Sphere; s.radius = r; return s;
    }
    static Shape box(const Vec3& halfExtents) {
        Shape s; s.type = ShapeType::Box; s.halfExtents = halfExtents; return s;
    }
    static Shape capsule(float r, float halfHeight) {
        Shape s; s.type = ShapeType::Capsule; s.radius = r; s.halfHeight = halfHeight; return s;
    }

    // Support point in LOCAL space for direction d (GJK support mapping).
    Vec3 localSupport(const Vec3& d) const {
        switch (type) {
        case ShapeType::Sphere:
            return normalize(d) * radius;
        case ShapeType::Box:
            return {d.x >= 0 ? halfExtents.x : -halfExtents.x,
                    d.y >= 0 ? halfExtents.y : -halfExtents.y,
                    d.z >= 0 ? halfExtents.z : -halfExtents.z};
        case ShapeType::Capsule: {
            // segment along local Y plus sphere radius
            Vec3 p{0, d.y >= 0 ? halfHeight : -halfHeight, 0};
            return p + normalize(d) * radius;
        }
        }
        return {};
    }

    AABB computeAABB(const Transform& xf) const {
        switch (type) {
        case ShapeType::Sphere: {
            Vec3 r{radius, radius, radius};
            return {xf.position - r, xf.position + r};
        }
        case ShapeType::Box: {
            // |R| * halfExtents gives the world-space extent of an oriented box
            Mat3 R = xf.orientation.toMat3();
            Vec3 ext{
                std::fabs(R.c[0].x) * halfExtents.x + std::fabs(R.c[1].x) * halfExtents.y + std::fabs(R.c[2].x) * halfExtents.z,
                std::fabs(R.c[0].y) * halfExtents.x + std::fabs(R.c[1].y) * halfExtents.y + std::fabs(R.c[2].y) * halfExtents.z,
                std::fabs(R.c[0].z) * halfExtents.x + std::fabs(R.c[1].z) * halfExtents.y + std::fabs(R.c[2].z) * halfExtents.z};
            return {xf.position - ext, xf.position + ext};
        }
        case ShapeType::Capsule: {
            Vec3 axis = rotate(xf.orientation, Vec3{0, halfHeight, 0});
            Vec3 r{radius, radius, radius};
            Vec3 p0 = xf.position - axis, p1 = xf.position + axis;
            return {vmin(p0, p1) - r, vmax(p0, p1) + r};
        }
        }
        return {};
    }

    // Inverse inertia tensor in local (principal) frame for the given mass.
    Mat3 invInertia(float mass) const {
        if (mass <= 0.0f) return Mat3::zero();
        switch (type) {
        case ShapeType::Sphere: {
            float i = 0.4f * mass * radius * radius;
            return Mat3::diagonal(1.0f / i, 1.0f / i, 1.0f / i);
        }
        case ShapeType::Box: {
            const Vec3& h = halfExtents;
            float ix = mass / 3.0f * (h.y * h.y + h.z * h.z);
            float iy = mass / 3.0f * (h.x * h.x + h.z * h.z);
            float iz = mass / 3.0f * (h.x * h.x + h.y * h.y);
            return Mat3::diagonal(1.0f / ix, 1.0f / iy, 1.0f / iz);
        }
        case ShapeType::Capsule: {
            // cylinder + two hemispheres, mass split by volume
            float r = radius, h = 2.0f * halfHeight;
            float volCyl = kPi * r * r * h;
            float volCaps = (4.0f / 3.0f) * kPi * r * r * r;
            float mCyl = mass * volCyl / (volCyl + volCaps);
            float mCaps = mass - mCyl;
            float iAxis = 0.5f * mCyl * r * r + 0.4f * mCaps * r * r;
            float iPerp = mCyl * (h * h / 12.0f + r * r / 4.0f) +
                          mCaps * (0.4f * r * r + halfHeight * halfHeight + 0.375f * h * r);
            return Mat3::diagonal(1.0f / iPerp, 1.0f / iAxis, 1.0f / iPerp);
        }
        }
        return Mat3::zero();
    }
};

} // namespace mc
