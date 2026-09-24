#pragma once
#include "../math/math.h"
#include "../collision/shapes.h"

namespace mc {

using BodyId = int32_t;
constexpr BodyId kInvalidBody = -1;

struct RigidBody {
    Transform xf;                 // position + orientation
    Vec3 linearVelocity;
    Vec3 angularVelocity;
    Vec3 force;                   // accumulated external force (cleared each step)
    Vec3 torque;

    float invMass = 1.0f;         // 0 => static/kinematic
    Mat3 invInertiaLocal;         // inverse inertia in body frame (principal axes)
    Mat3 invInertiaWorld;         // R * I_local^-1 * R^T, refreshed every step

    Shape shape;
    float friction = 0.6f;
    float restitution = 0.0f;

    // Sleeping: bodies below the velocity threshold for kSleepTime go inactive
    // and are skipped by integration/solve until an island neighbor wakes them.
    float lowVelocityTime = 0.0f;
    bool asleep = false;

    bool isStatic() const { return invMass == 0.0f; }

    void updateWorldInertia() {
        if (isStatic()) { invInertiaWorld = Mat3::zero(); return; }
        Mat3 R = xf.orientation.toMat3();
        invInertiaWorld = R * invInertiaLocal * R.transposed();
    }

    void applyImpulse(const Vec3& impulse, const Vec3& r) {
        linearVelocity += impulse * invMass;
        angularVelocity += invInertiaWorld * cross(r, impulse);
    }

    Vec3 velocityAt(const Vec3& r) const {
        return linearVelocity + cross(angularVelocity, r);
    }

    static RigidBody makeDynamic(const Shape& shape, const Vec3& pos, float mass) {
        RigidBody b;
        b.shape = shape;
        b.xf.position = pos;
        b.invMass = 1.0f / mass;
        b.invInertiaLocal = shape.invInertia(mass);
        b.updateWorldInertia();
        return b;
    }

    static RigidBody makeStatic(const Shape& shape, const Vec3& pos, const Quat& q = Quat::identity()) {
        RigidBody b;
        b.shape = shape;
        b.xf.position = pos;
        b.xf.orientation = q;
        b.invMass = 0.0f;
        b.invInertiaLocal = Mat3::zero();
        b.invInertiaWorld = Mat3::zero();
        return b;
    }
};

} // namespace mc
