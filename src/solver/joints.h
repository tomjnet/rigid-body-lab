#pragma once
// Joints in the same Jacobian-row framework as contacts.
//
// BallSocket: 3 linear rows solved as a block with a 3x3 effective-mass matrix
//   K = (1/mA + 1/mB) I + skew(rA) IA^-1 skew(rA)^T + skew(rB) IB^-1 skew(rB)^T
// Hinge: ball-socket + 2 angular rows keeping the body axes aligned, solved
//   with a 2x2 effective mass over a basis (b1, b2) perpendicular to the axis.

#include "../core/rigid_body.h"

namespace mc {

enum class JointType : uint8_t { BallSocket, Hinge };

struct Joint {
    JointType type = JointType::BallSocket;
    BodyId a = kInvalidBody, b = kInvalidBody;
    Vec3 localAnchorA, localAnchorB;
    Vec3 localAxisA{0, 0, 1}, localAxisB{0, 0, 1}; // hinge only

    // Accumulated impulses (warm starting)
    Vec3 linearImpulse;
    float angularImpulse[2] = {0, 0};

    // Per-step scratch
    Vec3 rA, rB;
    Mat3 invK;          // linear block
    Vec3 linearBias;
    Vec3 b1, b2;        // hinge angular basis (world)
    float angMass[3] = {0, 0, 0}; // 2x2 symmetric: k11, k12, k22 -> inverted on demand
    float angBias[2] = {0, 0};

    static Joint ballSocket(BodyId a, BodyId b, const Vec3& worldAnchor,
                            const RigidBody& bodyA, const RigidBody& bodyB) {
        Joint j;
        j.type = JointType::BallSocket;
        j.a = a; j.b = b;
        j.localAnchorA = bodyA.xf.toLocal(worldAnchor);
        j.localAnchorB = bodyB.xf.toLocal(worldAnchor);
        return j;
    }

    static Joint hinge(BodyId a, BodyId b, const Vec3& worldAnchor, const Vec3& worldAxis,
                       const RigidBody& bodyA, const RigidBody& bodyB) {
        Joint j = ballSocket(a, b, worldAnchor, bodyA, bodyB);
        j.type = JointType::Hinge;
        j.localAxisA = rotateInv(bodyA.xf.orientation, normalize(worldAxis));
        j.localAxisB = rotateInv(bodyB.xf.orientation, normalize(worldAxis));
        return j;
    }
};

namespace joint_solver {

void prestep(RigidBody* bodies, Joint& j, float invDt, float baumgarte);
void warmStart(RigidBody* bodies, Joint& j);
void iterate(RigidBody* bodies, Joint& j);

} // namespace joint_solver

} // namespace mc
