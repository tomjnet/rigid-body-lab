#include "joints.h"

namespace mc {
namespace joint_solver {

void prestep(RigidBody* bodies, Joint& j, float invDt, float baumgarte) {
    RigidBody& A = bodies[j.a];
    RigidBody& B = bodies[j.b];

    j.rA = rotate(A.xf.orientation, j.localAnchorA);
    j.rB = rotate(B.xf.orientation, j.localAnchorB);

    Mat3 K = Mat3::diagonal(A.invMass + B.invMass, A.invMass + B.invMass, A.invMass + B.invMass);
    Mat3 Sa = Mat3::skew(j.rA), Sb = Mat3::skew(j.rB);
    K = K + Sa * A.invInertiaWorld * Sa.transposed() + Sb * B.invInertiaWorld * Sb.transposed();
    j.invK = K.inverse();

    Vec3 C = (B.xf.position + j.rB) - (A.xf.position + j.rA);
    j.linearBias = C * (baumgarte * invDt);

    if (j.type == JointType::Hinge) {
        Vec3 axisA = rotate(A.xf.orientation, j.localAxisA);
        Vec3 axisB = rotate(B.xf.orientation, j.localAxisB);
        basisFromNormal(axisA, j.b1, j.b2);

        Mat3 Isum = A.invInertiaWorld + B.invInertiaWorld;
        j.angMass[0] = dot(j.b1, Isum * j.b1);
        j.angMass[1] = dot(j.b1, Isum * j.b2);
        j.angMass[2] = dot(j.b2, Isum * j.b2);

        // C_i = b_i . axisB, zero when the axes stay aligned.
        j.angBias[0] = baumgarte * invDt * dot(j.b1, axisB);
        j.angBias[1] = baumgarte * invDt * dot(j.b2, axisB);
    }
}

void warmStart(RigidBody* bodies, Joint& j) {
    RigidBody& A = bodies[j.a];
    RigidBody& B = bodies[j.b];
    A.applyImpulse(-j.linearImpulse, j.rA);
    B.applyImpulse(j.linearImpulse, j.rB);
    if (j.type == JointType::Hinge) {
        Vec3 L = j.b1 * j.angularImpulse[0] + j.b2 * j.angularImpulse[1];
        A.angularVelocity -= A.invInertiaWorld * L;
        B.angularVelocity += B.invInertiaWorld * L;
    }
}

void iterate(RigidBody* bodies, Joint& j) {
    RigidBody& A = bodies[j.a];
    RigidBody& B = bodies[j.b];

    if (j.type == JointType::Hinge) {
        // 2 angular rows: drive the perpendicular components of the relative
        // angular velocity (plus alignment bias) to zero.
        Vec3 wRel = B.angularVelocity - A.angularVelocity;
        float c1 = dot(j.b1, wRel) + j.angBias[0];
        float c2 = dot(j.b2, wRel) + j.angBias[1];
        float k11 = j.angMass[0], k12 = j.angMass[1], k22 = j.angMass[2];
        float det = k11 * k22 - k12 * k12;
        if (std::fabs(det) > 1e-12f) {
            float inv = 1.0f / det;
            float l1 = -( k22 * c1 - k12 * c2) * inv;
            float l2 = -(-k12 * c1 + k11 * c2) * inv;
            j.angularImpulse[0] += l1;
            j.angularImpulse[1] += l2;
            Vec3 L = j.b1 * l1 + j.b2 * l2;
            A.angularVelocity -= A.invInertiaWorld * L;
            B.angularVelocity += B.invInertiaWorld * L;
        }
    }

    // 3 linear rows as one block: point velocities at the anchors must match.
    Vec3 vRel = B.velocityAt(j.rB) - A.velocityAt(j.rA);
    Vec3 impulse = j.invK * (-(vRel + j.linearBias));
    j.linearImpulse += impulse;
    A.applyImpulse(-impulse, j.rA);
    B.applyImpulse(impulse, j.rB);
}

} // namespace joint_solver
} // namespace mc
