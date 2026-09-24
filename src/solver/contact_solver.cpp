#include "contact_solver.h"

namespace mc {
namespace contact_solver {

void prestep(RigidBody* bodies, Manifold& m, float invDt, const SolverConfig& cfg) {
    RigidBody& A = bodies[m.a];
    RigidBody& B = bodies[m.b];
    basisFromNormal(m.normal, m.tangent[0], m.tangent[1]);

    for (int i = 0; i < m.count; ++i) {
        ContactPoint& p = m.points[i];
        p.rA = p.position - A.xf.position;
        p.rB = p.position - B.xf.position;

        // Effective mass along a direction d:
        //   1 / (mA^-1 + mB^-1 + d . ((IA^-1 (rA x d)) x rA + (IB^-1 (rB x d)) x rB))
        auto effectiveMass = [&](const Vec3& d) {
            Vec3 raxd = cross(p.rA, d), rbxd = cross(p.rB, d);
            float k = A.invMass + B.invMass +
                      dot(d, cross(A.invInertiaWorld * raxd, p.rA) +
                             cross(B.invInertiaWorld * rbxd, p.rB));
            return k > 0 ? 1.0f / k : 0.0f;
        };
        p.normalMass = effectiveMass(m.normal);
        p.tangentMass[0] = effectiveMass(m.tangent[0]);
        p.tangentMass[1] = effectiveMass(m.tangent[1]);

        // Baumgarte bias: push out a fraction of penetration per step; the slop
        // keeps a hair of overlap so contacts stay persistent (warm starting).
        float bias = cfg.baumgarte * invDt * std::max(p.penetration - cfg.linearSlop, 0.0f);

        // Restitution: bounce only above the threshold, using pre-solve velocity.
        float vn = dot(B.velocityAt(p.rB) - A.velocityAt(p.rA), m.normal);
        float bounce = vn < -cfg.restitutionThreshold ? -m.restitution * vn : 0.0f;
        p.velocityBias = std::max(bias, bounce);
    }
}

void warmStart(RigidBody* bodies, Manifold& m) {
    RigidBody& A = bodies[m.a];
    RigidBody& B = bodies[m.b];
    for (int i = 0; i < m.count; ++i) {
        ContactPoint& p = m.points[i];
        Vec3 impulse = m.normal * p.normalImpulse +
                       m.tangent[0] * p.tangentImpulse[0] +
                       m.tangent[1] * p.tangentImpulse[1];
        A.applyImpulse(-impulse, p.rA);
        B.applyImpulse(impulse, p.rB);
    }
}

void iterate(RigidBody* bodies, Manifold& m) {
    RigidBody& A = bodies[m.a];
    RigidBody& B = bodies[m.b];
    for (int i = 0; i < m.count; ++i) {
        ContactPoint& p = m.points[i];

        // Normal impulse, clamped so the total stays non-negative.
        Vec3 rv = B.velocityAt(p.rB) - A.velocityAt(p.rA);
        float vn = dot(rv, m.normal);
        float dPn = p.normalMass * (-vn + p.velocityBias);
        float newPn = std::max(p.normalImpulse + dPn, 0.0f);
        dPn = newPn - p.normalImpulse;
        p.normalImpulse = newPn;
        Vec3 Pn = m.normal * dPn;
        A.applyImpulse(-Pn, p.rA);
        B.applyImpulse(Pn, p.rB);

        // Friction: Coulomb cone approximated by a box clamp per tangent,
        // bounded by the accumulated normal impulse.
        float maxPt = m.friction * p.normalImpulse;
        for (int t = 0; t < 2; ++t) {
            rv = B.velocityAt(p.rB) - A.velocityAt(p.rA);
            float vt = dot(rv, m.tangent[t]);
            float dPt = p.tangentMass[t] * (-vt);
            float newPt = clampf(p.tangentImpulse[t] + dPt, -maxPt, maxPt);
            dPt = newPt - p.tangentImpulse[t];
            p.tangentImpulse[t] = newPt;
            Vec3 Pt = m.tangent[t] * dPt;
            A.applyImpulse(-Pt, p.rA);
            B.applyImpulse(Pt, p.rB);
        }
    }
}

} // namespace contact_solver
} // namespace mc
