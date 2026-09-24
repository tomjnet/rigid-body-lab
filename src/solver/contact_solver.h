#pragma once
// Sequential-impulse (projected Gauss-Seidel) contact solver, Box2D-style:
//  - accumulated impulses with clamping (Pn >= 0, |Pt| <= mu*Pn)
//  - warm starting from the previous frame's impulses
//  - Baumgarte positional bias with slop, restitution above a speed threshold
//
// See Catto, "Iterative Dynamics with Temporal Coherence" (2005).
//
// The API is stepwise (prestep / warmStart / iterate) so the island solver can
// interleave contact and joint iterations in one Gauss-Seidel sweep.

#include "../collision/contact.h"

namespace mc {

struct SolverConfig {
    int velocityIterations = 10;
    float baumgarte = 0.2f;              // fraction of penetration corrected per step
    float linearSlop = 0.005f;           // allowed penetration (m), prevents jitter
    float restitutionThreshold = 1.0f;   // approach speed (m/s) below which e = 0
};

namespace contact_solver {

void prestep(RigidBody* bodies, Manifold& m, float invDt, const SolverConfig& cfg);
void warmStart(RigidBody* bodies, Manifold& m);
void iterate(RigidBody* bodies, Manifold& m); // one Gauss-Seidel sweep over the manifold

} // namespace contact_solver

} // namespace mc
