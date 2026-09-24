#pragma once
#include "contact.h"

namespace mc {

// Narrowphase entry point: computes contact points between two bodies.
// Fills m.normal (world, pointing from a to b) and m.points[i].position /
// .penetration. Returns the number of contact points (0 => no contact).
// Does NOT touch impulses/anchors — persistence is layered on top by the world.
int collideShapes(const RigidBody& a, const RigidBody& b, Manifold& m);

// Closest points between segments [p1,q1] and [p2,q2] (Ericson, RTCD 5.1.9).
void closestPtSegmentSegment(const Vec3& p1, const Vec3& q1, const Vec3& p2, const Vec3& q2,
                             Vec3& c1, Vec3& c2);

} // namespace mc
