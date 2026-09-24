#include "collide.h"
#include "gjk.h"
#include <algorithm>

namespace mc {

void closestPtSegmentSegment(const Vec3& p1, const Vec3& q1, const Vec3& p2, const Vec3& q2,
                             Vec3& c1, Vec3& c2) {
    Vec3 d1 = q1 - p1, d2 = q2 - p2, r = p1 - p2;
    float a = dot(d1, d1), e = dot(d2, d2), f = dot(d2, r);
    float s, t;
    if (a <= 1e-10f && e <= 1e-10f) { c1 = p1; c2 = p2; return; }
    if (a <= 1e-10f) {
        s = 0;
        t = clampf(f / e, 0, 1);
    } else {
        float c = dot(d1, r);
        if (e <= 1e-10f) {
            t = 0;
            s = clampf(-c / a, 0, 1);
        } else {
            float b = dot(d1, d2);
            float denom = a * e - b * b;
            s = denom > 1e-10f ? clampf((b * f - c * e) / denom, 0, 1) : 0.0f;
            t = (b * s + f) / e;
            if (t < 0) { t = 0; s = clampf(-c / a, 0, 1); }
            else if (t > 1) { t = 1; s = clampf((b - c) / a, 0, 1); }
        }
    }
    c1 = p1 + d1 * s;
    c2 = p2 + d2 * t;
}

namespace {

void addPoint(Manifold& m, const Vec3& pos, float pen) {
    if (m.count < 4) {
        m.points[m.count] = ContactPoint{};
        m.points[m.count].position = pos;
        m.points[m.count].penetration = pen;
        ++m.count;
    }
}

// --- sphere pairs ----------------------------------------------------------

// Contact between two spheres given centers/radii; normal from A to B.
int sphereSphere(const Vec3& cA, float rA, const Vec3& cB, float rB, Manifold& m) {
    Vec3 d = cB - cA;
    float distSq = lengthSq(d);
    float rSum = rA + rB;
    if (distSq > rSum * rSum) return 0;
    float dist = std::sqrt(distSq);
    m.normal = dist > 1e-9f ? d / dist : Vec3{0, 1, 0};
    addPoint(m, cA + m.normal * (rA - 0.5f * (rSum - dist)), rSum - dist);
    return m.count;
}

// Closest point on an oriented box to world point p; returns true if p is
// inside. On the inside path, outputs the smallest push-out face instead.
bool closestOnBox(const Transform& xf, const Vec3& e, const Vec3& p,
                  Vec3& onBox, Vec3& normalOut, float& insideDepth) {
    Vec3 local = xf.toLocal(p);
    Vec3 clamped = vmin(vmax(local, -e), e);
    if (lengthSq(local - clamped) > 1e-12f) {
        onBox = xf.toWorld(clamped);
        return false;
    }
    // Inside: exit through the nearest face.
    int axis = 0;
    float minDepth = e.x - std::fabs(local.x);
    float dy = e.y - std::fabs(local.y), dz = e.z - std::fabs(local.z);
    if (dy < minDepth) { minDepth = dy; axis = 1; }
    if (dz < minDepth) { minDepth = dz; axis = 2; }
    Vec3 nLocal{0, 0, 0};
    nLocal[axis] = local[axis] >= 0 ? 1.0f : -1.0f;
    Vec3 surf = local;
    surf[axis] = nLocal[axis] * e[axis];
    onBox = xf.toWorld(surf);
    normalOut = rotate(xf.orientation, nLocal);
    insideDepth = minDepth;
    return true;
}

int sphereBox(const RigidBody& sph, const RigidBody& box, Manifold& m, bool sphereIsA) {
    Vec3 c = sph.xf.position;
    float r = sph.shape.radius;
    Vec3 onBox, inNormal;
    float insideDepth;
    if (closestOnBox(box.xf, box.shape.halfExtents, c, onBox, inNormal, insideDepth)) {
        Vec3 nBoxToSphere = inNormal;
        m.normal = sphereIsA ? -nBoxToSphere : nBoxToSphere;
        addPoint(m, onBox, insideDepth + r);
        return m.count;
    }
    Vec3 d = c - onBox;
    float distSq = lengthSq(d);
    if (distSq > r * r) return 0;
    float dist = std::sqrt(distSq);
    Vec3 nBoxToSphere = dist > 1e-9f ? d / dist : Vec3{0, 1, 0};
    m.normal = sphereIsA ? -nBoxToSphere : nBoxToSphere;
    addPoint(m, onBox, r - dist);
    return m.count;
}

void capsuleSegment(const RigidBody& cap, Vec3& p0, Vec3& p1) {
    Vec3 axis = rotate(cap.xf.orientation, Vec3{0, cap.shape.halfHeight, 0});
    p0 = cap.xf.position - axis;
    p1 = cap.xf.position + axis;
}

// --- box vs box: SAT with reference-face clipping --------------------------

struct ClipPoly {
    Vec3 v[8];
    int count = 0;
};

// Sutherland-Hodgman clip of a polygon against half-space dot(n, x) <= offset.
ClipPoly clipAgainstPlane(const ClipPoly& in, const Vec3& n, float offset) {
    ClipPoly out;
    for (int i = 0; i < in.count; ++i) {
        const Vec3& cur = in.v[i];
        const Vec3& nxt = in.v[(i + 1) % in.count];
        float dc = dot(n, cur) - offset;
        float dn = dot(n, nxt) - offset;
        if (dc <= 0) out.v[out.count++] = cur;
        if (dc * dn < 0 && out.count < 8)
            out.v[out.count++] = cur + (nxt - cur) * (dc / (dc - dn));
    }
    return out;
}

// Supporting edge of an oriented box in world direction n, along local axis `axis`.
void supportEdge(const Transform& xf, const Vec3& e, const Mat3& R, int axis, const Vec3& n,
                 Vec3& e0, Vec3& e1) {
    Vec3 coords;
    for (int k = 0; k < 3; ++k) {
        if (k == axis) continue;
        coords[k] = dot(R.c[k], n) >= 0 ? e[k] : -e[k];
    }
    coords[axis] = e[axis];
    Vec3 a = coords;
    coords[axis] = -e[axis];
    Vec3 b = coords;
    e0 = xf.position + R.c[0] * a.x + R.c[1] * a.y + R.c[2] * a.z;
    e1 = xf.position + R.c[0] * b.x + R.c[1] * b.y + R.c[2] * b.z;
}

int boxBox(const RigidBody& A, const RigidBody& B, Manifold& m) {
    const Vec3 eA = A.shape.halfExtents, eB = B.shape.halfExtents;
    const Mat3 RA = A.xf.orientation.toMat3(), RB = B.xf.orientation.toMat3();
    const Vec3 d = B.xf.position - A.xf.position;

    auto projA = [&](const Vec3& L) {
        return eA.x * std::fabs(dot(RA.c[0], L)) + eA.y * std::fabs(dot(RA.c[1], L)) + eA.z * std::fabs(dot(RA.c[2], L));
    };
    auto projB = [&](const Vec3& L) {
        return eB.x * std::fabs(dot(RB.c[0], L)) + eB.y * std::fabs(dot(RB.c[1], L)) + eB.z * std::fabs(dot(RB.c[2], L));
    };

    float bestFaceSep = -1e30f;
    int bestFace = -1; // 0..2: A's axes, 3..5: B's axes
    Vec3 bestFaceN;    // oriented from A toward B
    for (int i = 0; i < 3; ++i) {
        Vec3 L = RA.c[i];
        float sep = std::fabs(dot(d, L)) - (eA[i] + projB(L));
        if (sep > 0) return 0;
        if (sep > bestFaceSep) { bestFaceSep = sep; bestFace = i; bestFaceN = dot(d, L) >= 0 ? L : -L; }
    }
    for (int j = 0; j < 3; ++j) {
        Vec3 L = RB.c[j];
        float sep = std::fabs(dot(d, L)) - (projA(L) + eB[j]);
        if (sep > 0) return 0;
        if (sep > bestFaceSep) { bestFaceSep = sep; bestFace = 3 + j; bestFaceN = dot(d, L) >= 0 ? L : -L; }
    }

    float bestEdgeSep = -1e30f;
    int bestI = -1, bestJ = -1;
    Vec3 bestEdgeN;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Vec3 L = cross(RA.c[i], RB.c[j]);
            float lenSq = lengthSq(L);
            if (lenSq < 1e-8f) continue; // near-parallel axes: covered by face tests
            L = L / std::sqrt(lenSq);
            float sep = std::fabs(dot(d, L)) - (projA(L) + projB(L));
            if (sep > 0) return 0;
            if (sep > bestEdgeSep) { bestEdgeSep = sep; bestI = i; bestJ = j; bestEdgeN = dot(d, L) >= 0 ? L : -L; }
        }
    }

    // Prefer face contacts: they give 4-point manifolds (stable stacking);
    // an edge axis must beat the best face by a margin to win.
    if (bestI >= 0 && bestEdgeSep > bestFaceSep + 1e-3f) {
        Vec3 n = bestEdgeN;
        Vec3 a0, a1, b0, b1, pA, pB;
        supportEdge(A.xf, eA, RA, bestI, n, a0, a1);
        supportEdge(B.xf, eB, RB, bestJ, -n, b0, b1);
        closestPtSegmentSegment(a0, a1, b0, b1, pA, pB);
        m.normal = n;
        addPoint(m, (pA + pB) * 0.5f, -bestEdgeSep);
        return m.count;
    }

    const bool refIsA = bestFace < 3;
    const int axis = bestFace % 3;
    const RigidBody& R = refIsA ? A : B;
    const RigidBody& I = refIsA ? B : A;
    const Mat3& Rr = refIsA ? RA : RB;
    const Mat3& Ri = refIsA ? RB : RA;
    const Vec3 eR = refIsA ? eA : eB;
    const Vec3 eI = refIsA ? eB : eA;
    // Outward normal of the reference face (toward the incident box).
    const Vec3 refN = refIsA ? bestFaceN : -bestFaceN;

    // Incident face: the face of I most anti-parallel to refN.
    int incAxis = 0;
    float incSign = 1, minDot = 1e30f;
    for (int k = 0; k < 3; ++k) {
        float dk = dot(Ri.c[k], refN);
        if (dk < minDot) { minDot = dk; incAxis = k; incSign = 1; }
        if (-dk < minDot) { minDot = -dk; incAxis = k; incSign = -1; }
    }

    int u = (incAxis + 1) % 3, v = (incAxis + 2) % 3;
    Vec3 faceCenter = I.xf.position + Ri.c[incAxis] * (incSign * eI[incAxis]);
    const float su[4] = {1, -1, -1, 1}, sv[4] = {1, 1, -1, -1}; // perimeter order
    ClipPoly poly;
    for (int k = 0; k < 4; ++k)
        poly.v[poly.count++] = faceCenter + Ri.c[u] * (su[k] * eI[u]) + Ri.c[v] * (sv[k] * eI[v]);

    // Clip against the 4 side planes of the reference face.
    for (int k = 0; k < 3; ++k) {
        if (k == axis) continue;
        Vec3 sn = Rr.c[k];
        float center = dot(sn, R.xf.position);
        poly = clipAgainstPlane(poly, sn, center + eR[k]);
        poly = clipAgainstPlane(poly, -sn, -center + eR[k]);
        if (poly.count == 0) return 0;
    }

    float faceOffset = dot(refN, R.xf.position) + eR[axis];
    struct Cand { Vec3 p; float pen; };
    Cand cands[8];
    int nc = 0;
    for (int k = 0; k < poly.count; ++k) {
        float sep = dot(refN, poly.v[k]) - faceOffset;
        if (sep < 0) cands[nc++] = {poly.v[k], -sep};
    }
    if (nc == 0) return 0;
    if (nc > 4)
        std::partial_sort(cands, cands + 4, cands + nc,
                          [](const Cand& x, const Cand& y) { return x.pen > y.pen; });

    m.normal = bestFaceN; // always A -> B
    for (int k = 0; k < std::min(nc, 4); ++k) addPoint(m, cands[k].p, cands[k].pen);
    return m.count;
}

// --- capsule vs box (GJK on the capsule's inner segment, EPA when deep) ----

int boxCapsule(const RigidBody& box, const RigidBody& cap, Manifold& m) {
    SupportShape sBox{&box.shape, box.xf, false};
    SupportShape sSeg{&cap.shape, cap.xf, true};
    float r = cap.shape.radius;

    GjkResult g = gjkDistance(sBox, sSeg);
    Vec3 nBoxToCap; // normal from box toward capsule
    if (!g.intersecting) {
        if (g.distance >= r) return 0;
        Vec3 dv = g.pointB - g.pointA;
        nBoxToCap = g.distance > 1e-6f ? dv / g.distance : Vec3{0, 1, 0};
        m.normal = nBoxToCap;
        addPoint(m, g.pointA, r - g.distance);
    } else {
        EpaResult e = epaPenetration(sBox, sSeg);
        if (e.valid) {
            nBoxToCap = e.normal;
            m.normal = nBoxToCap;
            addPoint(m, e.contactOnB - nBoxToCap * r, e.depth + r);
        } else {
            // Degenerate EPA (should be rare): push out along the nearest box face.
            Vec3 onBox, n;
            float depth;
            closestOnBox(box.xf, box.shape.halfExtents, cap.xf.position, onBox, n, depth);
            nBoxToCap = n;
            m.normal = nBoxToCap;
            addPoint(m, onBox, depth + r);
        }
    }

    // A capsule lying across a face needs two contact points or it rocks:
    // when the capsule axis is nearly perpendicular to the normal, also test
    // both end-spheres against the box.
    Vec3 axisDir = rotate(cap.xf.orientation, Vec3{0, 1, 0});
    if (std::fabs(dot(axisDir, nBoxToCap)) < 0.3f && cap.shape.halfHeight > 1e-4f) {
        ContactPoint single = m.points[0];
        Vec3 p0, p1;
        capsuleSegment(cap, p0, p1);
        m.count = 0; // rebuild with the two end-sphere contacts
        for (const Vec3& p : {p0, p1}) {
            Vec3 onBox, inN;
            float insideDepth;
            if (closestOnBox(box.xf, box.shape.halfExtents, p, onBox, inN, insideDepth)) {
                addPoint(m, onBox, insideDepth + r);
            } else {
                Vec3 dv = p - onBox;
                float dist = length(dv);
                if (dist < r) addPoint(m, onBox, r - dist);
            }
        }
        if (m.count == 0) { // ends clear of the box; keep the single GJK point
            m.points[0] = single;
            m.count = 1;
        }
    }
    return m.count;
}

} // namespace

int collideShapes(const RigidBody& a, const RigidBody& b, Manifold& m) {
    m.count = 0;

    // Canonical ordering: dispatch with the lower ShapeType first, flip after.
    const bool swapped = static_cast<int>(a.shape.type) > static_cast<int>(b.shape.type);
    const RigidBody& A = swapped ? b : a;
    const RigidBody& B = swapped ? a : b;

    switch (A.shape.type) {
    case ShapeType::Sphere:
        switch (B.shape.type) {
        case ShapeType::Sphere:
            sphereSphere(A.xf.position, A.shape.radius, B.xf.position, B.shape.radius, m);
            break;
        case ShapeType::Box:
            sphereBox(A, B, m, /*sphereIsA=*/true);
            break;
        case ShapeType::Capsule: {
            Vec3 p0, p1;
            capsuleSegment(B, p0, p1);
            Vec3 c1, c2;
            closestPtSegmentSegment(A.xf.position, A.xf.position, p0, p1, c1, c2);
            sphereSphere(A.xf.position, A.shape.radius, c2, B.shape.radius, m);
            break;
        }
        }
        break;
    case ShapeType::Box:
        switch (B.shape.type) {
        case ShapeType::Box:
            boxBox(A, B, m);
            break;
        case ShapeType::Capsule:
            boxCapsule(A, B, m);
            break;
        default: break;
        }
        break;
    case ShapeType::Capsule: {
        // capsule-capsule (only remaining pair in canonical order)
        Vec3 a0, a1, b0, b1, cA, cB;
        capsuleSegment(A, a0, a1);
        capsuleSegment(B, b0, b1);
        closestPtSegmentSegment(a0, a1, b0, b1, cA, cB);
        sphereSphere(cA, A.shape.radius, cB, B.shape.radius, m);
        break;
    }
    }

    if (m.count == 0) return 0;
    if (swapped) m.normal = -m.normal;
    return m.count;
}

} // namespace mc
