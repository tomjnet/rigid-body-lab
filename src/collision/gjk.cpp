#include "gjk.h"
#include <vector>
#include <cstring>

namespace mc {

namespace {

struct SV { // simplex vertex on the configuration-space obstacle (Minkowski difference)
    Vec3 p;      // sa - sb
    Vec3 sa, sb; // witness supports on A and B
};

SV supportCSO(const SupportShape& a, const SupportShape& b, const Vec3& d) {
    SV v;
    v.sa = a.support(d);
    v.sb = b.support(-d);
    v.p = v.sa - v.sb;
    return v;
}

// Closest point to the origin on triangle (a,b,c), Ericson RTCD 5.1.5 adapted
// for P = origin. Returns barycentric coords and which vertices remain active.
struct TriResult {
    Vec3 closest;
    float u = 0, v = 0, w = 0; // weights for a, b, c
    int mask = 0;              // bit i set => vertex i is part of the closest feature
};

TriResult closestOnTriangle(const Vec3& a, const Vec3& b, const Vec3& c) {
    TriResult r;
    Vec3 ab = b - a, ac = c - a, ap = -a;
    float d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0 && d2 <= 0) { r.closest = a; r.u = 1; r.mask = 1; return r; }

    Vec3 bp = -b;
    float d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0 && d4 <= d3) { r.closest = b; r.v = 1; r.mask = 2; return r; }

    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        float t = d1 / (d1 - d3);
        r.closest = a + ab * t; r.u = 1 - t; r.v = t; r.mask = 1 | 2;
        return r;
    }

    Vec3 cp = -c;
    float d5 = dot(ab, cp), d6 = dot(ac, cp);
    if (d6 >= 0 && d5 <= d6) { r.closest = c; r.w = 1; r.mask = 4; return r; }

    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
        float t = d2 / (d2 - d6);
        r.closest = a + ac * t; r.u = 1 - t; r.w = t; r.mask = 1 | 4;
        return r;
    }

    float va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
        float t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        r.closest = b + (c - b) * t; r.v = 1 - t; r.w = t; r.mask = 2 | 4;
        return r;
    }

    float denom = 1.0f / (va + vb + vc);
    float v = vb * denom, w = vc * denom;
    r.closest = a + ab * v + ac * w;
    r.u = 1 - v - w; r.v = v; r.w = w; r.mask = 1 | 2 | 4;
    return r;
}

struct Simplex {
    SV v[4];
    float lambda[4] = {0};
    int count = 0;

    void keep(int mask, const float* weights) {
        SV kept[4]; float lam[4]; int n = 0;
        for (int i = 0; i < count; ++i) {
            if (mask & (1 << i)) { kept[n] = v[i]; lam[n] = weights[i]; ++n; }
        }
        std::memcpy(v, kept, sizeof(SV) * n);
        std::memcpy(lambda, lam, sizeof(float) * n);
        count = n;
    }
};

// Reduce the simplex to the feature closest to the origin; returns that closest
// point. Sets `containsOrigin` when a tetrahedron encloses the origin.
Vec3 closestOnSimplex(Simplex& s, bool& containsOrigin) {
    containsOrigin = false;
    switch (s.count) {
    case 1: {
        s.lambda[0] = 1;
        return s.v[0].p;
    }
    case 2: {
        Vec3 a = s.v[0].p, b = s.v[1].p, ab = b - a;
        float t = lengthSq(ab) > 1e-12f ? clampf(-dot(a, ab) / lengthSq(ab), 0.0f, 1.0f) : 0.0f;
        if (t <= 0) { float w[2] = {1, 0}; s.keep(1, w); return a; }
        if (t >= 1) { float w[2] = {0, 1}; s.keep(2, w); return b; }
        float w[2] = {1 - t, t};
        s.lambda[0] = w[0]; s.lambda[1] = w[1];
        return a + ab * t;
    }
    case 3: {
        TriResult r = closestOnTriangle(s.v[0].p, s.v[1].p, s.v[2].p);
        float w[3] = {r.u, r.v, r.w};
        s.keep(r.mask, w);
        return r.closest;
    }
    case 4: {
        // Test the origin against the four faces; recurse to the closest face
        // the origin lies outside of.
        const int faces[4][3] = {{0, 1, 2}, {0, 2, 3}, {0, 3, 1}, {1, 3, 2}};
        float bestDist = 1e30f;
        TriResult bestTri;
        int bestFace[3] = {0, 1, 2};
        bool inside = true;
        for (auto& f : faces) {
            Vec3 a = s.v[f[0]].p, b = s.v[f[1]].p, c = s.v[f[2]].p;
            Vec3 n = cross(b - a, c - a);
            // vertex not on this face determines the inside direction
            int other = 6 - f[0] - f[1] - f[2];
            float side = dot(n, s.v[other].p - a);
            float origSide = dot(n, -a);
            if (side * origSide < 0) { // origin outside this face
                inside = false;
                TriResult r = closestOnTriangle(a, b, c);
                float d = lengthSq(r.closest);
                if (d < bestDist) {
                    bestDist = d; bestTri = r;
                    bestFace[0] = f[0]; bestFace[1] = f[1]; bestFace[2] = f[2];
                }
            }
        }
        if (inside) { containsOrigin = true; return {0, 0, 0}; }
        // Rebuild simplex from the winning face's active vertices.
        Simplex ns;
        float triW[3] = {bestTri.u, bestTri.v, bestTri.w};
        for (int i = 0; i < 3; ++i) {
            if (bestTri.mask & (1 << i)) {
                ns.v[ns.count] = s.v[bestFace[i]];
                ns.lambda[ns.count] = triW[i];
                ++ns.count;
            }
        }
        s = ns;
        return bestTri.closest;
    }
    }
    return {0, 0, 0};
}

} // namespace

GjkResult gjkDistance(const SupportShape& a, const SupportShape& b) {
    GjkResult res;
    Vec3 dir = a.xf.position - b.xf.position;
    if (lengthSq(dir) < 1e-12f) dir = {1, 0, 0};

    Simplex s;
    s.v[0] = supportCSO(a, b, dir);
    s.count = 1;

    Vec3 closest = s.v[0].p;
    for (int iter = 0; iter < 64; ++iter) {
        bool containsOrigin = false;
        closest = closestOnSimplex(s, containsOrigin);
        float distSq = lengthSq(closest);
        if (containsOrigin || distSq < 1e-10f) {
            res.intersecting = true;
            return res;
        }
        SV w = supportCSO(a, b, -closest);
        // No progress toward the origin => converged to the true distance.
        if (distSq - dot(closest, w.p) <= 1e-6f * distSq) break;
        bool duplicate = false;
        for (int i = 0; i < s.count; ++i)
            if (lengthSq(s.v[i].p - w.p) < 1e-12f) { duplicate = true; break; }
        if (duplicate || s.count == 4) break;
        s.v[s.count++] = w;
    }

    res.distance = length(closest);
    res.pointA = {0, 0, 0};
    res.pointB = {0, 0, 0};
    for (int i = 0; i < s.count; ++i) {
        res.pointA += s.v[i].sa * s.lambda[i];
        res.pointB += s.v[i].sb * s.lambda[i];
    }
    return res;
}

// ---------------------------------------------------------------------------
// EPA: expand a polytope around the origin inside the CSO until the closest
// boundary face is found; its normal/distance are the penetration direction
// and depth.
// ---------------------------------------------------------------------------

namespace {

struct EpaFace {
    int i0, i1, i2;
    Vec3 n;      // unit outward normal
    float dist;  // distance from origin along n
    bool alive = true;
};

bool makeFace(const std::vector<SV>& verts, int i0, int i1, int i2, EpaFace& f) {
    Vec3 a = verts[i0].p, b = verts[i1].p, c = verts[i2].p;
    Vec3 n = cross(b - a, c - a);
    float len = length(n);
    if (len < 1e-10f) return false;
    n = n / len;
    float d = dot(n, a);
    if (d < 0) { // flip so normal points away from origin
        std::swap(i1, i2);
        n = -n; d = -d;
    }
    f = {i0, i1, i2, n, d, true};
    return true;
}

} // namespace

EpaResult epaPenetration(const SupportShape& a, const SupportShape& b) {
    EpaResult res;

    // Build an initial tetrahedron around the origin from axis supports.
    std::vector<SV> verts;
    const Vec3 seeds[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    for (const Vec3& d : seeds) {
        SV v = supportCSO(a, b, d);
        bool dup = false;
        for (auto& e : verts)
            if (lengthSq(e.p - v.p) < 1e-10f) { dup = true; break; }
        if (!dup) verts.push_back(v);
        if (verts.size() == 4) {
            Vec3 ab = verts[1].p - verts[0].p, ac = verts[2].p - verts[0].p, ad = verts[3].p - verts[0].p;
            if (std::fabs(dot(ab, cross(ac, ad))) < 1e-9f) verts.pop_back(); // degenerate, try next seed
        }
    }
    if (verts.size() < 4) return res;

    std::vector<EpaFace> faces;
    const int tetra[4][3] = {{0, 1, 2}, {0, 2, 3}, {0, 3, 1}, {1, 3, 2}};
    for (auto& t : tetra) {
        EpaFace f;
        if (makeFace(verts, t[0], t[1], t[2], f)) faces.push_back(f);
    }
    if (faces.size() < 4) return res;

    for (int iter = 0; iter < 48; ++iter) {
        // closest live face
        int best = -1;
        float bestDist = 1e30f;
        for (size_t i = 0; i < faces.size(); ++i)
            if (faces[i].alive && faces[i].dist < bestDist) { bestDist = faces[i].dist; best = (int)i; }
        if (best < 0) return res;

        const EpaFace f = faces[best];
        SV w = supportCSO(a, b, f.n);
        float growth = dot(f.n, w.p) - f.dist;
        if (growth < 1e-4f || (int)verts.size() > 64) {
            // Converged: project origin onto the face for witness barycentrics.
            res.valid = true;
            res.normal = f.n;
            res.depth = f.dist;
            TriResult tr = closestOnTriangle(verts[f.i0].p, verts[f.i1].p, verts[f.i2].p);
            res.contactOnB = verts[f.i0].sb * tr.u + verts[f.i1].sb * tr.v + verts[f.i2].sb * tr.w;
            return res;
        }

        // Remove faces visible from w, collect horizon edges.
        int wi = (int)verts.size();
        verts.push_back(w);
        std::vector<std::pair<int, int>> horizon;
        for (auto& face : faces) {
            if (!face.alive) continue;
            if (dot(face.n, w.p - verts[face.i0].p) > 0) {
                face.alive = false;
                const int e[3][2] = {{face.i0, face.i1}, {face.i1, face.i2}, {face.i2, face.i0}};
                for (auto& ed : e) {
                    // an edge shared with another removed face cancels out
                    bool cancelled = false;
                    for (size_t h = 0; h < horizon.size(); ++h) {
                        if (horizon[h].first == ed[1] && horizon[h].second == ed[0]) {
                            horizon.erase(horizon.begin() + h);
                            cancelled = true;
                            break;
                        }
                    }
                    if (!cancelled) horizon.emplace_back(ed[0], ed[1]);
                }
            }
        }
        for (auto& ed : horizon) {
            EpaFace nf;
            if (makeFace(verts, ed.first, ed.second, wi, nf)) faces.push_back(nf);
        }
    }
    return res;
}

} // namespace mc
