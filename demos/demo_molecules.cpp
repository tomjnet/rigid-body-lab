// Molecular sandbox: atoms are spheres, covalent bonds are ball-socket joints
// created at runtime from the engine's own contact manifolds. Atoms carry a
// valence (H:1 O:2 N:3 C:4); when two atoms with free valence touch they bond
// at the contact point, and union-find over the bond graph names the molecules
// that emerge (H2O, CO2, CH4, ...). Zero gravity + bouncy walls keep the
// "gas" mixing; SPACE injects thermal energy to drive more collisions.
#include "viewer.h"
#include <algorithm>
#include <array>
#include <map>
#include <random>
#include <string>
#include <thread>
#include <vector>

using namespace mc;

namespace {

struct Element {
    const char* symbol;
    float radius;
    float mass; // scaled, not amu: keeps PGS mass ratios tame (see README)
    int valence;
    Color color; // CPK-ish
};
const Element kElements[] = {
    {"H", 0.22f, 1.0f, 1, Color{235, 235, 235, 255}},
    {"O", 0.33f, 4.0f, 2, Color{225, 65, 55, 255}},
    {"N", 0.32f, 3.7f, 3, Color{70, 100, 225, 255}},
    {"C", 0.35f, 3.5f, 4, Color{80, 80, 85, 255}},
};
constexpr int kNumElements = 4;
const int kDisplayOrder[kNumElements] = {3, 0, 2, 1}; // Hill-ish: C, H, N, O

struct Atom {
    BodyId body;
    int elem;
    int freeValence;
};

struct Bond {
    int a, b; // atom indices
};

// Reaction chamber: inner half extents and center.
const Vec3 kChamber{6.0f, 4.5f, 6.0f};
const Vec3 kCenter{0, 4.5f, 0};

int ufFind(std::vector<int>& parent, int i) {
    while (parent[i] != i) { parent[i] = parent[parent[i]]; i = parent[i]; }
    return i;
}

std::string formulaFor(const int counts[kNumElements]) {
    std::string f;
    for (int e : kDisplayOrder)
        if (counts[e]) {
            f += kElements[e].symbol;
            if (counts[e] > 1) f += std::to_string(counts[e]);
        }
    return f;
}

const char* commonName(const std::string& f) {
    if (f == "H2O") return "water";
    if (f == "CO2") return "carbon dioxide";
    if (f == "CH4") return "methane";
    if (f == "NH3") return "ammonia";
    if (f == "CO") return "carbon monoxide";
    if (f == "H2") return "hydrogen";
    if (f == "O2") return "oxygen";
    if (f == "N2") return "nitrogen";
    return nullptr;
}

} // namespace

int main() {
    World w(std::thread::hardware_concurrency());
    w.gravity = {0, 0, 0};   // a gas, not a pile
    w.linearDamping = 0.0f;  // conserve the thermal motion

    // Chamber walls: 6 static slabs just outside the inner volume.
    const float t = 0.5f;
    auto wall = [&](Vec3 half, Vec3 pos) {
        RigidBody b = RigidBody::makeStatic(Shape::box(half), pos);
        b.restitution = 0.85f;
        b.friction = 0.05f;
        w.addBody(b);
    };
    wall({kChamber.x + t, t, kChamber.z + t}, {kCenter.x, kCenter.y - kChamber.y - t, kCenter.z});
    wall({kChamber.x + t, t, kChamber.z + t}, {kCenter.x, kCenter.y + kChamber.y + t, kCenter.z});
    wall({t, kChamber.y, kChamber.z + t}, {kCenter.x - kChamber.x - t, kCenter.y, kCenter.z});
    wall({t, kChamber.y, kChamber.z + t}, {kCenter.x + kChamber.x + t, kCenter.y, kCenter.z});
    wall({kChamber.x, kChamber.y, t}, {kCenter.x, kCenter.y, kCenter.z - kChamber.z - t});
    wall({kChamber.x, kChamber.y, t}, {kCenter.x, kCenter.y, kCenter.z + kChamber.z + t});
    const int firstAtomBody = w.bodyCount();

    std::vector<Atom> atoms;
    std::vector<Bond> bonds;
    std::vector<int> parent;     // union-find over atoms (bonds never break)
    std::vector<int> bodyToAtom; // BodyId -> atom index, -1 for walls
    bodyToAtom.assign(firstAtomBody, -1);

    std::mt19937 rng(42);
    auto frand = [&](float lo, float hi) {
        return std::uniform_real_distribution<float>(lo, hi)(rng);
    };
    auto spawnAtom = [&](int elem) {
        const Element& e = kElements[elem];
        Vec3 pos{kCenter.x + frand(-kChamber.x + 1, kChamber.x - 1),
                 kCenter.y + frand(-kChamber.y + 1, kChamber.y - 1),
                 kCenter.z + frand(-kChamber.z + 1, kChamber.z - 1)};
        RigidBody b = RigidBody::makeDynamic(Shape::sphere(e.radius), pos, e.mass);
        Vec3 dir = normalize(Vec3{frand(-1, 1), frand(-1, 1), frand(-1, 1)});
        b.linearVelocity = dir * frand(1.5f, 4.0f);
        b.restitution = 0.85f;
        b.friction = 0.05f;
        BodyId id = w.addBody(b);
        bodyToAtom.push_back(atoms.size());
        parent.push_back(atoms.size());
        atoms.push_back({id, elem, e.valence});
    };

    // Initial gas mix: enough H for lots of water/methane/ammonia.
    for (int i = 0; i < 26; ++i) spawnAtom(0); // H
    for (int i = 0; i < 12; ++i) spawnAtom(1); // O
    for (int i = 0; i < 6; ++i) spawnAtom(2);  // N
    for (int i = 0; i < 6; ++i) spawnAtom(3);  // C

    viewer::Viewer v;
    v.init("minicollide - molecule chamber (SPACE: heat, 1-4: add H/O/N/C)",
           {15, 11, 15}, kCenter);
    viewer::Stepper stepper;

    while (!WindowShouldClose()) {
        if (IsKeyPressed(KEY_ONE)) spawnAtom(0);
        if (IsKeyPressed(KEY_TWO)) spawnAtom(1);
        if (IsKeyPressed(KEY_THREE)) spawnAtom(2);
        if (IsKeyPressed(KEY_FOUR)) spawnAtom(3);
        if (IsKeyPressed(KEY_SPACE)) // heat: +3 m/s in a random direction, every atom
            for (const Atom& a : atoms) {
                Vec3 dir = normalize(Vec3{frand(-1, 1), frand(-1, 1), frand(-1, 1)});
                w.applyImpulse(a.body, dir * (3.0f / w.body(a.body).invMass));
            }
        UpdateCamera(&v.camera, CAMERA_ORBITAL);

        stepper.advance(w);

        // Chemistry: scan this step's manifolds for touching atoms with free
        // valence. Same-molecule pairs are skipped (union-find), so molecules
        // stay acyclic and the formula census stays simple.
        for (const Manifold& m : w.manifolds()) {
            int ia = bodyToAtom[m.a], ib = bodyToAtom[m.b];
            if (ia < 0 || ib < 0 || m.count == 0) continue;
            Atom& A = atoms[ia];
            Atom& B = atoms[ib];
            if (A.freeValence <= 0 || B.freeValence <= 0) continue;
            int ra = ufFind(parent, ia), rb = ufFind(parent, ib);
            if (ra == rb) continue;
            w.addJoint(Joint::ballSocket(A.body, B.body, m.points[0].position,
                                         w.body(A.body), w.body(B.body)));
            --A.freeValence;
            --B.freeValence;
            parent[ra] = rb;
            bonds.push_back({ia, ib});
        }

        // Census: group atoms by union-find root, turn counts into formulas.
        std::map<std::string, int> census;
        int freeAtoms = 0;
        {
            std::map<int, std::array<int, kNumElements>> groups;
            for (int i = 0; i < (int)atoms.size(); ++i)
                groups[ufFind(parent, i)][atoms[i].elem]++;
            for (const auto& [root, counts] : groups) {
                int size = counts[0] + counts[1] + counts[2] + counts[3];
                if (size == 1) { ++freeAtoms; continue; }
                census[formulaFor(counts.data())]++;
            }
        }

        BeginDrawing();
        ClearBackground(Color{245, 245, 248, 255});
        BeginMode3D(v.camera);
        for (const Atom& a : atoms) {
            const RigidBody& b = w.body(a.body);
            Color c = kElements[a.elem].color;
            if (b.asleep) {
                c.r = (unsigned char)((c.r + 2 * 128) / 3);
                c.g = (unsigned char)((c.g + 2 * 128) / 3);
                c.b = (unsigned char)((c.b + 2 * 128) / 3);
            }
            v.drawBody(b, c);
        }
        for (const Bond& bd : bonds) {
            const Vec3& pa = w.body(atoms[bd.a].body).xf.position;
            const Vec3& pb = w.body(atoms[bd.b].body).xf.position;
            DrawCylinderEx({pa.x, pa.y, pa.z}, {pb.x, pb.y, pb.z}, 0.07f, 0.07f, 8,
                           Color{140, 140, 150, 255});
        }
        DrawCubeWires({kCenter.x, kCenter.y, kCenter.z},
                      2 * kChamber.x, 2 * kChamber.y, 2 * kChamber.z,
                      Color{150, 150, 160, 255});
        DrawGrid(40, 1.0f);
        EndMode3D();

        v.hud(w, TextFormat("atoms %d  bonds %d  free atoms %d",
                            (int)atoms.size(), (int)bonds.size(), freeAtoms));
        {
            std::vector<std::pair<std::string, int>> sorted(census.begin(), census.end());
            std::sort(sorted.begin(), sorted.end(),
                      [](const auto& x, const auto& y) { return x.second > y.second; });
            std::string line = "molecules: ";
            if (sorted.empty()) line += "none yet - press SPACE to heat";
            for (size_t i = 0; i < sorted.size() && i < 6; ++i) {
                if (i) line += "   ";
                line += sorted[i].first + " x" + std::to_string(sorted[i].second);
                if (const char* name = commonName(sorted[i].first))
                    line += std::string(" (") + name + ")";
            }
            if (sorted.size() > 6) line += "   ...";
            DrawText(line.c_str(), 12, 94, 18, Color{120, 60, 140, 255});
        }
        EndDrawing();
    }
    v.close();
    return 0;
}
