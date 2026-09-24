// Assert-style tests, no framework dependency. Each CHECK failure prints and
// increments a counter; the process exit code is the failure count.
#include "core/world.h"
#include "collision/collide.h"
#include "collision/gjk.h"
#include <cstdio>
#include <cmath>

using namespace mc;

static int g_failures = 0;
#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);    \
            ++g_failures;                                                  \
        }                                                                  \
    } while (0)
#define CHECK_NEAR(a, b, tol) CHECK(std::fabs((a) - (b)) <= (tol))

static void testMath() {
    // quaternion rotation matches matrix rotation
    Quat q = Quat::fromAxisAngle({0.3f, 1.0f, -0.5f}, 1.234f);
    Vec3 v{1.5f, -2.0f, 0.7f};
    Vec3 r1 = rotate(q, v);
    Vec3 r2 = q.toMat3() * v;
    CHECK(length(r1 - r2) < 1e-5f);

    // rotate then inverse-rotate is identity
    Vec3 r3 = rotateInv(q, r1);
    CHECK(length(r3 - v) < 1e-5f);

    // matrix inverse
    Mat3 R = q.toMat3();
    Mat3 shouldBeI = R * R.inverse();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            CHECK_NEAR(shouldBeI.c[i][j], i == j ? 1.0f : 0.0f, 1e-5f);

    // skew(v) * u == cross(v, u)
    Vec3 a{1, 2, 3}, b{-2, 0.5f, 4};
    CHECK(length(Mat3::skew(a) * b - cross(a, b)) < 1e-6f);

    // inertia rotation preserves symmetry
    RigidBody body = RigidBody::makeDynamic(Shape::box({0.5f, 1.0f, 0.25f}), {0, 0, 0}, 2.0f);
    body.xf.orientation = q;
    body.updateWorldInertia();
    const Mat3& I = body.invInertiaWorld;
    CHECK_NEAR(I.c[0].y, I.c[1].x, 1e-5f);
    CHECK_NEAR(I.c[0].z, I.c[2].x, 1e-5f);
}

static void testGjk() {
    // spheres 5 apart, radii 1 and 1 -> distance 3
    Shape s1 = Shape::sphere(1.0f);
    SupportShape a{&s1, {{0, 0, 0}, Quat::identity()}, false};
    SupportShape b{&s1, {{5, 0, 0}, Quat::identity()}, false};
    GjkResult g = gjkDistance(a, b);
    CHECK(!g.intersecting);
    CHECK_NEAR(g.distance, 3.0f, 1e-3f);
    CHECK_NEAR(g.pointA.x, 1.0f, 1e-2f);
    CHECK_NEAR(g.pointB.x, 4.0f, 1e-2f);

    // unit boxes with centers 2 apart on x -> gap of 1
    Shape box = Shape::box({0.5f, 0.5f, 0.5f});
    SupportShape ba{&box, {{0, 0, 0}, Quat::identity()}, false};
    SupportShape bb{&box, {{2, 0, 0}, Quat::identity()}, false};
    g = gjkDistance(ba, bb);
    CHECK(!g.intersecting);
    CHECK_NEAR(g.distance, 1.0f, 1e-3f);

    // rotated box corner-on: box at 45 deg about z, corner reaches sqrt(0.5)
    SupportShape br{&box, {{2, 0, 0}, Quat::fromAxisAngle({0, 0, 1}, kPi / 4)}, false};
    g = gjkDistance(ba, br);
    CHECK(!g.intersecting);
    CHECK_NEAR(g.distance, 2.0f - 0.5f - std::sqrt(0.5f), 1e-3f);

    // overlapping boxes are reported as intersecting, and EPA finds the axis
    SupportShape bo{&box, {{0.8f, 0, 0}, Quat::identity()}, false};
    g = gjkDistance(ba, bo);
    CHECK(g.intersecting);
    EpaResult e = epaPenetration(ba, bo);
    CHECK(e.valid);
    CHECK_NEAR(e.depth, 0.2f, 1e-2f);
    CHECK_NEAR(std::fabs(e.normal.x), 1.0f, 1e-3f);
}

static void testBoxBoxContact() {
    RigidBody ground = RigidBody::makeStatic(Shape::box({10, 0.5f, 10}), {0, -0.5f, 0});
    RigidBody box = RigidBody::makeDynamic(Shape::box({0.5f, 0.5f, 0.5f}), {0, 0.45f, 0}, 1.0f);
    Manifold m;
    int n = collideShapes(ground, box, m);
    CHECK(n == 4);                      // face-face contact clips to a quad
    CHECK_NEAR(m.normal.y, 1.0f, 1e-4f); // from ground (a) to box (b): up
    for (int i = 0; i < n; ++i) CHECK_NEAR(m.points[i].penetration, 0.05f, 1e-3f);
}

static void testSphereDropSettles() {
    World w;
    w.addBody(RigidBody::makeStatic(Shape::box({10, 0.5f, 10}), {0, -0.5f, 0}));
    BodyId ball = w.addBody(RigidBody::makeDynamic(Shape::sphere(0.5f), {0, 2.0f, 0}, 1.0f));
    for (int i = 0; i < 300; ++i) w.step(1.0f / 60.0f);
    CHECK_NEAR(w.body(ball).xf.position.y, 0.5f, 0.02f);
    CHECK(length(w.body(ball).linearVelocity) < 0.05f);
    CHECK(w.body(ball).asleep); // settled bodies must go to sleep
}

static void testStackStability() {
    World w;
    w.addBody(RigidBody::makeStatic(Shape::box({10, 0.5f, 10}), {0, -0.5f, 0}));
    const int N = 5;
    BodyId top = kInvalidBody;
    for (int i = 0; i < N; ++i)
        top = w.addBody(RigidBody::makeDynamic(Shape::box({0.5f, 0.5f, 0.5f}),
                                               {0, 0.5f + 1.0f * i, 0}, 1.0f));
    for (int i = 0; i < 600; ++i) w.step(1.0f / 60.0f);
    const RigidBody& t = w.body(top);
    CHECK_NEAR(t.xf.position.y, 0.5f + (N - 1), 0.08f); // still stacked
    CHECK(std::fabs(t.xf.position.x) < 0.1f && std::fabs(t.xf.position.z) < 0.1f);
}

static void testPendulumJoint() {
    World w;
    BodyId anchor = w.addBody(RigidBody::makeStatic(Shape::sphere(0.1f), {0, 5, 0}));
    BodyId bob = w.addBody(RigidBody::makeDynamic(Shape::sphere(0.25f), {2, 5, 0}, 1.0f));
    w.body(bob).lowVelocityTime = 0; // ensure awake
    Joint j = Joint::ballSocket(anchor, bob, {0, 5, 0}, w.body(anchor), w.body(bob));
    w.addJoint(j);
    w.enableSleeping = false;

    float maxErr = 0;
    for (int i = 0; i < 240; ++i) {
        w.step(1.0f / 60.0f);
        float len = length(w.body(bob).xf.position - Vec3{0, 5, 0});
        maxErr = std::max(maxErr, std::fabs(len - 2.0f));
    }
    CHECK(maxErr < 0.05f);                                  // constraint holds
    CHECK(w.body(bob).xf.position.y < 4.5f);                // it actually swings
}

static void testCapsuleRests() {
    World w;
    w.addBody(RigidBody::makeStatic(Shape::box({10, 0.5f, 10}), {0, -0.5f, 0}));
    // lying horizontally (axis along x after rotation about z)
    RigidBody cap = RigidBody::makeDynamic(Shape::capsule(0.25f, 0.5f), {0, 1.0f, 0}, 1.0f);
    cap.xf.orientation = Quat::fromAxisAngle({0, 0, 1}, kPi / 2);
    BodyId id = w.addBody(cap);
    for (int i = 0; i < 400; ++i) w.step(1.0f / 60.0f);
    CHECK_NEAR(w.body(id).xf.position.y, 0.25f, 0.03f);
}

static void testDeterminism() {
    auto run = [](unsigned threads) {
        World w(threads);
        w.addBody(RigidBody::makeStatic(Shape::box({20, 0.5f, 20}), {0, -0.5f, 0}));
        // a messy pile: boxes, spheres, capsules
        for (int i = 0; i < 40; ++i) {
            float x = (i % 5) * 0.6f - 1.2f;
            float z = ((i / 5) % 4) * 0.6f - 0.9f;
            float y = 1.0f + (i / 20) * 1.5f + (i % 3) * 0.1f;
            RigidBody b;
            if (i % 3 == 0)
                b = RigidBody::makeDynamic(Shape::sphere(0.3f), {x, y, z}, 1.0f);
            else if (i % 3 == 1)
                b = RigidBody::makeDynamic(Shape::box({0.25f, 0.25f, 0.25f}), {x, y, z}, 1.0f);
            else
                b = RigidBody::makeDynamic(Shape::capsule(0.15f, 0.25f), {x, y, z}, 1.0f);
            w.addBody(b);
        }
        for (int i = 0; i < 240; ++i) w.step(1.0f / 60.0f);
        return w.stateHash();
    };
    uint64_t h1 = run(4), h2 = run(4), h3 = run(1);
    CHECK(h1 == h2); // multithreaded runs match each other
    CHECK(h1 == h3); // and match the single-threaded result
}

int main() {
    testMath();
    testGjk();
    testBoxBoxContact();
    testSphereDropSettles();
    testStackStability();
    testPendulumJoint();
    testCapsuleRests();
    testDeterminism();
    if (g_failures == 0) std::printf("All tests passed.\n");
    else std::printf("%d check(s) failed.\n", g_failures);
    return g_failures;
}
