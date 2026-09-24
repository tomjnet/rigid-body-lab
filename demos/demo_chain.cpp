// M4 demo: joints. A capsule chain (ball-socket links) carrying a heavy
// wrecking ball swings into a box tower; a hinge "gate" sits behind it.
#include "viewer.h"
#include <thread>

using namespace mc;

int main() {
    World w(std::thread::hardware_concurrency());
    w.addBody(RigidBody::makeStatic(Shape::box({25, 0.5f, 25}), {0, -0.5f, 0}));

    // Chain anchor high up; links hang toward +x so gravity swings the ball into the tower.
    Vec3 anchorPos{0, 11, 0};
    BodyId anchor = w.addBody(RigidBody::makeStatic(Shape::sphere(0.15f), anchorPos));

    const int links = 7;
    const float linkLen = 0.9f; // segment + caps span per link
    BodyId prev = anchor;
    Vec3 dir = normalize(Vec3{0.75f, 0.66f, 0});  // hang the chain out at an angle (potential energy)
    Vec3 p = anchorPos;
    for (int i = 0; i < links; ++i) {
        Vec3 center = p + dir * (linkLen * 0.5f) * -1.0f; // extend downward along -dir
        center = p - dir * (linkLen * 0.5f);
        RigidBody link = RigidBody::makeDynamic(Shape::capsule(0.12f, linkLen * 0.5f - 0.12f), center, 0.8f);
        // orient the capsule's local Y along the chain direction
        Vec3 axis = cross(Vec3{0, 1, 0}, dir);
        float angle = std::acos(clampf(dot(Vec3{0, 1, 0}, dir), -1, 1));
        if (length(axis) > 1e-4f) link.xf.orientation = Quat::fromAxisAngle(axis, angle);
        BodyId id = w.addBody(link);
        w.addJoint(Joint::ballSocket(prev, id, p, w.body(prev), w.body(id)));
        prev = id;
        p = p - dir * linkLen;
    }
    RigidBody ballBody = RigidBody::makeDynamic(Shape::sphere(0.8f), p - dir * 0.8f, 25.0f);
    BodyId ball = w.addBody(ballBody);
    w.addJoint(Joint::ballSocket(prev, ball, p, w.body(prev), w.body(ball)));

    // Box tower in the swing path.
    for (int i = 0; i < 8; ++i)
        w.addBody(RigidBody::makeDynamic(Shape::box({0.45f, 0.45f, 0.45f}), {-3.5f, 0.45f + i * 0.9f, 0}, 0.8f));

    // Hinge gate: a door panel jointed to a static post, axis vertical.
    BodyId post = w.addBody(RigidBody::makeStatic(Shape::box({0.15f, 1.6f, 0.15f}), {-6.5f, 1.6f, 0}));
    RigidBody door = RigidBody::makeDynamic(Shape::box({0.08f, 1.4f, 1.1f}), {-6.5f, 1.7f, 1.25f}, 3.0f);
    BodyId doorId = w.addBody(door);
    w.addJoint(Joint::hinge(post, doorId, {-6.5f, 1.7f, 0.15f}, {0, 1, 0}, w.body(post), w.body(doorId)));

    viewer::Viewer v;
    v.init("minicollide - wrecking ball & hinge (SPACE: push ball)", {14, 10, 14}, {-2, 4, 0});
    viewer::Stepper stepper;

    while (!WindowShouldClose()) {
        if (IsKeyPressed(KEY_SPACE)) w.applyImpulse(ball, {-60, 0, 0});
        UpdateCamera(&v.camera, CAMERA_ORBITAL);

        stepper.advance(w);

        BeginDrawing();
        ClearBackground(Color{245, 245, 248, 255});
        BeginMode3D(v.camera);
        v.drawWorld(w);
        EndMode3D();
        v.hud(w, "ball-socket chain + hinge door");
        EndDrawing();
    }
    v.close();
    return 0;
}
