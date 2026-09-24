// M3 credibility demo: a 10-box stack plus a pyramid that must rest without
// jitter (warm starting + Baumgarte + slop). SPACE fires a heavy sphere;
// watch sleeping bodies (desaturated) wake as the island graph reconnects.
#include "viewer.h"
#include <thread>

using namespace mc;

int main() {
    World w(std::thread::hardware_concurrency());
    w.addBody(RigidBody::makeStatic(Shape::box({25, 0.5f, 25}), {0, -0.5f, 0}));

    // 10-box tower
    for (int i = 0; i < 10; ++i)
        w.addBody(RigidBody::makeDynamic(Shape::box({0.5f, 0.5f, 0.5f}), {-4, 0.5f + i, 0}, 1.0f));

    // pyramid
    const int base = 7;
    for (int row = 0; row < base; ++row)
        for (int i = 0; i < base - row; ++i)
            w.addBody(RigidBody::makeDynamic(
                Shape::box({0.5f, 0.5f, 0.5f}),
                {2.0f + i * 1.02f + row * 0.51f, 0.5f + row * 1.0f, 0}, 1.0f));

    viewer::Viewer v;
    v.init("minicollide - stack & pyramid (SPACE: fire sphere, R: reset camera)");
    viewer::Stepper stepper;

    int shots = 0;
    while (!WindowShouldClose()) {
        if (IsKeyPressed(KEY_SPACE)) {
            Vector3 cp = v.camera.position;
            Vec3 origin{cp.x, cp.y, cp.z};
            Vec3 dir = normalize(Vec3{v.camera.target.x, v.camera.target.y, v.camera.target.z} - origin);
            RigidBody ball = RigidBody::makeDynamic(Shape::sphere(0.6f), origin + dir * 1.5f, 8.0f);
            ball.linearVelocity = dir * 25.0f;
            ball.restitution = 0.3f;
            w.addBody(ball);
            ++shots;
        }
        UpdateCamera(&v.camera, CAMERA_ORBITAL);

        stepper.advance(w);

        BeginDrawing();
        ClearBackground(Color{245, 245, 248, 255});
        BeginMode3D(v.camera);
        v.drawWorld(w);
        EndMode3D();
        v.hud(w, TextFormat("spheres fired: %d", shots));
        EndDrawing();
    }
    v.close();
    return 0;
}
