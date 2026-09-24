// M5 demo: performance. Mixed bodies rain into a pile; the HUD shows the
// per-phase profile so you can watch where the frame budget goes. Launch with
// an argument to set thread count (default: all cores), e.g. ./demo_pile 1
#include "viewer.h"
#include <cstdlib>
#include <thread>

using namespace mc;

int main(int argc, char** argv) {
    unsigned threads = std::thread::hardware_concurrency();
    if (argc > 1) threads = static_cast<unsigned>(std::atoi(argv[1]));

    World w(threads);
    w.addBody(RigidBody::makeStatic(Shape::box({14, 0.5f, 14}), {0, -0.5f, 0}));

    viewer::Viewer v;
    v.init(TextFormat("minicollide - pile, %u thread(s) (SPACE: pause spawn)", threads),
           {20, 14, 20}, {0, 3, 0});
    viewer::Stepper stepper;

    bool spawning = true;
    int spawned = 0, frame = 0;
    const int maxBodies = 600;

    while (!WindowShouldClose()) {
        if (IsKeyPressed(KEY_SPACE)) spawning = !spawning;
        UpdateCamera(&v.camera, CAMERA_ORBITAL);

        if (spawning && spawned < maxBodies && (frame++ % 3) == 0) {
            // deterministic pseudo-random spawn positions
            uint32_t s = 7919u * (spawned + 1);
            auto fr = [&s](float lo, float hi) {
                s = s * 1664525u + 1013904223u;
                return lo + (hi - lo) * (float)(s >> 8) / (float)(1 << 24);
            };
            Vec3 pos{fr(-6, 6), 12.0f + fr(0, 3), fr(-6, 6)};
            RigidBody b;
            int kind = spawned % 3;
            if (kind == 0) b = RigidBody::makeDynamic(Shape::sphere(fr(0.25f, 0.45f)), pos, 1.0f);
            else if (kind == 1) b = RigidBody::makeDynamic(Shape::box({fr(0.25f, 0.5f), fr(0.25f, 0.5f), fr(0.25f, 0.5f)}), pos, 1.0f);
            else b = RigidBody::makeDynamic(Shape::capsule(0.2f, fr(0.2f, 0.4f)), pos, 1.0f);
            b.linearVelocity = {fr(-1, 1), 0, fr(-1, 1)};
            w.addBody(b);
            ++spawned;
        }

        stepper.advance(w);

        BeginDrawing();
        ClearBackground(Color{245, 245, 248, 255});
        BeginMode3D(v.camera);
        v.drawWorld(w);
        EndMode3D();
        v.hud(w, TextFormat("spawned %d / %d   threads %u", spawned, maxBodies, threads));
        EndDrawing();
    }
    v.close();
    return 0;
}
