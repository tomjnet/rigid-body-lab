#pragma once
// The networked scene, shared by server and client. The server steps it; the
// client only uses it for shapes/colors (all motion comes from snapshots).
#include "core/world.h"

namespace mc::net {

inline void buildNetScene(mc::World& w) {
    using namespace mc;
    w.addBody(RigidBody::makeStatic(Shape::box({12, 0.5f, 12}), {0, -0.5f, 0}));

    // 5-box tower + small pyramid + spheres/capsules: 35 dynamic bodies
    for (int i = 0; i < 5; ++i)
        w.addBody(RigidBody::makeDynamic(Shape::box({0.5f, 0.5f, 0.5f}), {-3, 0.5f + i, 0}, 1.0f));
    for (int row = 0; row < 4; ++row)
        for (int i = 0; i < 4 - row; ++i)
            w.addBody(RigidBody::makeDynamic(Shape::box({0.45f, 0.45f, 0.45f}),
                                             {1.0f + i * 0.95f + row * 0.48f, 0.45f + row * 0.9f, 1.5f}, 1.0f));
    for (int i = 0; i < 10; ++i)
        w.addBody(RigidBody::makeDynamic(Shape::sphere(0.35f),
                                         {-1.5f + (i % 5) * 0.8f, 4.0f + (i / 5) * 1.2f, -2.5f}, 1.0f));
    for (int i = 0; i < 10; ++i) {
        RigidBody c = RigidBody::makeDynamic(Shape::capsule(0.18f, 0.3f),
                                             {-2.0f + (i % 5) * 0.9f, 6.0f + (i / 5) * 1.5f, 2.8f}, 0.8f);
        c.xf.orientation = Quat::fromAxisAngle({1, 0, 0.3f}, 0.4f * i);
        w.addBody(c);
    }
}

} // namespace mc::net
