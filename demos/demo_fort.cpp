// Gameplay sandbox: everything the engine provides, driven like a game.
//
//   - First-person character: a capsule with zero inverse inertia (can't tip
//     over), horizontal velocity driven directly, grounded state read from the
//     contact manifolds, jump only when grounded.
//   - Build mode: place wall / floor / ramp panels on a 3 m grid, snapped to
//     the view direction. Panels are static bodies with hit points.
//   - Shooting: a recycled pool of physical sphere projectiles. Panels that
//     reach 0 HP convert from static to dynamic and collapse physically.
//   - Targets: capsule dummies that the physics knocks over; score counts them.
//
// Controls: WASD or arrow keys move, mouse look, SPACE or BACKSPACE jump,
//           LMB shoot/place, B toggle build mode, 1/2/3 wall/floor/ramp,
//           R respawn, ESC quit.
#include "viewer.h"
#include <cmath>
#include <thread>
#include <unordered_map>
#include <vector>

using namespace mc;

namespace {

enum class Piece { Wall, Floor, Ramp };

struct Panel {
    float health = 100;
    bool alive = true; // false once converted to dynamic debris
};

struct Shot {
    BodyId id = kInvalidBody;
    bool active = false;
    bool dealtDamage = false;
    float age = 0;
};

constexpr float kCell = 3.0f; // build grid size

// Local-frame shape + tilt for each buildable piece (+z = facing direction).
Shape pieceShape(Piece p) {
    switch (p) {
    case Piece::Wall:  return Shape::box({1.5f, 1.5f, 0.15f});
    case Piece::Floor: return Shape::box({1.5f, 0.1f, 1.5f});
    case Piece::Ramp:  return Shape::box({1.5f, 0.12f, 2.12f});
    }
    return Shape::box({1, 1, 1});
}

mc::Transform pieceTransform(Piece p, const Vec3& playerPos, float yaw) {
    float snapYaw = std::round(yaw / (kPi / 2)) * (kPi / 2);
    Vec3 fwd{std::sin(snapYaw), 0, std::cos(snapYaw)};
    Vec3 base = playerPos + fwd * kCell;
    float cx = std::round(base.x / kCell) * kCell;
    float cz = std::round(base.z / kCell) * kCell;
    float y0 = std::round((playerPos.y - 1.0f) / kCell) * kCell; // current story

    mc::Transform xf;
    Quat yawQ = Quat::fromAxisAngle({0, 1, 0}, snapYaw);
    switch (p) {
    case Piece::Wall:
        xf.position = {cx, y0 + 1.5f, cz};
        xf.orientation = yawQ;
        break;
    case Piece::Floor:
        xf.position = {cx, y0 + 0.1f, cz};
        xf.orientation = yawQ;
        break;
    case Piece::Ramp:
        xf.position = {cx, y0 + 1.5f, cz};
        xf.orientation = yawQ * Quat::fromAxisAngle({1, 0, 0}, -kPi / 4); // far edge high
        break;
    }
    return xf;
}

} // namespace

int main() {
    World w(std::thread::hardware_concurrency());
    w.addBody(RigidBody::makeStatic(Shape::box({30, 0.5f, 30}), {0, -0.5f, 0}));

    // Player: locked rotation, driven kinematically through velocities.
    RigidBody pb = RigidBody::makeDynamic(Shape::capsule(0.4f, 0.5f), {0, 1.2f, 16}, 70.0f);
    pb.invInertiaLocal = Mat3::zero();
    pb.updateWorldInertia();
    pb.friction = 0.1f;
    pb.restitution = 0;
    const BodyId player = w.addBody(pb);

    // Target dummies in a loose circle: slender boxes, so they stand stably on
    // a 4-point manifold until a hit tips them (an upright capsule would rest
    // on a single point and topple on its own, like a balanced egg).
    std::vector<BodyId> dummies;
    for (int i = 0; i < 6; ++i) {
        float a = i * (2 * kPi / 6);
        RigidBody d = RigidBody::makeDynamic(Shape::box({0.32f, 0.85f, 0.32f}),
                                             {std::sin(a) * 8.0f, 0.85f, std::cos(a) * 8.0f}, 8.0f);
        d.friction = 0.7f;
        dummies.push_back(w.addBody(d));
    }

    // A crate pyramid and a small pre-built fort to knock down.
    for (int row = 0; row < 4; ++row)
        for (int i = 0; i < 4 - row; ++i)
            w.addBody(RigidBody::makeDynamic(Shape::box({0.45f, 0.45f, 0.45f}),
                                             {4.0f + i * 0.95f + row * 0.48f, 0.45f + row * 0.9f, -5.0f}, 1.0f));
    std::unordered_map<BodyId, Panel> panels;
    auto addFortWall = [&](Vec3 pos, float yaw) {
        BodyId id = w.addBody(RigidBody::makeStatic(pieceShape(Piece::Wall), pos,
                                                    Quat::fromAxisAngle({0, 1, 0}, yaw)));
        panels.emplace(id, Panel{});
    };
    addFortWall({-7, 1.5f, -7}, 0);
    addFortWall({-8.5f, 1.5f, -5.5f}, kPi / 2);
    addFortWall({-5.5f, 1.5f, -5.5f}, kPi / 2);

    // ---- city block: hollow buildings with door + window openings --------
    // Each facade is assembled per story from a spandrel band (below the
    // windows), piers (between the windows), and a lintel band (above them);
    // the openings are real gaps, so windows are shootable and the door is
    // walkable. Roofs are flat with a bulkhead box, reachable via built ramps.
    std::unordered_map<BodyId, Color> buildingTint;
    auto addBuilding = [&](float cx, float cz, float hw, float hd, int stories, Color tint) {
        const float storyH = 3.0f, t = 0.22f;
        const float sillH = 0.8f, winTop = 2.6f;
        auto addPart = [&](Vec3 center, Vec3 half) {
            buildingTint.emplace(w.addBody(RigidBody::makeStatic(Shape::box(half), center)), tint);
        };
        // Door on the facade that faces the central intersection.
        int doorFacade; // 0:+z 1:-z 2:+x 3:-x (outward normal)
        if (std::fabs(cx) > std::fabs(cz)) doorFacade = cx > 0 ? 3 : 2;
        else doorFacade = cz > 0 ? 1 : 0;

        struct Seg { float a, b; };
        auto subtract = [](std::vector<Seg> segs, float ca, float cb) {
            std::vector<Seg> out;
            for (auto& s : segs) {
                if (cb <= s.a || ca >= s.b) { out.push_back(s); continue; }
                if (ca > s.a) out.push_back({s.a, ca});
                if (cb < s.b) out.push_back({cb, s.b});
            }
            return out;
        };

        for (int f = 0; f < 4; ++f) {
            bool alongX = f < 2; // z-facing walls run along x
            float sign = (f == 0 || f == 2) ? 1.0f : -1.0f;
            float hspan = alongX ? hw : hd;

            int nWin = 3;
            const float wWin = 1.1f;
            while (nWin > 1 && nWin * wWin + (nWin + 1) * 0.6f > 2 * hspan) --nWin;
            float pierW = (2 * hspan - nWin * wWin) / (nWin + 1);
            std::vector<Seg> wins;
            for (int k = 0; k < nWin; ++k) {
                float a = -hspan + pierW * (k + 1) + wWin * k;
                wins.push_back({a, a + wWin});
            }

            auto place = [&](float a, float b, float y0, float y1) {
                if (b - a < 0.05f || y1 - y0 < 0.05f) return;
                Vec3 c, half;
                float mid = (a + b) * 0.5f, len = b - a, yc = (y0 + y1) * 0.5f, h = y1 - y0;
                if (alongX) { c = {cx + mid, yc, cz + sign * hd}; half = {len / 2, h / 2, t / 2}; }
                else { c = {cx + sign * hw, yc, cz + mid}; half = {t / 2, h / 2, len / 2}; }
                addPart(c, half);
            };

            for (int s = 0; s < stories; ++s) {
                float y0 = s * storyH;
                std::vector<Seg> spandrel{{-hspan, hspan}};
                std::vector<Seg> piers{{-hspan, hspan}};
                for (auto& wnd : wins) piers = subtract(piers, wnd.a, wnd.b);
                if (f == doorFacade && s == 0) { // carve the doorway
                    spandrel = subtract(spandrel, -0.9f, 0.9f);
                    piers = subtract(piers, -0.9f, 0.9f);
                }
                for (auto& sg : spandrel) place(sg.a, sg.b, y0, y0 + sillH);
                for (auto& sg : piers) place(sg.a, sg.b, y0 + sillH, y0 + winTop);
                place(-hspan, hspan, y0 + winTop, y0 + storyH);
            }
        }
        float roofY = stories * storyH;
        addPart({cx, roofY + 0.1f, cz}, {hw, 0.1f, hd});
        addPart({cx + hw * 0.4f, roofY + 0.75f, cz + hd * 0.4f}, {0.9f, 0.55f, 0.9f});
    };
    addBuilding(-14, -14, 5, 5, 4, Color{158, 82, 66, 255});   // brick
    addBuilding(14, -14, 5, 5, 5, Color{134, 134, 142, 255});  // gray stone
    addBuilding(-14, 14, 5, 5, 3, Color{184, 160, 124, 255});  // tan
    addBuilding(14, 14, 5, 5, 6, Color{118, 98, 86, 255});     // brownstone

    // Projectile pool, parked asleep far below the arena.
    const int kMaxShots = 20;
    std::vector<Shot> shots(kMaxShots);
    for (int i = 0; i < kMaxShots; ++i) {
        RigidBody s = RigidBody::makeDynamic(Shape::sphere(0.22f), {200.0f + i * 3.0f, -60, 200}, 2.0f);
        s.restitution = 0.35f;
        shots[i].id = w.addBody(s);
        w.body(shots[i].id).asleep = true;
    }
    auto shotByBody = [&](BodyId id) -> Shot* {
        for (auto& s : shots)
            if (s.id == id) return &s;
        return nullptr;
    };

    viewer::Viewer v;
    v.init("rigid-body-lab - build & battle sandbox");
    SetExitKey(KEY_NULL); // ESC pauses instead of closing the window
    viewer::Stepper stepper;

    enum class Screen { Menu, Controls, Playing, Paused };
    Screen screen = Screen::Menu; // start on the title menu (cursor visible)
    bool quit = false;
    float menuYaw = 0.6f;

    float yaw = kPi, pitch = -0.05f;
    bool buildMode = false, grounded = false;
    Piece piece = Piece::Wall;
    int nextShot = 0, dummiesDown = 0;
    float shootCooldown = 0, buildCooldown = 0;
    mc::Transform ghost{};
    Shape ghostShape = pieceShape(piece);
    bool ghostValid = false;

    auto enterScreen = [&](Screen s) {
        if (s == Screen::Playing) {
            DisableCursor();
            shootCooldown = std::max(shootCooldown, 0.3f); // swallow the menu click
        } else {
            EnableCursor();
        }
        screen = s;
    };

    // Centered menu button; returns true on click.
    auto button = [&](float y, const char* label) {
        const float bw = 280, bh = 48;
        Rectangle r{GetScreenWidth() / 2.0f - bw / 2, y, bw, bh};
        bool hover = CheckCollisionPointRec(GetMousePosition(), r);
        DrawRectangleRec(r, hover ? Color{45, 120, 200, 235} : Color{22, 54, 96, 210});
        DrawRectangleLinesEx(r, 2, RAYWHITE);
        int tw = MeasureText(label, 24);
        DrawText(label, (int)(r.x + (bw - tw) / 2), (int)(y + 12), 24, RAYWHITE);
        return hover && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
    };

    while (!WindowShouldClose() && !quit) {
        float ft = GetFrameTime();

        if (screen == Screen::Playing) {
            shootCooldown = std::max(0.0f, shootCooldown - ft);
            buildCooldown = std::max(0.0f, buildCooldown - ft);
            if (IsKeyPressed(KEY_ESCAPE)) enterScreen(Screen::Paused);

            // ---- look & move ---------------------------------------------
            Vector2 md = GetMouseDelta();
            yaw -= md.x * 0.0032f;
            pitch = clampf(pitch - md.y * 0.0032f, -1.45f, 1.45f);
            Vec3 fwd{std::cos(pitch) * std::sin(yaw), std::sin(pitch), std::cos(pitch) * std::cos(yaw)};
            Vec3 fwdFlat = normalize(Vec3{fwd.x, 0, fwd.z});
            Vec3 right = cross(fwdFlat, Vec3{0, 1, 0});

            RigidBody& P = w.body(player);
            Vec3 wish{};
            if (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP)) wish += fwdFlat;
            if (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN)) wish -= fwdFlat;
            if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) wish += right;
            if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT)) wish -= right;
            float speed = IsKeyDown(KEY_LEFT_SHIFT) ? 9.0f : 6.0f;
            if (lengthSq(wish) > 0) wish = normalize(wish);
            P.linearVelocity.x = wish.x * speed;
            P.linearVelocity.z = wish.z * speed;
            P.asleep = false;
            P.lowVelocityTime = 0;
            if ((IsKeyDown(KEY_SPACE) || IsKeyDown(KEY_BACKSPACE)) && grounded)
                P.linearVelocity.y = 8.0f;
            if (IsKeyPressed(KEY_R) || P.xf.position.y < -15) {
                P.xf.position = {0, 1.2f, 16};
                P.linearVelocity = {};
            }

            // ---- build & shoot -------------------------------------------
            if (IsKeyPressed(KEY_B)) buildMode = !buildMode;
            if (IsKeyPressed(KEY_ONE)) piece = Piece::Wall;
            if (IsKeyPressed(KEY_TWO)) piece = Piece::Floor;
            if (IsKeyPressed(KEY_THREE)) piece = Piece::Ramp;

            ghost = pieceTransform(piece, P.xf.position, yaw);
            ghostShape = pieceShape(piece);
            ghostValid = false;
            if (buildMode) {
                AABB pieceBox = ghostShape.computeAABB(ghost);
                AABB playerBox = P.shape.computeAABB(P.xf);
                ghostValid = !pieceBox.overlaps(playerBox);
            }

            Vec3 muzzle = P.xf.position + Vec3{0, 0.75f, 0};
            if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
                if (buildMode && ghostValid && buildCooldown == 0) {
                    BodyId id = w.addBody(RigidBody::makeStatic(ghostShape, ghost.position, ghost.orientation));
                    panels.emplace(id, Panel{});
                    buildCooldown = 0.25f;
                } else if (!buildMode && shootCooldown == 0) {
                    Shot& s = shots[nextShot];
                    nextShot = (nextShot + 1) % kMaxShots;
                    RigidBody& B = w.body(s.id);
                    B.xf.position = muzzle + fwd * 1.0f;
                    B.linearVelocity = fwd * 20.0f;
                    B.angularVelocity = {};
                    B.asleep = false;
                    B.lowVelocityTime = 0;
                    s.active = true;
                    s.dealtDamage = false;
                    s.age = 0;
                    shootCooldown = 0.22f;
                }
            }

            stepper.advance(w);

            // ---- post-step game logic ------------------------------------
            grounded = false;
            for (const auto& m : w.manifolds()) {
                if (m.b == player && m.normal.y > 0.6f) grounded = true;
                if (m.a == player && m.normal.y < -0.6f) grounded = true;

                // projectile damage to panels
                Shot* s = nullptr;
                BodyId other = kInvalidBody;
                if ((s = shotByBody(m.a))) other = m.b;
                else if ((s = shotByBody(m.b))) other = m.a;
                if (s && s->active && !s->dealtDamage && length(w.body(s->id).linearVelocity) > 6.0f) {
                    auto it = panels.find(other);
                    if (it != panels.end() && it->second.alive) {
                        it->second.health -= 40.0f;
                        s->dealtDamage = true;
                        if (it->second.health <= 0) {
                            // static -> dynamic: the panel collapses under physics
                            RigidBody& pnl = w.body(other);
                            float mass = 25.0f;
                            pnl.invMass = 1.0f / mass;
                            pnl.invInertiaLocal = pnl.shape.invInertia(mass);
                            pnl.updateWorldInertia();
                            pnl.linearVelocity = w.body(s->id).linearVelocity * 0.15f;
                            w.wake(other);
                            it->second.alive = false;
                        }
                    }
                }
            }
            for (auto& s : shots) {
                if (!s.active) continue;
                s.age += ft;
                if (s.age > 4.0f) { // recycle: park asleep below the arena
                    RigidBody& B = w.body(s.id);
                    B.xf.position = {200.0f + float(&s - shots.data()) * 3.0f, -60, 200};
                    B.linearVelocity = {};
                    B.angularVelocity = {};
                    B.asleep = true;
                    s.active = false;
                }
            }
            dummiesDown = 0;
            for (BodyId d : dummies)
                if (rotate(w.body(d).xf.orientation, Vec3{0, 1, 0}).y < 0.5f) ++dummiesDown;
        } else {
            // Menus: world frozen; slow orbital camera over the arena.
            menuYaw += ft * 0.12f;
            if (screen == Screen::Paused && IsKeyPressed(KEY_ESCAPE)) enterScreen(Screen::Playing);
            if (screen == Screen::Controls && IsKeyPressed(KEY_ESCAPE)) screen = Screen::Menu;
        }

        // ---- render ------------------------------------------------------
        if (screen == Screen::Menu || screen == Screen::Controls) {
            v.camera.position = {std::sin(menuYaw) * 36.0f, 19.0f, std::cos(menuYaw) * 36.0f};
            v.camera.target = {0, 4, 0};
        } else {
            const RigidBody& Pr = w.body(player);
            Vec3 eye = Pr.xf.position + Vec3{0, 0.75f, 0};
            Vec3 look{std::cos(pitch) * std::sin(yaw), std::sin(pitch), std::cos(pitch) * std::cos(yaw)};
            v.camera.position = {eye.x, eye.y, eye.z};
            v.camera.target = {eye.x + look.x, eye.y + look.y, eye.z + look.z};
        }

        BeginDrawing();
        ClearBackground(Color{198, 222, 240, 255});
        BeginMode3D(v.camera);
        for (int i = 0; i < w.bodyCount(); ++i) {
            if (i == player) continue;
            const RigidBody& b = w.body(i);
            if (Shot* s = shotByBody(i)) {
                if (s->active) v.drawBody(b, Color{40, 40, 48, 255});
                continue;
            }
            auto bt = buildingTint.find(i);
            if (bt != buildingTint.end()) {
                v.drawBody(b, bt->second);
                continue;
            }
            auto it = panels.find(i);
            if (it != panels.end() && it->second.alive) {
                // tint placed/fort panels by remaining health
                float h = clampf(it->second.health / 100.0f, 0, 1);
                v.drawBody(b, Color{(unsigned char)(150 + 90 * (1 - h)),
                                    (unsigned char)(120 * h + 60), 70, 255});
                continue;
            }
            bool isDummy = false;
            for (BodyId d : dummies) isDummy = isDummy || d == i;
            v.drawBody(b, isDummy ? Color{210, 60, 60, 255} : viewer::Viewer::bodyColor(b, i));
        }
        if (screen == Screen::Playing && buildMode) { // translucent placement ghost
            RigidBody g;
            g.shape = ghostShape;
            g.xf = ghost;
            g.invMass = 0;
            v.drawBody(g, ghostValid ? Color{80, 180, 250, 110} : Color{250, 80, 80, 110});
        }
        DrawGrid(20, kCell);
        EndMode3D();

        const int sw = GetScreenWidth(), sh = GetScreenHeight();
        if (screen == Screen::Playing) {
            DrawLine(sw / 2 - 9, sh / 2, sw / 2 + 9, sh / 2, DARKGRAY);
            DrawLine(sw / 2, sh / 2 - 9, sw / 2, sh / 2 + 9, DARKGRAY);
            const char* pieceName = piece == Piece::Wall ? "wall" : piece == Piece::Floor ? "floor" : "ramp";
            DrawText(buildMode ? TextFormat("BUILD MODE [%s]  (1 wall / 2 floor / 3 ramp, LMB place)", pieceName)
                               : "COMBAT MODE  (LMB shoot, B to build)",
                     12, 10, 20, buildMode ? Color{20, 110, 200, 255} : Color{160, 40, 40, 255});
            DrawText(TextFormat("dummies down %d / %d   panels %zu", dummiesDown, (int)dummies.size(), panels.size()),
                     12, 36, 18, DARKGRAY);
            DrawText("WASD/arrows + mouse move | SPACE/BACKSPACE jump | SHIFT sprint | R respawn | ESC menu",
                     12, 58, 16, GRAY);
            DrawText(TextFormat("step %.2f ms  bodies %d  awake %d", w.profile().total(), w.bodyCount(), w.awakeCount()),
                     12, sh - 26, 16, GRAY);
        } else if (screen == Screen::Menu) {
            DrawRectangle(0, 0, sw, sh, Color{12, 18, 30, 150});
            const char* title = "BUILD & BATTLE";
            DrawText(title, (sw - MeasureText(title, 62)) / 2, (int)(sh * 0.16f), 62, RAYWHITE);
            const char* sub = "a rigid-body-lab physics sandbox";
            DrawText(sub, (sw - MeasureText(sub, 22)) / 2, (int)(sh * 0.16f) + 70, 22, Color{170, 190, 210, 255});
            float y0 = sh * 0.42f;
            if (button(y0, "PLAY")) enterScreen(Screen::Playing);
            if (button(y0 + 62, "CONTROLS")) screen = Screen::Controls;
            if (button(y0 + 124, "QUIT")) quit = true;
        } else if (screen == Screen::Controls) {
            DrawRectangle(0, 0, sw, sh, Color{12, 18, 30, 170});
            const char* title = "CONTROLS";
            DrawText(title, (sw - MeasureText(title, 44)) / 2, (int)(sh * 0.14f), 44, RAYWHITE);
            const char* lines[] = {
                "WASD / arrow keys ....... move",
                "mouse ................... look",
                "SPACE or BACKSPACE ...... jump (when grounded)",
                "LEFT SHIFT .............. sprint",
                "LMB ..................... shoot / place panel",
                "B ....................... toggle build mode",
                "1 / 2 / 3 ............... wall / floor / ramp",
                "R ....................... respawn",
                "ESC ..................... pause menu",
            };
            int y = (int)(sh * 0.14f) + 70;
            for (const char* l : lines) {
                DrawText(l, sw / 2 - 230, y, 20, Color{200, 214, 228, 255});
                y += 30;
            }
            if (button((float)(y + 16), "BACK")) screen = Screen::Menu;
        } else { // Paused
            DrawRectangle(0, 0, sw, sh, Color{12, 18, 30, 120});
            const char* title = "PAUSED";
            DrawText(title, (sw - MeasureText(title, 50)) / 2, (int)(sh * 0.2f), 50, RAYWHITE);
            float y0 = sh * 0.42f;
            if (button(y0, "RESUME")) enterScreen(Screen::Playing);
            if (button(y0 + 62, "MAIN MENU")) enterScreen(Screen::Menu);
            if (button(y0 + 124, "QUIT")) quit = true;
        }
        EndDrawing();
    }
    v.close();
    return 0;
}
