// Snapshot-interpolation client: buffers server snapshots and renders ~100 ms
// in the past, lerping positions and nlerping orientations between the two
// bracketing snapshots. Never simulates.
//
//   ./net_client [--port N] [--probe N]
// --probe N: headless mode; receive N snapshots, verify interpolation
// continuity, print stats, exit (used for automated verification).
#include "net_scene.h"
#include "protocol.h"
#ifdef MC_HAVE_RAYLIB
#include "viewer.h"
#endif

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

using namespace mc;
using namespace mc::net;

struct Snapshot {
    uint32_t tick = 0;
    std::vector<BodyState> states;
};

struct Connection {
    int sock = -1;
    sockaddr_in server{};

    bool open(uint16_t port) {
        sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0) return false;
        fcntl(sock, F_SETFL, O_NONBLOCK);
        server.sin_family = AF_INET;
        server.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        server.sin_port = htons(port);
        sendMsg(ClientMsg::Hello);
        return true;
    }
    void sendMsg(ClientMsg m) {
        uint8_t b = static_cast<uint8_t>(m);
        sendto(sock, &b, 1, 0, reinterpret_cast<sockaddr*>(&server), sizeof(server));
    }
    bool receive(Snapshot& out) {
        uint8_t buf[2048];
        ssize_t n = recv(sock, buf, sizeof(buf), 0);
        if (n < static_cast<ssize_t>(sizeof(SnapshotHeader))) return false;
        auto* hdr = reinterpret_cast<SnapshotHeader*>(buf);
        if (hdr->magic != kMagic) return false;
        size_t need = sizeof(SnapshotHeader) + sizeof(BodyState) * hdr->bodyCount;
        if (static_cast<size_t>(n) < need) return false;
        out.tick = hdr->tick;
        out.states.assign(reinterpret_cast<BodyState*>(buf + sizeof(SnapshotHeader)),
                          reinterpret_cast<BodyState*>(buf + need));
        return true;
    }
};

// Interpolate body i at renderTick from the snapshot buffer.
static bool sampleState(const std::deque<Snapshot>& buffer, float renderTick, int i,
                        Vec3& pos, Quat& rot) {
    if (buffer.empty()) return false;
    const Snapshot* s0 = &buffer.front();
    const Snapshot* s1 = nullptr;
    for (const auto& s : buffer) {
        if (s.tick <= renderTick) s0 = &s;
        else { s1 = &s; break; }
    }
    if (i >= static_cast<int>(s0->states.size())) return false;
    const BodyState& a = s0->states[i];
    if (!s1) { // extrapolation-free: clamp to newest known state
        pos = {a.px, a.py, a.pz};
        rot = {a.qx, a.qy, a.qz, a.qw};
        return true;
    }
    const BodyState& b = s1->states[i];
    float t = clampf((renderTick - s0->tick) / float(s1->tick - s0->tick), 0, 1);
    pos = Vec3{a.px, a.py, a.pz} * (1 - t) + Vec3{b.px, b.py, b.pz} * t;
    // nlerp with hemisphere fix
    float d = a.qx * b.qx + a.qy * b.qy + a.qz * b.qz + a.qw * b.qw;
    float sign = d >= 0 ? 1.0f : -1.0f;
    rot = Quat{a.qx + (b.qx * sign - a.qx) * t, a.qy + (b.qy * sign - a.qy) * t,
               a.qz + (b.qz * sign - a.qz) * t, a.qw + (b.qw * sign - a.qw) * t}.normalized();
    return true;
}

static int runProbe(Connection& conn, int wanted) {
    std::deque<Snapshot> buffer;
    int received = 0;
    float maxJump = 0;
    std::vector<Vec3> lastPos;
    auto now = [] { return std::chrono::steady_clock::now(); };
    auto deadline = now() + std::chrono::seconds(30);
    auto nextHello = now(); // re-send Hello until snapshots flow (server may start late)

    while (received < wanted && now() < deadline) {
        if (now() >= nextHello) {
            conn.sendMsg(ClientMsg::Hello);
            nextHello = now() + std::chrono::seconds(1);
        }
        Snapshot s;
        if (conn.receive(s)) {
            buffer.push_back(std::move(s));
            ++received;
            if (received % 10 == 0) conn.sendMsg(ClientMsg::Poke); // stir the sim so motion is nonzero

            // Sample the interpolated state and track per-snapshot movement.
            float renderTick = buffer.back().tick - static_cast<float>(kInterpTicks);
            Vec3 p; Quat q;
            if (sampleState(buffer, renderTick, 1, p, q)) {
                if (!lastPos.empty()) maxJump = std::max(maxJump, length(p - lastPos[0]));
                lastPos.assign(1, p);
            }
            while (buffer.size() > 32) buffer.pop_front();
        } else {
            usleep(2000);
        }
    }
    std::printf("probe: received %d/%d snapshots, max interpolated jump %.3f m\n",
                received, wanted, maxJump);
    // A body should never teleport between consecutive interpolated samples;
    // 3 ticks (one snapshot interval) of free fall at ~10 m/s is ~0.5 m.
    bool ok = received == wanted && maxJump < 1.0f;
    std::printf("probe: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}

int main(int argc, char** argv) {
    uint16_t port = kDefaultPort;
    int probe = 0;
    for (int i = 1; i < argc - 1; ++i) {
        if (std::string(argv[i]) == "--port") port = static_cast<uint16_t>(std::atoi(argv[i + 1]));
        if (std::string(argv[i]) == "--probe") probe = std::atoi(argv[i + 1]);
    }

    Connection conn;
    if (!conn.open(port)) { std::perror("socket"); return 1; }
    if (probe > 0) return runProbe(conn, probe);

#ifdef MC_HAVE_RAYLIB
    // Local world used only as the scene description (shapes/colors).
    World scene;
    buildNetScene(scene);

    viewer::Viewer v;
    v.init("minicollide - net client (SPACE: poke server)", {14, 10, 14}, {0, 2, 0});

    std::deque<Snapshot> buffer;
    double lastHello = 0;
    while (!WindowShouldClose()) {
        if (GetTime() - lastHello > 1.0) { conn.sendMsg(ClientMsg::Hello); lastHello = GetTime(); }
        if (IsKeyPressed(KEY_SPACE)) conn.sendMsg(ClientMsg::Poke);
        UpdateCamera(&v.camera, CAMERA_ORBITAL);

        Snapshot s;
        while (conn.receive(s)) {
            buffer.push_back(std::move(s));
            while (buffer.size() > 64) buffer.pop_front();
        }

        BeginDrawing();
        ClearBackground(Color{245, 245, 248, 255});
        BeginMode3D(v.camera);
        if (!buffer.empty()) {
            float renderTick = buffer.back().tick - static_cast<float>(kInterpTicks);
            for (int i = 0; i < scene.bodyCount(); ++i) {
                Vec3 p; Quat q;
                if (sampleState(buffer, renderTick, i, p, q)) {
                    scene.body(i).xf.position = p;
                    scene.body(i).xf.orientation = q;
                    scene.body(i).asleep =
                        buffer.back().states[i].asleep != 0 && !scene.body(i).isStatic();
                }
                v.drawBody(scene.body(i), viewer::Viewer::bodyColor(scene.body(i), i));
            }
        }
        DrawGrid(30, 1.0f);
        EndMode3D();
        DrawFPS(12, 10);
        DrawText(buffer.empty() ? "waiting for server on 127.0.0.1..."
                                : TextFormat("snapshots buffered: %zu  latest tick: %u",
                                             buffer.size(), buffer.back().tick),
                 12, 34, 18, DARKGRAY);
        EndDrawing();
    }
    v.close();
    return 0;
#else
    std::printf("built without raylib; use --probe N for headless mode\n");
    return 1;
#endif
}
