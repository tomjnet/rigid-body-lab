// Authoritative headless physics server: steps the shared scene at 60 Hz and
// broadcasts snapshots over UDP to every client that says Hello.
//
//   ./net_server [--port N] [--frames N]   (--frames for finite test runs)
#include "net_scene.h"
#include "protocol.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

using namespace mc;
using namespace mc::net;

struct ClientAddr {
    sockaddr_in addr;
    bool operator==(const ClientAddr& o) const {
        return addr.sin_addr.s_addr == o.addr.sin_addr.s_addr && addr.sin_port == o.addr.sin_port;
    }
};

int main(int argc, char** argv) {
    uint16_t port = kDefaultPort;
    long maxFrames = -1;
    for (int i = 1; i < argc - 1; ++i) {
        if (std::string(argv[i]) == "--port") port = static_cast<uint16_t>(std::atoi(argv[i + 1]));
        if (std::string(argv[i]) == "--frames") maxFrames = std::atol(argv[i + 1]);
    }

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { std::perror("socket"); return 1; }
    fcntl(sock, F_SETFL, O_NONBLOCK);
    sockaddr_in bindAddr{};
    bindAddr.sin_family = AF_INET;
    bindAddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    bindAddr.sin_port = htons(port);
    if (bind(sock, reinterpret_cast<sockaddr*>(&bindAddr), sizeof(bindAddr)) < 0) {
        std::perror("bind");
        return 1;
    }

    World w(2);
    buildNetScene(w);
    if (w.bodyCount() > kMaxBodies) {
        std::fprintf(stderr, "scene too large for one datagram (%d > %d)\n", w.bodyCount(), kMaxBodies);
        return 1;
    }
    std::printf("net_server: %d bodies on udp://127.0.0.1:%u (60 Hz sim, 20 Hz snapshots)\n",
                w.bodyCount(), port);

    std::vector<ClientAddr> clients;
    std::vector<uint8_t> packet(sizeof(SnapshotHeader) + sizeof(BodyState) * kMaxBodies);
    uint32_t tick = 0;
    uint32_t pokes = 0;

    using Clock = std::chrono::steady_clock;
    auto next = Clock::now();
    const auto tickDur = std::chrono::microseconds(1000000 / kTickHz);

    while (maxFrames < 0 || tick < static_cast<uint32_t>(maxFrames)) {
        // Drain incoming client messages.
        for (;;) {
            uint8_t buf[16];
            ClientAddr from{};
            socklen_t len = sizeof(from.addr);
            ssize_t n = recvfrom(sock, buf, sizeof(buf), 0,
                                 reinterpret_cast<sockaddr*>(&from.addr), &len);
            if (n <= 0) break;
            ClientMsg msg = static_cast<ClientMsg>(buf[0]);
            if (msg == ClientMsg::Hello) {
                bool known = false;
                for (const auto& c : clients) known = known || (c == from);
                if (!known) {
                    clients.push_back(from);
                    std::printf("client joined (%zu total)\n", clients.size());
                }
            } else if (msg == ClientMsg::Poke) {
                // deterministic "random" pick so repeated runs behave the same
                BodyId id = 1 + static_cast<BodyId>((pokes * 7) % (w.bodyCount() - 1));
                w.applyImpulse(id, {0, 6.0f + (pokes % 3), 0});
                ++pokes;
            }
        }

        w.step(1.0f / kTickHz);
        ++tick;

        if (tick % kSnapshotEvery == 0 && !clients.empty()) {
            auto* hdr = reinterpret_cast<SnapshotHeader*>(packet.data());
            hdr->magic = kMagic;
            hdr->tick = tick;
            hdr->bodyCount = static_cast<uint16_t>(w.bodyCount());
            auto* states = reinterpret_cast<BodyState*>(packet.data() + sizeof(SnapshotHeader));
            for (int i = 0; i < w.bodyCount(); ++i) {
                const RigidBody& b = w.body(i);
                states[i] = {b.xf.position.x, b.xf.position.y, b.xf.position.z,
                             b.xf.orientation.x, b.xf.orientation.y, b.xf.orientation.z,
                             b.xf.orientation.w, static_cast<uint8_t>(b.asleep ? 1 : 0)};
            }
            size_t size = sizeof(SnapshotHeader) + sizeof(BodyState) * w.bodyCount();
            for (const auto& c : clients)
                sendto(sock, packet.data(), size, 0,
                       reinterpret_cast<const sockaddr*>(&c.addr), sizeof(c.addr));
        }

        if (tick % 600 == 0)
            std::printf("tick %u  awake %d  step %.2f ms  clients %zu\n",
                        tick, w.awakeCount(), w.profile().total(), clients.size());

        next += tickDur;
        std::this_thread::sleep_until(next);
    }

    std::printf("net_server: done after %u ticks\n", tick);
    close(sock);
    return 0;
}
