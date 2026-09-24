#pragma once
// Wire protocol for the networked-physics demo (M6).
//
// Model: authoritative server steps the simulation at 60 Hz and broadcasts a
// full-state snapshot every kSnapshotEvery ticks (20 Hz). The client never
// simulates; it buffers snapshots and renders ~100 ms in the past,
// interpolating between the two snapshots that bracket the render time —
// the standard client-side interpolation scheme (Valve/Source, Gaffer On Games).
//
// The scene is sized so one snapshot fits a single UDP datagram (< 1200 B).

#include "math/math.h"
#include <cstdint>

namespace mc::net {

constexpr uint32_t kMagic = 0x6D63704Eu; // "mcpN"
constexpr uint16_t kDefaultPort = 47001;
constexpr int kTickHz = 60;
constexpr int kSnapshotEvery = 3;   // 20 snapshots/s
constexpr int kInterpTicks = 6;     // render 6 ticks (~100 ms) behind latest

enum class ClientMsg : uint8_t {
    Hello = 1, // register for snapshots (re-sent periodically as a keepalive)
    Poke = 2,  // apply an upward impulse to a random dynamic body
};

#pragma pack(push, 1)
struct BodyState {
    float px, py, pz;
    float qx, qy, qz, qw;
    uint8_t asleep;
};

struct SnapshotHeader {
    uint32_t magic;
    uint32_t tick;
    uint16_t bodyCount;
};
#pragma pack(pop)

constexpr int kMaxBodies = 36; // header + 36 * 29 B = 1054 B, fits one datagram

} // namespace mc::net
