#include "world.h"
#include "../collision/collide.h"
#include "../parallel/job_system.h"
#include <algorithm>
#include <chrono>
#include <cstring>
#include <numeric>

namespace mc {

namespace {
constexpr float kAabbMargin = 0.04f;
constexpr float kSleepLinearTol = 0.08f;  // m/s
constexpr float kSleepAngularTol = 0.10f; // rad/s
constexpr float kSleepTime = 0.5f;        // s below tolerance before sleeping
constexpr float kMatchTolSq = 0.02f * 0.02f; // warm-start anchor match distance
} // namespace

World::World(unsigned threads) {
    if (threads > 1) jobs_ = std::make_unique<JobSystem>(threads);
}

World::~World() = default;

BodyId World::addBody(const RigidBody& body) {
    bodies_.push_back(body);
    bodies_.back().updateWorldInertia();
    return static_cast<BodyId>(bodies_.size() - 1);
}

int World::addJoint(const Joint& joint) {
    joints_.push_back(joint);
    return static_cast<int>(joints_.size() - 1);
}

void World::wake(BodyId id) {
    RigidBody& b = bodies_[id];
    if (b.isStatic()) return;
    b.asleep = false;
    b.lowVelocityTime = 0.0f;
}

void World::applyImpulse(BodyId id, const Vec3& impulse) {
    wake(id);
    bodies_[id].linearVelocity += impulse * bodies_[id].invMass;
}

int World::awakeCount() const {
    int n = 0;
    for (const auto& b : bodies_)
        if (!b.isStatic() && !b.asleep) ++n;
    return n;
}

void World::integrateVelocities(float dt) {
    float linDamp = 1.0f / (1.0f + dt * linearDamping);
    float angDamp = 1.0f / (1.0f + dt * angularDamping);
    for (auto& b : bodies_) {
        if (b.isStatic() || b.asleep) continue;
        b.linearVelocity += (gravity + b.force * b.invMass) * dt;
        b.angularVelocity += (b.invInertiaWorld * b.torque) * dt;
        b.linearVelocity *= linDamp;
        b.angularVelocity *= angDamp;
        b.force = {};
        b.torque = {};
        b.updateWorldInertia();
    }
}

void World::findPairs(std::vector<PairKey>& pairs) {
    const int n = static_cast<int>(bodies_.size());
    aabbs_.resize(n);
    for (int i = 0; i < n; ++i) {
        aabbs_[i] = bodies_[i].shape.computeAABB(bodies_[i].xf);
        Vec3 mgn{kAabbMargin, kAabbMargin, kAabbMargin};
        aabbs_[i].min -= mgn;
        aabbs_[i].max += mgn;
    }

    // Sweep and prune along x: sort by min.x, then only test while intervals overlap.
    std::vector<int> order(n);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        return aabbs_[a].min.x < aabbs_[b].min.x || (aabbs_[a].min.x == aabbs_[b].min.x && a < b);
    });

    pairs.clear();
    for (int ii = 0; ii < n; ++ii) {
        int i = order[ii];
        for (int jj = ii + 1; jj < n; ++jj) {
            int j = order[jj];
            if (aabbs_[j].min.x > aabbs_[i].max.x) break;
            if (bodies_[i].isStatic() && bodies_[j].isStatic()) continue;
            if (aabbs_[i].min.y > aabbs_[j].max.y || aabbs_[i].max.y < aabbs_[j].min.y) continue;
            if (aabbs_[i].min.z > aabbs_[j].max.z || aabbs_[i].max.z < aabbs_[j].min.z) continue;
            pairs.push_back({std::min(i, j), std::max(i, j)});
        }
    }
    // Canonical order so downstream processing (and thus results) never depends
    // on the sort permutation.
    std::sort(pairs.begin(), pairs.end(), [](const PairKey& x, const PairKey& y) {
        return x.a < y.a || (x.a == y.a && x.b < y.b);
    });
    lastPairCount_ = static_cast<int>(pairs.size());
}

void World::narrowphase(const std::vector<PairKey>& pairs) {
    const int n = static_cast<int>(pairs.size());
    freshBuffer_.assign(n, Manifold{});
    std::vector<Manifold>& fresh = freshBuffer_;

    auto processRange = [&](int begin, int end) {
        for (int k = begin; k < end; ++k) {
            const PairKey& pk = pairs[k];
            RigidBody& A = bodies_[pk.a];
            RigidBody& B = bodies_[pk.b];
            Manifold& m = fresh[k];

            // Both asleep: bodies haven't moved, reuse last step's manifold.
            if (A.asleep && B.asleep) {
                auto it = prevIndex_.find(pk);
                if (it != prevIndex_.end()) m = prevManifolds_[it->second];
                continue;
            }

            if (collideShapes(A, B, m) == 0) { m.count = 0; continue; }
            m.a = pk.a;
            m.b = pk.b;
            m.friction = std::sqrt(A.friction * B.friction);
            m.restitution = std::max(A.restitution, B.restitution);
            for (int i = 0; i < m.count; ++i) {
                m.points[i].localA = A.xf.toLocal(m.points[i].position);
                m.points[i].localB = B.xf.toLocal(m.points[i].position);
            }

            // Warm-start matching: carry accumulated impulses from last step's
            // point with the nearest anchor on body B.
            auto it = prevIndex_.find(pk);
            if (it != prevIndex_.end()) {
                const Manifold& old = prevManifolds_[it->second];
                for (int i = 0; i < m.count; ++i) {
                    for (int o = 0; o < old.count; ++o) {
                        if (lengthSq(m.points[i].localB - old.points[o].localB) < kMatchTolSq) {
                            m.points[i].normalImpulse = old.points[o].normalImpulse;
                            m.points[i].tangentImpulse[0] = old.points[o].tangentImpulse[0];
                            m.points[i].tangentImpulse[1] = old.points[o].tangentImpulse[1];
                            break;
                        }
                    }
                }
            }
        }
    };

    if (jobs_ && n > 32)
        jobs_->parallelFor(n, 16, processRange);
    else
        processRange(0, n);

    // Keep only actual contacts; wake bodies whose persistent contact vanished
    // (e.g. their support was pulled away while they slept).
    manifolds_.clear();
    std::unordered_map<PairKey, int, PairKeyHash> newIndex;
    for (int k = 0; k < n; ++k) {
        if (fresh[k].count > 0) {
            newIndex.emplace(pairs[k], static_cast<int>(manifolds_.size()));
            manifolds_.push_back(fresh[k]);
        }
    }
    for (const auto& [key, idx] : prevIndex_) {
        if (!newIndex.count(key)) {
            wake(key.a);
            wake(key.b);
        }
    }
    prevIndex_ = std::move(newIndex);
}

void World::buildIslandsAndSolve(float dt) {
    const int n = static_cast<int>(bodies_.size());

    // Union-find over dynamic bodies; static bodies never merge islands.
    std::vector<int> parent(n);
    std::iota(parent.begin(), parent.end(), 0);
    auto find = [&parent](int x) {
        while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
        return x;
    };
    auto unite = [&](int a, int b) {
        if (bodies_[a].isStatic() || bodies_[b].isStatic()) return;
        int ra = find(a), rb = find(b);
        if (ra != rb) parent[std::max(ra, rb)] = std::min(ra, rb);
    };
    for (const auto& m : manifolds_) unite(m.a, m.b);
    for (const auto& j : joints_) unite(j.a, j.b);

    // Group constraints per island (root = the dynamic body's root).
    struct Island {
        std::vector<int> manifoldIdx;
        std::vector<int> jointIdx;
        std::vector<int> members;
        bool awake = false;
    };
    std::unordered_map<int, int> rootToIsland;
    std::vector<Island> islands;
    auto islandOf = [&](int bodyId) -> Island& {
        int root = find(bodyId);
        auto it = rootToIsland.find(root);
        if (it == rootToIsland.end()) {
            it = rootToIsland.emplace(root, static_cast<int>(islands.size())).first;
            islands.emplace_back();
        }
        return islands[it->second];
    };

    for (int i = 0; i < n; ++i) {
        if (bodies_[i].isStatic()) continue;
        Island& isl = islandOf(i);
        isl.members.push_back(i);
        if (!bodies_[i].asleep) isl.awake = true;
    }
    for (int k = 0; k < static_cast<int>(manifolds_.size()); ++k) {
        int dyn = bodies_[manifolds_[k].a].isStatic() ? manifolds_[k].b : manifolds_[k].a;
        islandOf(dyn).manifoldIdx.push_back(k);
    }
    for (int k = 0; k < static_cast<int>(joints_.size()); ++k) {
        int dyn = bodies_[joints_[k].a].isStatic() ? joints_[k].b : joints_[k].a;
        islandOf(dyn).jointIdx.push_back(k);
    }

    // Wake propagation: one awake member wakes the whole island.
    for (auto& isl : islands) {
        if (!isl.awake) continue;
        for (int id : isl.members) {
            if (bodies_[id].asleep) {
                bodies_[id].asleep = false;
                bodies_[id].lowVelocityTime = 0.0f;
                bodies_[id].updateWorldInertia();
            }
        }
    }

    float invDt = dt > 0 ? 1.0f / dt : 0.0f;
    RigidBody* bodyArr = bodies_.data();
    auto solveIsland = [&](int begin, int end) {
        for (int ii = begin; ii < end; ++ii) {
            Island& isl = islands[ii];
            if (!isl.awake) continue;
            for (int k : isl.manifoldIdx)
                contact_solver::prestep(bodyArr, manifolds_[k], invDt, solverConfig);
            for (int k : isl.jointIdx)
                joint_solver::prestep(bodyArr, joints_[k], invDt, solverConfig.baumgarte);
            for (int k : isl.manifoldIdx)
                contact_solver::warmStart(bodyArr, manifolds_[k]);
            for (int k : isl.jointIdx)
                joint_solver::warmStart(bodyArr, joints_[k]);
            for (int iter = 0; iter < solverConfig.velocityIterations; ++iter) {
                for (int k : isl.jointIdx) joint_solver::iterate(bodyArr, joints_[k]);
                for (int k : isl.manifoldIdx) contact_solver::iterate(bodyArr, manifolds_[k]);
            }
        }
    };

    int ni = static_cast<int>(islands.size());
    if (jobs_ && ni > 1)
        jobs_->parallelFor(ni, 1, solveIsland);
    else
        solveIsland(0, ni);
}

void World::integratePositions(float dt) {
    for (auto& b : bodies_) {
        if (b.isStatic() || b.asleep) continue;
        b.xf.position += b.linearVelocity * dt;
        b.xf.orientation = integrateOrientation(b.xf.orientation, b.angularVelocity, dt);
        b.updateWorldInertia();
    }
}

void World::updateSleeping(float dt) {
    if (!enableSleeping) return;

    for (auto& b : bodies_) {
        if (b.isStatic() || b.asleep) continue;
        bool slow = lengthSq(b.linearVelocity) < kSleepLinearTol * kSleepLinearTol &&
                    lengthSq(b.angularVelocity) < kSleepAngularTol * kSleepAngularTol;
        b.lowVelocityTime = slow ? b.lowVelocityTime + dt : 0.0f;
    }

    // An island sleeps only as a whole (re-derive islands cheaply via contacts).
    const int n = static_cast<int>(bodies_.size());
    std::vector<int> parent(n);
    std::iota(parent.begin(), parent.end(), 0);
    auto find = [&parent](int x) {
        while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
        return x;
    };
    auto unite = [&](int a, int b) {
        if (bodies_[a].isStatic() || bodies_[b].isStatic()) return;
        int ra = find(a), rb = find(b);
        if (ra != rb) parent[std::max(ra, rb)] = std::min(ra, rb);
    };
    for (const auto& m : manifolds_) unite(m.a, m.b);
    for (const auto& j : joints_) unite(j.a, j.b);

    std::unordered_map<int, bool> islandCanSleep;
    for (int i = 0; i < n; ++i) {
        if (bodies_[i].isStatic() || bodies_[i].asleep) continue;
        int root = find(i);
        auto [it, inserted] = islandCanSleep.emplace(root, true);
        if (bodies_[i].lowVelocityTime < kSleepTime) it->second = false;
    }
    for (int i = 0; i < n; ++i) {
        if (bodies_[i].isStatic() || bodies_[i].asleep) continue;
        auto it = islandCanSleep.find(find(i));
        if (it != islandCanSleep.end() && it->second) {
            bodies_[i].asleep = true;
            bodies_[i].linearVelocity = {};
            bodies_[i].angularVelocity = {};
        }
    }
}

void World::step(float dt) {
    using Clock = std::chrono::steady_clock;
    auto mark = [t = Clock::now()](double& out) mutable {
        auto now = Clock::now();
        out = std::chrono::duration<double, std::milli>(now - t).count();
        t = now;
    };

    integrateVelocities(dt);
    mark(profile_.integrateVel);

    std::vector<PairKey> pairs;
    findPairs(pairs);
    mark(profile_.broadphase);

    prevManifolds_ = std::move(manifolds_);
    narrowphase(pairs);
    mark(profile_.narrowphase);

    buildIslandsAndSolve(dt);
    mark(profile_.solve);

    integratePositions(dt);
    mark(profile_.integratePos);

    updateSleeping(dt);
    mark(profile_.sleep);
}

uint64_t World::stateHash() const {
    uint64_t h = 1469598103934665603ull; // FNV-1a
    auto mix = [&h](const void* data, size_t len) {
        const unsigned char* p = static_cast<const unsigned char*>(data);
        for (size_t i = 0; i < len; ++i) {
            h ^= p[i];
            h *= 1099511628211ull;
        }
    };
    for (const auto& b : bodies_) {
        mix(&b.xf.position, sizeof(Vec3));
        mix(&b.xf.orientation, sizeof(Quat));
        mix(&b.linearVelocity, sizeof(Vec3));
        mix(&b.angularVelocity, sizeof(Vec3));
    }
    return h;
}

} // namespace mc
