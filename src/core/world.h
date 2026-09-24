#pragma once
// The simulation world: fixed-timestep pipeline
//   integrate velocities -> broadphase (sweep & prune) -> narrowphase (parallel)
//   -> manifold persistence/warm-start matching -> islands (union-find)
//   -> per-island sequential-impulse solve (parallel across islands)
//   -> integrate positions -> sleeping
//
// Determinism: with a fixed dt the pipeline is deterministic across runs on the
// same binary, including with threads (work is indexed by input order, islands
// are disjoint, and per-island solve order is fixed).

#include "rigid_body.h"
#include "../collision/contact.h"
#include "../solver/contact_solver.h"
#include "../solver/joints.h"
#include <memory>
#include <unordered_map>
#include <vector>

namespace mc {

class JobSystem;

class World {
public:
    Vec3 gravity{0, -9.81f, 0};
    SolverConfig solverConfig;
    bool enableSleeping = true;
    float linearDamping = 0.01f;
    float angularDamping = 0.05f;

    // threads = 1 => fully single-threaded (no pool created)
    explicit World(unsigned threads = 1);
    ~World();

    BodyId addBody(const RigidBody& body);
    int addJoint(const Joint& joint);

    RigidBody& body(BodyId id) { return bodies_[id]; }
    const RigidBody& body(BodyId id) const { return bodies_[id]; }
    int bodyCount() const { return static_cast<int>(bodies_.size()); }
    std::vector<RigidBody>& bodies() { return bodies_; }
    const std::vector<Manifold>& manifolds() const { return manifolds_; }

    void wake(BodyId id);
    void applyImpulse(BodyId id, const Vec3& impulse); // at center of mass; wakes

    void step(float dt);

    // FNV-1a over all body transforms/velocities — for determinism tests.
    uint64_t stateHash() const;

    int lastPairCount() const { return lastPairCount_; }
    int awakeCount() const;

    // Per-phase timings for the most recent step, in milliseconds.
    struct Profile {
        double integrateVel = 0, broadphase = 0, narrowphase = 0, solve = 0, integratePos = 0, sleep = 0;
        double total() const { return integrateVel + broadphase + narrowphase + solve + integratePos + sleep; }
    };
    const Profile& profile() const { return profile_; }

private:
    void integrateVelocities(float dt);
    void findPairs(std::vector<PairKey>& pairs);
    void narrowphase(const std::vector<PairKey>& pairs);
    void buildIslandsAndSolve(float dt);
    void integratePositions(float dt);
    void updateSleeping(float dt);

    std::vector<RigidBody> bodies_;
    std::vector<Joint> joints_;

    std::vector<Manifold> manifolds_;      // this step's manifolds, pair order
    std::vector<Manifold> prevManifolds_;  // last step's, for persistence
    std::unordered_map<PairKey, int, PairKeyHash> prevIndex_; // key -> prevManifolds_ index

    std::unique_ptr<JobSystem> jobs_;
    std::vector<AABB> aabbs_;
    std::vector<Manifold> freshBuffer_; // reused narrowphase scratch
    int lastPairCount_ = 0;
    Profile profile_;
};

} // namespace mc
