# Core theory reading list — real-time rigid body physics engines

A map of the theory behind real-time physics engines, assembled while building
minicollide to understand the field from first principles. Tier 1 is the canon
behind every production engine (including the sequential-impulse / PGS family
most engines use); Tier 2 is modern and specialized work for going deeper.
Each entry notes *why it matters* and where it shows up in this repo's code.

---

## Tier 1 — the canon (read these first)

### Baraff (1989) — *Analytical Methods for Dynamic Simulation of Non-penetrating Rigid Bodies* (SIGGRAPH)
The founding formulation of contact as a constrained-dynamics problem: contact
forces computed so bodies do not interpenetrate, with complementarity
(force ≥ 0 ⟂ separation ≥ 0). Everything since is a refinement of this framing.
Baraff's papers and the excellent *Physically Based Modeling* course notes are at
https://www.cs.cmu.edu/~baraff/papers/ and https://graphics.pixar.com/pbm2001/
**In this repo:** the complementarity condition is the clamp `Pn >= 0` in
`src/solver/contact_solver.cpp`.

### Baraff (1994) — *Fast Contact Force Computation for Nonpenetrating Rigid Bodies* (SIGGRAPH)
Poses multi-contact force computation as an LCP (linear complementarity
problem) and gives a practical pivoting solver. Worth understanding why
game engines abandoned direct LCP solvers for iterative ones (cost, warm
starting, graceful degradation under a fixed time budget).

### Stewart & Trinkle (1996) — *An Implicit Time-Stepping Scheme for Rigid Body Dynamics with Inelastic Collisions and Coulomb Friction* (IJNME)
The velocity-level time-stepping formulation: instead of resolving each
collision event, integrate in fixed steps and solve for impulses satisfying
non-penetration + friction at the velocity level. This is the mathematical
skeleton of every modern engine loop, including this one.

### Anitescu & Potra (1997) — *Formulating Dynamic Multi-Rigid-Body Contact Problems with Friction as Solvable LCPs* (Nonlinear Dynamics)
Proves the time-stepping LCP with a polyhedral friction cone is always
solvable — the theoretical guarantee that the velocity-stepping approach is
well-posed. Know the linearized friction cone vs. the box approximation used
in practice (and in `contact_solver.cpp`).

### Catto (2005) — *Iterative Dynamics with Temporal Coherence*
THE practical solver paper: sequential impulses (= projected Gauss-Seidel on
the contact LCP) with **accumulated impulses** and **warm starting**. Basis of
Box2D and the PGS family nearly every production solver belongs to. Read alongside
Catto's GDC decks (*Modeling and Solving Constraints*, *Soft Constraints* —
https://box2d.org/publications/), which cover Baumgarte stabilization, slop,
and soft constraints.
**In this repo:** `src/solver/contact_solver.cpp` is a faithful 3D
implementation; `tests/test_main.cpp: testStackStability` is the payoff.

### Gilbert, Johnson & Keerthi (1988) — *A Fast Procedure for Computing the Distance Between Complex Objects in Three-Dimensional Space* (IEEE J. Robotics)
GJK: distance between convex sets using only support mappings on the Minkowski
difference. The universal narrowphase primitive.
**In this repo:** `src/collision/gjk.cpp`, used for capsule-vs-box.

### Baumgarte (1972) — *Stabilization of Constraints and Integrals of Motion in Dynamical Systems* (Comp. Methods in Applied Mechanics)
Why constraint drift happens under numerical integration and the classic fix:
feed a fraction of the position error back into the velocity constraint. Every
"beta * C / dt" term in every engine is this paper.
**In this repo:** the `baumgarte` term in `SolverConfig`, used by both
contacts and joints. Know its failure mode (energy injection) and the
alternatives: split impulse / NGS position projection.

---

## Tier 2 — modern & specialized

### Mirtich & Canny (1995) — *Impulse-Based Simulation of Rigid Bodies* (I3D)
The competing paradigm: resolve everything, including resting contact, as
micro-collisions. Largely superseded, but the contrast clarifies why
constraint-based resting contact wins for stacks.

### Guendelman, Bridson & Fedkiw (2003) — *Nonconvex Rigid Bodies with Stacking* (SIGGRAPH)
Contact ordering, shock propagation, and the position-before-velocity update
trick for stable stacking of hard cases.
https://graphics.stanford.edu/papers/rigid_bodies/

### van den Bergen (1999) — *A Fast and Robust GJK Implementation for Collision Detection of Convex Objects* (JGT)
The engineering companion to GJK: termination criteria in floating point,
caching, margins. His book (*Collision Detection in Interactive 3D
Environments*) adds EPA. Read when you want to go deeper on narrowphase
robustness.

### The position-based dynamics lineage
- Müller et al. (2007) — *Position Based Dynamics* (J. Vis. Comm.)
- Macklin et al. (2016) — *XPBD: Position-Based Simulation of Compliant Constrained Dynamics* (MIG)
- Macklin et al. (2019) — *Small Steps in Physics Simulation* (SCA) — the
  surprising result that many tiny steps with 1 solver iteration beat one big
  step with many iterations.
- Müller et al. (2020) — *Detailed Rigid Body Simulation with Extended Position Based Dynamics* (SCA)

PDFs on the authors' pages: https://matthias-research.github.io/pages/publications/publications.html
and https://mmacklin.com/publications. This lineage is the main modern
alternative to velocity-level PGS; comparing the two clarifies both (PBD:
unconditionally stable, stiffness is iteration/dt-dependent unless XPBD;
PGS: impulses are physical, warm-startable).

### Tonge, Benevolenski & Voroshilov (2012) — *Mass Splitting for Jitter-Free Parallel Rigid Body Simulation* (SIGGRAPH)
How to parallelize the *inside* of a solver island (Gauss-Seidel is inherently
sequential): split bodies across sub-solvers and average. Pairs directly with
this repo's island-level parallelism and graph-coloring discussion in the
README's SIMD section.

### Featherstone — *Rigid Body Dynamics Algorithms* (book, 2008)
Articulated-body algorithm: O(n) forward dynamics in joint (reduced)
coordinates. Know the trade-off vs. maximal coordinates + constraints (what
this repo and most game engines use): reduced coordinates never drift but make
contact and breakage harder; Featherstone shines for ragdolls/robots.

### Networked physics (industry canon rather than academia)
- Glenn Fiedler — *Networked Physics* series and *Snapshot Interpolation*:
  https://gafferongames.com/categories/networked-physics/
- Bernier (2001) — *Latency Compensating Methods in Client/Server In-game
  Protocol Design* (GDC / Valve). Summarized with diagrams at
  https://developer.valvesoftware.com/wiki/Source_Multiplayer_Networking

**In this repo:** `net/` implements the snapshot-interpolation half of this
literature (authoritative server, 20 Hz snapshots, 100 ms interpolation
buffer). Some large-scale platforms go further and *partition simulation
ownership across clients* — the trust/latency/consistency trade-offs of that
design vs. server-authoritative are a rich follow-up study.

---

## Suggested reading order

1. Catto 2005 + GDC decks (map them to `contact_solver.cpp` as you read)
2. Baumgarte 1972 (skim; internalize the stabilization idea)
3. Stewart-Trinkle 1996 → Anitescu-Potra 1997 (the formal backbone)
4. GJK 1988 + van den Bergen 1999 (trace `gjk.cpp` alongside)
5. Baraff 1989/1994 + Pixar course notes (foundations, inertia, ODEs)
6. Small Steps 2019 + XPBD 2016 (the modern alternative paradigm)
7. Tonge 2012 (parallel solver design)
8. Fiedler series + Valve wiki (networked physics vocabulary)
