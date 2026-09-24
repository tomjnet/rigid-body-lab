// Two benchmarks:
//  1. Engine multithreading: a ~1300-body pile stepped with 1 thread vs all
//     cores (islands + narrowphase parallelism).
//  2. SIMD solver kernel: the sequential-impulse normal-row update over
//     independent contact rows, scalar vs AVX2 8-wide SoA. Independent rows are
//     what a production engine gets after graph-coloring contacts so no two
//     rows in a batch touch the same body.
#include "core/world.h"
#include <chrono>
#include <cstdio>
#include <thread>
#include <vector>
#include <cmath>
#include <immintrin.h>

using namespace mc;
using Clock = std::chrono::steady_clock;

static double msSince(Clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

// ---------------------------------------------------------------------------
// 1. Engine scene benchmark
// ---------------------------------------------------------------------------

static void buildPile(World& w, int layers) {
    w.addBody(RigidBody::makeStatic(Shape::box({40, 0.5f, 40}), {0, -0.5f, 0}));
    int n = 0;
    for (int layer = 0; layer < layers; ++layer) {
        for (int i = 0; i < 12; ++i) {
            for (int j = 0; j < 12; ++j) {
                // Separated columns => many small islands, the parallel-friendly case.
                Vec3 pos{i * 2.2f - 13.0f, 0.5f + layer * 1.05f, j * 2.2f - 13.0f};
                if (n % 3 == 0)
                    w.addBody(RigidBody::makeDynamic(Shape::sphere(0.4f), pos, 1.0f));
                else
                    w.addBody(RigidBody::makeDynamic(Shape::box({0.45f, 0.45f, 0.45f}), pos, 1.0f));
                ++n;
            }
        }
    }
}

static double runScene(unsigned threads, int frames, World::Profile& avg) {
    World w(threads);
    w.enableSleeping = false; // keep the workload constant for timing
    buildPile(w, 9);
    for (int i = 0; i < 30; ++i) w.step(1.0f / 60.0f); // warmup / settle contacts
    avg = {};
    auto t0 = Clock::now();
    for (int i = 0; i < frames; ++i) {
        w.step(1.0f / 60.0f);
        const auto& p = w.profile();
        avg.broadphase += p.broadphase; avg.narrowphase += p.narrowphase; avg.solve += p.solve;
        avg.integrateVel += p.integrateVel; avg.integratePos += p.integratePos; avg.sleep += p.sleep;
    }
    return msSince(t0) / frames;
}

static void printProfile(const World::Profile& p, int frames) {
    std::printf("             broad %5.2f | narrow %5.2f | solve %5.2f | integrate %5.2f ms\n",
                p.broadphase / frames, p.narrowphase / frames, p.solve / frames,
                (p.integrateVel + p.integratePos) / frames);
}

static void benchScene() {
    unsigned hw = std::thread::hardware_concurrency();
    const int frames = 60;
    std::printf("== Engine scene: %d bodies, %d frames ==\n", 12 * 12 * 9 + 1, frames);
    World::Profile p1, pn;
    double t1 = runScene(1, frames, p1);
    std::printf("  1 thread : %7.2f ms/step\n", t1);
    printProfile(p1, frames);
    double tn = runScene(hw, frames, pn);
    std::printf("  %u threads: %7.2f ms/step   speedup %.2fx\n", hw, tn, t1 / tn);
    printProfile(pn, frames);
}

// ---------------------------------------------------------------------------
// 2. SIMD contact-row kernel
// ---------------------------------------------------------------------------

struct Rows { // SoA: one entry per contact row, bodies not shared between rows
    std::vector<float> nx, ny, nz;          // contact normal
    std::vector<float> raxn_x, raxn_y, raxn_z, rbxn_x, rbxn_y, rbxn_z; // rA x n, rB x n
    std::vector<float> vax, vay, vaz, wax, way, waz;                    // body A vel
    std::vector<float> vbx, vby, vbz, wbx, wby, wbz;                    // body B vel
    std::vector<float> invMassA, invMassB, invInertiaA, invInertiaB;    // scalar inertia approx
    std::vector<float> mass, bias, lambda;                              // effective mass, bias, accumulated

    explicit Rows(int n) {
        for (auto* v : {&nx, &ny, &nz, &raxn_x, &raxn_y, &raxn_z, &rbxn_x, &rbxn_y, &rbxn_z,
                        &vax, &vay, &vaz, &wax, &way, &waz, &vbx, &vby, &vbz, &wbx, &wby, &wbz,
                        &invMassA, &invMassB, &invInertiaA, &invInertiaB, &mass, &bias, &lambda})
            v->assign(n, 0.0f);
    }
};

static Rows makeRows(int n) {
    Rows r(n);
    uint32_t s = 12345;
    auto frand = [&s](float lo, float hi) {
        s = s * 1664525u + 1013904223u;
        return lo + (hi - lo) * (float)(s >> 8) / (float)(1 << 24);
    };
    for (int i = 0; i < n; ++i) {
        Vec3 nrm = normalize(Vec3{frand(-1, 1), frand(-1, 1) + 1.5f, frand(-1, 1)});
        Vec3 ra{frand(-0.5f, 0.5f), frand(-0.5f, 0.5f), frand(-0.5f, 0.5f)};
        Vec3 rb{frand(-0.5f, 0.5f), frand(-0.5f, 0.5f), frand(-0.5f, 0.5f)};
        Vec3 raxn = cross(ra, nrm), rbxn = cross(rb, nrm);
        r.nx[i] = nrm.x; r.ny[i] = nrm.y; r.nz[i] = nrm.z;
        r.raxn_x[i] = raxn.x; r.raxn_y[i] = raxn.y; r.raxn_z[i] = raxn.z;
        r.rbxn_x[i] = rbxn.x; r.rbxn_y[i] = rbxn.y; r.rbxn_z[i] = rbxn.z;
        r.vax[i] = frand(-2, 2); r.vay[i] = frand(-2, 2); r.vaz[i] = frand(-2, 2);
        r.wax[i] = frand(-2, 2); r.way[i] = frand(-2, 2); r.waz[i] = frand(-2, 2);
        r.vbx[i] = frand(-2, 2); r.vby[i] = frand(-2, 2); r.vbz[i] = frand(-2, 2);
        r.wbx[i] = frand(-2, 2); r.wby[i] = frand(-2, 2); r.wbz[i] = frand(-2, 2);
        r.invMassA[i] = frand(0.5f, 2); r.invMassB[i] = frand(0.5f, 2);
        r.invInertiaA[i] = frand(0.5f, 3); r.invInertiaB[i] = frand(0.5f, 3);
        float k = r.invMassA[i] + r.invMassB[i] +
                  r.invInertiaA[i] * lengthSq(raxn) + r.invInertiaB[i] * lengthSq(rbxn);
        r.mass[i] = 1.0f / k;
        r.bias[i] = frand(0, 0.5f);
    }
    return r;
}

static void solveScalarRange(Rows& r, int begin, int end, int iters) {
    for (int it = 0; it < iters; ++it) {
        for (int i = begin; i < end; ++i) {
            float vn = r.nx[i] * (r.vbx[i] - r.vax[i]) + r.ny[i] * (r.vby[i] - r.vay[i]) +
                       r.nz[i] * (r.vbz[i] - r.vaz[i]) +
                       r.rbxn_x[i] * r.wbx[i] + r.rbxn_y[i] * r.wby[i] + r.rbxn_z[i] * r.wbz[i] -
                       (r.raxn_x[i] * r.wax[i] + r.raxn_y[i] * r.way[i] + r.raxn_z[i] * r.waz[i]);
            float dl = r.mass[i] * (-vn + r.bias[i]);
            float nl = r.lambda[i] + dl;
            nl = nl > 0.0f ? nl : 0.0f;
            dl = nl - r.lambda[i];
            r.lambda[i] = nl;

            float ia = r.invMassA[i] * dl, ib = r.invMassB[i] * dl;
            r.vax[i] -= r.nx[i] * ia; r.vay[i] -= r.ny[i] * ia; r.vaz[i] -= r.nz[i] * ia;
            r.vbx[i] += r.nx[i] * ib; r.vby[i] += r.ny[i] * ib; r.vbz[i] += r.nz[i] * ib;
            float ja = r.invInertiaA[i] * dl, jb = r.invInertiaB[i] * dl;
            r.wax[i] -= r.raxn_x[i] * ja; r.way[i] -= r.raxn_y[i] * ja; r.waz[i] -= r.raxn_z[i] * ja;
            r.wbx[i] += r.rbxn_x[i] * jb; r.wby[i] += r.rbxn_y[i] * jb; r.wbz[i] += r.rbxn_z[i] * jb;
        }
    }
}

#ifdef __AVX2__
static void solveAvx2(Rows& r, int n, int iters) {
    const __m256 zero = _mm256_setzero_ps();
    for (int it = 0; it < iters; ++it) {
        int i = 0;
        for (; i + 8 <= n; i += 8) {
            auto L = [&](const std::vector<float>& v) { return _mm256_loadu_ps(&v[i]); };
            __m256 nx = L(r.nx), ny = L(r.ny), nz = L(r.nz);
            __m256 vax = L(r.vax), vay = L(r.vay), vaz = L(r.vaz);
            __m256 vbx = L(r.vbx), vby = L(r.vby), vbz = L(r.vbz);
            __m256 wax = L(r.wax), way = L(r.way), waz = L(r.waz);
            __m256 wbx = L(r.wbx), wby = L(r.wby), wbz = L(r.wbz);
            __m256 rax = L(r.raxn_x), ray = L(r.raxn_y), raz = L(r.raxn_z);
            __m256 rbx = L(r.rbxn_x), rby = L(r.rbxn_y), rbz = L(r.rbxn_z);

            // vn = n.(vb-va) + rbxn.wb - raxn.wa
            __m256 vn = _mm256_mul_ps(nx, _mm256_sub_ps(vbx, vax));
            vn = _mm256_fmadd_ps(ny, _mm256_sub_ps(vby, vay), vn);
            vn = _mm256_fmadd_ps(nz, _mm256_sub_ps(vbz, vaz), vn);
            vn = _mm256_fmadd_ps(rbx, wbx, vn);
            vn = _mm256_fmadd_ps(rby, wby, vn);
            vn = _mm256_fmadd_ps(rbz, wbz, vn);
            vn = _mm256_fnmadd_ps(rax, wax, vn);
            vn = _mm256_fnmadd_ps(ray, way, vn);
            vn = _mm256_fnmadd_ps(raz, waz, vn);

            __m256 lam = L(r.lambda);
            __m256 dl = _mm256_mul_ps(L(r.mass), _mm256_sub_ps(L(r.bias), vn));
            __m256 nl = _mm256_max_ps(_mm256_add_ps(lam, dl), zero);
            dl = _mm256_sub_ps(nl, lam);
            _mm256_storeu_ps(&r.lambda[i], nl);

            __m256 ia = _mm256_mul_ps(L(r.invMassA), dl);
            __m256 ib = _mm256_mul_ps(L(r.invMassB), dl);
            _mm256_storeu_ps(&r.vax[i], _mm256_fnmadd_ps(nx, ia, vax));
            _mm256_storeu_ps(&r.vay[i], _mm256_fnmadd_ps(ny, ia, vay));
            _mm256_storeu_ps(&r.vaz[i], _mm256_fnmadd_ps(nz, ia, vaz));
            _mm256_storeu_ps(&r.vbx[i], _mm256_fmadd_ps(nx, ib, vbx));
            _mm256_storeu_ps(&r.vby[i], _mm256_fmadd_ps(ny, ib, vby));
            _mm256_storeu_ps(&r.vbz[i], _mm256_fmadd_ps(nz, ib, vbz));
            __m256 ja = _mm256_mul_ps(L(r.invInertiaA), dl);
            __m256 jb = _mm256_mul_ps(L(r.invInertiaB), dl);
            _mm256_storeu_ps(&r.wax[i], _mm256_fnmadd_ps(rax, ja, wax));
            _mm256_storeu_ps(&r.way[i], _mm256_fnmadd_ps(ray, ja, way));
            _mm256_storeu_ps(&r.waz[i], _mm256_fnmadd_ps(raz, ja, waz));
            _mm256_storeu_ps(&r.wbx[i], _mm256_fmadd_ps(rbx, jb, wbx));
            _mm256_storeu_ps(&r.wby[i], _mm256_fmadd_ps(rby, jb, wby));
            _mm256_storeu_ps(&r.wbz[i], _mm256_fmadd_ps(rbz, jb, wbz));
        }
        if (i < n) solveScalarRange(r, i, n, 1); // remainder rows
    }
}
#endif

static void benchKernel() {
    const int n = 8192, iters = 400;
    std::printf("== SIMD solver kernel: %d rows x %d iterations ==\n", n, iters);

    Rows a = makeRows(n);
    auto t0 = Clock::now();
    solveScalarRange(a, 0, n, iters);
    double ts = msSince(t0);
    std::printf("  scalar : %7.2f ms\n", ts);

#ifdef __AVX2__
    if (__builtin_cpu_supports("avx2")) {
        Rows b = makeRows(n);
        t0 = Clock::now();
        solveAvx2(b, n, iters);
        double tv = msSince(t0);
        float maxDiff = 0;
        for (int i = 0; i < n; ++i)
            maxDiff = std::max(maxDiff, std::fabs(a.lambda[i] - b.lambda[i]));
        std::printf("  AVX2   : %7.2f ms   speedup %.2fx   max |dLambda| vs scalar: %.2e\n",
                    tv, ts / tv, maxDiff);
    } else {
        std::printf("  AVX2   : CPU does not support AVX2, skipped\n");
    }
#else
    std::printf("  AVX2   : not compiled in\n");
#endif
}

int main() {
    benchKernel();
    benchScene();
    return 0;
}
