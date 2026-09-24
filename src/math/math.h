#pragma once
// Minimal 3D math for rigid body simulation: Vec3, Mat3, Quat, Transform.
// Written from scratch (no GLM) so every operation used by the solver is
// visible and discussable: inertia tensor rotation, quaternion integration, etc.

#include <cmath>
#include <cstdint>
#include <algorithm>

namespace mc {

constexpr float kPi = 3.14159265358979323846f;

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

struct Vec3 {
    float x = 0, y = 0, z = 0;

    constexpr Vec3() = default;
    constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(float s) const { return {x / s, y / s, z / s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }

    float operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
    float& operator[](int i) { return i == 0 ? x : (i == 1 ? y : z); }
};

inline Vec3 operator*(float s, const Vec3& v) { return v * s; }
inline float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline float lengthSq(const Vec3& v) { return dot(v, v); }
inline float length(const Vec3& v) { return std::sqrt(dot(v, v)); }
inline Vec3 normalize(const Vec3& v) {
    float len = length(v);
    return len > 1e-9f ? v / len : Vec3{0, 0, 0};
}
inline Vec3 vmin(const Vec3& a, const Vec3& b) { return {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)}; }
inline Vec3 vmax(const Vec3& a, const Vec3& b) { return {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)}; }
inline Vec3 vabs(const Vec3& v) { return {std::fabs(v.x), std::fabs(v.y), std::fabs(v.z)}; }

// Builds an orthonormal basis (t1, t2) perpendicular to unit vector n.
// Used to generate friction tangent directions from a contact normal.
inline void basisFromNormal(const Vec3& n, Vec3& t1, Vec3& t2) {
    if (std::fabs(n.x) >= 0.57735f)
        t1 = normalize(Vec3{n.y, -n.x, 0.0f});
    else
        t1 = normalize(Vec3{0.0f, n.z, -n.y});
    t2 = cross(n, t1);
}

// Column-major 3x3 matrix: c[i] is column i.
struct Mat3 {
    Vec3 c[3];

    Mat3() : c{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}} {}
    Mat3(const Vec3& c0, const Vec3& c1, const Vec3& c2) : c{c0, c1, c2} {}

    static Mat3 identity() { return Mat3{}; }
    static Mat3 diagonal(float a, float b, float d) { return {{a, 0, 0}, {0, b, 0}, {0, 0, d}}; }
    static Mat3 zero() { return {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}; }

    // Skew-symmetric cross-product matrix: skew(v) * u == cross(v, u)
    static Mat3 skew(const Vec3& v) {
        return {{0, v.z, -v.y}, {-v.z, 0, v.x}, {v.y, -v.x, 0}};
    }

    Vec3 operator*(const Vec3& v) const { return c[0] * v.x + c[1] * v.y + c[2] * v.z; }
    Mat3 operator*(const Mat3& m) const { return {(*this) * m.c[0], (*this) * m.c[1], (*this) * m.c[2]}; }
    Mat3 operator+(const Mat3& m) const { return {c[0] + m.c[0], c[1] + m.c[1], c[2] + m.c[2]}; }
    Mat3 operator-(const Mat3& m) const { return {c[0] - m.c[0], c[1] - m.c[1], c[2] - m.c[2]}; }
    Mat3 operator*(float s) const { return {c[0] * s, c[1] * s, c[2] * s}; }

    Mat3 transposed() const {
        return {{c[0].x, c[1].x, c[2].x},
                {c[0].y, c[1].y, c[2].y},
                {c[0].z, c[1].z, c[2].z}};
    }

    // Row i (handy for SAT tests where we need the basis vectors of A^T B).
    Vec3 row(int i) const { return {c[0][i], c[1][i], c[2][i]}; }

    float determinant() const { return dot(c[0], cross(c[1], c[2])); }

    Mat3 inverse() const {
        Vec3 r0 = cross(c[1], c[2]);
        Vec3 r1 = cross(c[2], c[0]);
        Vec3 r2 = cross(c[0], c[1]);
        float det = dot(c[0], r0);
        if (std::fabs(det) < 1e-12f) return Mat3::zero();
        float inv = 1.0f / det;
        // rows of the inverse are the scaled reciprocal vectors
        return Mat3{{r0.x * inv, r1.x * inv, r2.x * inv},
                    {r0.y * inv, r1.y * inv, r2.y * inv},
                    {r0.z * inv, r1.z * inv, r2.z * inv}};
    }
};

struct Quat {
    // q = w + xi + yj + zk
    float x = 0, y = 0, z = 0, w = 1;

    Quat() = default;
    Quat(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}

    static Quat identity() { return {}; }

    static Quat fromAxisAngle(const Vec3& axis, float angle) {
        Vec3 a = normalize(axis);
        float s = std::sin(angle * 0.5f);
        return {a.x * s, a.y * s, a.z * s, std::cos(angle * 0.5f)};
    }

    Quat operator*(const Quat& q) const {
        return {w * q.x + x * q.w + y * q.z - z * q.y,
                w * q.y - x * q.z + y * q.w + z * q.x,
                w * q.z + x * q.y - y * q.x + z * q.w,
                w * q.w - x * q.x - y * q.y - z * q.z};
    }
    Quat operator+(const Quat& q) const { return {x + q.x, y + q.y, z + q.z, w + q.w}; }
    Quat operator*(float s) const { return {x * s, y * s, z * s, w * s}; }

    Quat normalized() const {
        float n = std::sqrt(x * x + y * y + z * z + w * w);
        if (n < 1e-12f) return identity();
        float inv = 1.0f / n;
        return {x * inv, y * inv, z * inv, w * inv};
    }

    Mat3 toMat3() const {
        float xx = x * x, yy = y * y, zz = z * z;
        float xy = x * y, xz = x * z, yz = y * z;
        float wx = w * x, wy = w * y, wz = w * z;
        return {{1 - 2 * (yy + zz), 2 * (xy + wz), 2 * (xz - wy)},
                {2 * (xy - wz), 1 - 2 * (xx + zz), 2 * (yz + wx)},
                {2 * (xz + wy), 2 * (yz - wx), 1 - 2 * (xx + yy)}};
    }
};

inline Vec3 rotate(const Quat& q, const Vec3& v) {
    // v' = v + 2*qv x (qv x v + w*v)   (fewer ops than building the matrix)
    Vec3 qv{q.x, q.y, q.z};
    Vec3 t = cross(qv, v) * 2.0f;
    return v + t * q.w + cross(qv, t);
}
inline Vec3 rotateInv(const Quat& q, const Vec3& v) {
    return rotate(Quat{-q.x, -q.y, -q.z, q.w}, v);
}

// Semi-implicit integration of orientation: dq/dt = 0.5 * (w_quat) * q
inline Quat integrateOrientation(const Quat& q, const Vec3& omega, float dt) {
    Quat wq{omega.x, omega.y, omega.z, 0.0f};
    Quat dq = (wq * q) * (0.5f * dt);
    return (q + dq).normalized();
}

struct Transform {
    Vec3 position;
    Quat orientation;

    Vec3 toWorld(const Vec3& local) const { return rotate(orientation, local) + position; }
    Vec3 toLocal(const Vec3& world) const { return rotateInv(orientation, world - position); }
};

struct AABB {
    Vec3 min, max;
    bool overlaps(const AABB& o) const {
        return min.x <= o.max.x && max.x >= o.min.x &&
               min.y <= o.max.y && max.y >= o.min.y &&
               min.z <= o.max.z && max.z >= o.min.z;
    }
};

} // namespace mc
