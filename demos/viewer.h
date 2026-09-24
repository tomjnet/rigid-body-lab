#pragma once
// Shared raylib viewer for the demos: lit meshes for boxes/spheres/capsules,
// orbital camera, and a HUD line. Sleeping bodies are drawn desaturated so the
// island-sleeping behavior is visible.
#include "core/world.h"
#include "raylib.h"
#include "raymath.h"
#include <cstdio>

namespace viewer {

struct Viewer {
    Camera3D camera{};
    Mesh cube{}, sphere{}, cylinder{};
    Material mat{};
    Shader shader{};

    void init(const char* title, mc::Vec3 camPos = {12, 9, 12}, mc::Vec3 target = {0, 2, 0}) {
        SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
        InitWindow(1280, 720, title);
        SetTargetFPS(60);

        camera.position = {camPos.x, camPos.y, camPos.z};
        camera.target = {target.x, target.y, target.z};
        camera.up = {0, 1, 0};
        camera.fovy = 45;
        camera.projection = CAMERA_PERSPECTIVE;

        cube = GenMeshCube(1, 1, 1);
        sphere = GenMeshSphere(1.0f, 16, 24);
        cylinder = GenMeshCylinder(1.0f, 1.0f, 24);

        // Minimal directional-light shader; raylib auto-binds mvp/matNormal/colDiffuse.
        const char* vs =
            "#version 330\n"
            "in vec3 vertexPosition; in vec3 vertexNormal;\n"
            "uniform mat4 mvp; uniform mat4 matNormal;\n"
            "out vec3 fragNormal;\n"
            "void main(){ fragNormal = normalize(vec3(matNormal*vec4(vertexNormal,0.0)));\n"
            "  gl_Position = mvp*vec4(vertexPosition,1.0); }";
        const char* fs =
            "#version 330\n"
            "in vec3 fragNormal; out vec4 finalColor; uniform vec4 colDiffuse;\n"
            "void main(){ vec3 l = normalize(vec3(-0.45,0.8,0.5));\n"
            "  float d = max(dot(normalize(fragNormal), l), 0.0);\n"
            "  finalColor = vec4(colDiffuse.rgb*(0.35+0.65*d), colDiffuse.a); }";
        shader = LoadShaderFromMemory(vs, fs);
        mat = LoadMaterialDefault();
        mat.shader = shader;
    }

    void close() {
        UnloadMesh(cube);
        UnloadMesh(sphere);
        UnloadMesh(cylinder);
        UnloadShader(shader);
        CloseWindow();
    }

    static Color bodyColor(const mc::RigidBody& b, int id) {
        if (b.isStatic()) return Color{110, 110, 118, 255};
        static const Color palette[] = {
            {230, 90, 70, 255}, {70, 140, 230, 255}, {90, 190, 100, 255},
            {235, 180, 60, 255}, {170, 100, 220, 255}, {70, 200, 200, 255}};
        Color c = palette[id % 6];
        if (b.asleep) { // desaturate sleeping bodies
            c.r = (unsigned char)((c.r + 2 * 128) / 3);
            c.g = (unsigned char)((c.g + 2 * 128) / 3);
            c.b = (unsigned char)((c.b + 2 * 128) / 3);
        }
        return c;
    }

    void drawBody(const mc::RigidBody& b, Color color) {
        const mc::Quat& q = b.xf.orientation;
        Matrix mBody = MatrixMultiply(QuaternionToMatrix(Quaternion{q.x, q.y, q.z, q.w}),
                                      MatrixTranslate(b.xf.position.x, b.xf.position.y, b.xf.position.z));
        mat.maps[MATERIAL_MAP_DIFFUSE].color = color;
        switch (b.shape.type) {
        case mc::ShapeType::Box: {
            const mc::Vec3& h = b.shape.halfExtents;
            DrawMesh(cube, mat, MatrixMultiply(MatrixScale(2 * h.x, 2 * h.y, 2 * h.z), mBody));
            break;
        }
        case mc::ShapeType::Sphere: {
            float r = b.shape.radius;
            DrawMesh(sphere, mat, MatrixMultiply(MatrixScale(r, r, r), mBody));
            break;
        }
        case mc::ShapeType::Capsule: {
            float r = b.shape.radius, hh = b.shape.halfHeight;
            DrawMesh(cylinder, mat,
                     MatrixMultiply(MatrixMultiply(MatrixScale(r, 2 * hh, r), MatrixTranslate(0, -hh, 0)), mBody));
            DrawMesh(sphere, mat, MatrixMultiply(MatrixMultiply(MatrixScale(r, r, r), MatrixTranslate(0, hh, 0)), mBody));
            DrawMesh(sphere, mat, MatrixMultiply(MatrixMultiply(MatrixScale(r, r, r), MatrixTranslate(0, -hh, 0)), mBody));
            break;
        }
        }
    }

    void drawWorld(mc::World& w) {
        for (int i = 0; i < w.bodyCount(); ++i)
            drawBody(w.body(i), bodyColor(w.body(i), i));
        DrawGrid(40, 1.0f);
    }

    void hud(mc::World& w, const char* extra = nullptr) {
        const auto& p = w.profile();
        DrawFPS(12, 10);
        DrawText(TextFormat("step %.2f ms  (broad %.2f  narrow %.2f  solve %.2f)",
                            p.total(), p.broadphase, p.narrowphase, p.solve),
                 12, 34, 18, DARKGRAY);
        DrawText(TextFormat("bodies %d  awake %d  pairs %d",
                            w.bodyCount(), w.awakeCount(), w.lastPairCount()),
                 12, 54, 18, DARKGRAY);
        if (extra) DrawText(extra, 12, 74, 18, Color{40, 90, 160, 255});
    }
};

// Fixed-timestep stepping decoupled from render rate.
struct Stepper {
    float accumulator = 0;
    float dt = 1.0f / 60.0f;
    void advance(mc::World& w) {
        accumulator += GetFrameTime();
        if (accumulator > 0.25f) accumulator = 0.25f; // avoid spiral of death
        while (accumulator >= dt) {
            w.step(dt);
            accumulator -= dt;
        }
    }
};

} // namespace viewer
