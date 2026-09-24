# Installing dependencies (Ubuntu)

Everything needed to build and run **rigid-body-lab / minicollide** on a clean
Ubuntu machine. Verified against **Ubuntu 26.04 LTS (x86_64)**; the same package
names are valid on 22.04 and 24.04.

The engine itself is dependency-free C++20 (only `pthread` from the system).
Everything in the "graphics" section below exists solely because the demo
viewers link **raylib 5.5**, which CMake downloads and builds from source at
configure time.

---

## TL;DR — one command

```bash
sudo apt update && sudo apt install -y \
  build-essential cmake ninja-build git ca-certificates pkg-config \
  libgl1-mesa-dev libglu1-mesa-dev \
  libx11-dev libxrandr-dev libxi-dev libxcursor-dev libxinerama-dev \
  libxkbcommon-dev libwayland-dev wayland-protocols \
  libasound2-dev
```

Headless server / CI (no demos, no window) — much smaller:

```bash
sudo apt update && sudo apt install -y build-essential cmake ninja-build git ca-certificates
# then configure with -DMC_BUILD_DEMOS=OFF
```

---

## What each package is for

### 1. Compiler & build toolchain (always required)

| Package | Provides | Why the project needs it |
|---|---|---|
| `build-essential` | `g++`, `gcc`, `make`, `libc6-dev`, `libstdc++-dev` | The engine is **C++20** (`concepts`-free but uses C++20 language level, `<thread>`, `<atomic>`, `<numeric>`). GCC ≥ 10 is enough; Ubuntu 26.04 ships GCC 15. `CMAKE_CXX_STANDARD 20` / `CXX_STANDARD_REQUIRED ON` in `CMakeLists.txt`. |
| `cmake` | `cmake`, `ctest` | Build system. `cmake_minimum_required(VERSION 3.16)`. `ctest` runs the unit/determinism suite. |
| `ninja-build` | `ninja` | Optional but recommended generator (faster than Make, especially for the raylib build). Use `-G Ninja`. |
| `git` | `git` | **Required when demos are enabled**: `FetchContent` clones `https://github.com/raysan5/raylib.git` at tag `5.5` during `cmake -B build`. |
| `ca-certificates` | TLS root certs | So the `git clone` over HTTPS to GitHub validates. Usually already present; missing in minimal container images. |
| `pkg-config` | `pkg-config` | Used by raylib's CMake to locate the X11/Wayland/ALSA libs below. |

Threads: `find_package(Threads REQUIRED)` → satisfied by glibc's `pthread`,
already part of `libc6-dev` (in `build-essential`). No separate package.

### 2. Graphics / windowing / audio (only if you build the demos)

raylib 5.5 builds GLFW for the desktop platform, which links against X11, OpenGL
and ALSA. These are all `-dev` headers needed **at compile time**.

| Package | Why |
|---|---|
| `libgl1-mesa-dev` | OpenGL headers + `libGL` link target. raylib's renderer targets **OpenGL 3.3**. |
| `libglu1-mesa-dev` | GLU headers some raylib/GLFW configurations expect. |
| `libx11-dev` | Core X11 client library — GLFW's X11 backend. |
| `libxrandr-dev` | Monitor/video-mode enumeration (RandR). |
| `libxi-dev` | XInput2 — mouse/keyboard input. |
| `libxcursor-dev` | Cursor shapes / hiding the cursor (the demos use FPS-style mouse look). |
| `libxinerama-dev` | Multi-monitor geometry queries. |
| `libxkbcommon-dev` | Keyboard keymap handling (required by GLFW 3.4, both X11 and Wayland). |
| `libwayland-dev`, `wayland-protocols` | Only needed if you build raylib with the Wayland backend. On a Wayland desktop the default X11 build still works fine through **XWayland**, so these are optional — include them if you want a native Wayland build or want to avoid surprises. |
| `libasound2-dev` | ALSA headers. raylib always compiles its audio module (miniaudio) even though these demos play no sound. |

### 3. Networking (`net_server` / `net_client`)

No packages. Plain POSIX UDP sockets (`<sys/socket.h>`, `<netinet/in.h>`,
`<arpa/inet.h>`) from glibc. Default port **UDP 47001** on loopback. If you run
server and client on different machines and a firewall is active:

```bash
sudo ufw allow 47001/udp    # only if ufw is enabled and you go cross-machine
```

### 4. Optional / nice to have

| Package | Why |
|---|---|
| `gdb` | Debugging (`cmake -DCMAKE_BUILD_TYPE=Debug`). |
| `mesa-utils` | `glxinfo` — check you actually have OpenGL 3.3 before wondering why a demo won't open a window. |
| `clang`, `clang-tools` / `clangd` | Alternative compiler; `clangd` consumes the `compile_commands.json` this project already exports (`CMAKE_EXPORT_COMPILE_COMMANDS ON`). |
| `linux-tools-common linux-tools-generic` | `perf`, for profiling `mc_bench` / `demo_pile` beyond the built-in per-phase profiler. |

---

## Hardware / runtime requirements

- **x86_64 CPU with AVX2 + FMA.** `bench/bench_main.cpp` is compiled with
  `-mavx2 -mfma`. The SIMD kernel is guarded at runtime by
  `__builtin_cpu_supports("avx2")`, so it prints a scalar-only result on old
  CPUs — but since the whole translation unit is built with those flags, a
  pre-2013 CPU can still fault. Check with:
  ```bash
  grep -o -m1 avx2 /proc/cpuinfo     # prints "avx2" if supported
  ```
  On ARM (e.g. a Raspberry Pi or Apple Silicon VM) remove `-mavx2 -mfma` from
  the `mc_bench` target in `CMakeLists.txt`; everything else is portable.
- **GPU with OpenGL 3.3** (or Mesa software rendering) for the demos. Verify:
  ```bash
  glxinfo | grep "OpenGL version"
  ```
- **A graphical session** (X11 or Wayland) for the demos. Over SSH use
  `ssh -X`, or skip the demos entirely with `-DMC_BUILD_DEMOS=OFF` —
  `mc_tests`, `mc_bench`, `net_server` and `net_client --probe` are all headless.
- **Internet access at configure time** for the raylib clone (first configure
  only; it is cached in `build/_deps/` afterwards).
- ~500 MB of disk for the build tree (most of it raylib).

---

## Build & verify

```bash
cd rigid-body-lab
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release   # add -DMC_BUILD_DEMOS=OFF to skip raylib
cmake --build build -j$(nproc)
cd build
ctest --output-on-failure    # unit + integration + determinism tests
./mc_bench                   # SIMD + multithreading benchmarks (no window)
./demo_stack                 # needs a display
```

Quick headless smoke test of the network stack:

```bash
./net_server --frames 600 &
./net_client --probe 100
```

---

## Troubleshooting

**`Could NOT find OpenGL` / `X11_X11_LIB not found` during configure**
The raylib build is missing the section-2 `-dev` packages. Install them, then
reconfigure from scratch: `rm -rf build && cmake -B build ...`.

**`fatal: unable to access 'https://github.com/raysan5/raylib.git'`**
No network, a proxy, or missing `ca-certificates`. Either fix connectivity or
build without demos: `cmake -B build -DMC_BUILD_DEMOS=OFF`.

**CMake 4.x rejects raylib's policy version**
Ubuntu 26.04 ships CMake 4.2, which dropped compatibility with
`cmake_minimum_required` values below 3.5. If the raylib subproject trips this,
configure with:
```bash
cmake -B build -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```

**Demo opens then exits / `Failed to initialize graphics device`**
No OpenGL 3.3 context — check `glxinfo`, install your GPU driver, or force
software rendering: `LIBGL_ALWAYS_SOFTWARE=1 ./demo_stack`.

**`net_client` receives nothing**
The server must be running (`./net_server &`) and UDP 47001 must be free:
`ss -lunp | grep 47001`. Use `--port N` on both sides to change it.
