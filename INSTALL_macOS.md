# Installing dependencies (macOS)

Everything needed to build and run **rigid-body-lab / minicollide** on a clean
Mac. Verified end to end on **macOS 12.7.6 Monterey (x86_64)** with Apple clang
11.0.0, CMake 3.17.3 and Ninja 1.11.1 — full build, `ctest`, `mc_bench` and all
five raylib demos. The Ninja build below was re-verified on the same machine
with the current Homebrew toolchain — **CMake 4.4.0, Ninja 1.11.1, AppleClang
14.0.0** — which needs one extra configure flag; see
[Build & verify](#build--verify). The same steps apply to macOS 13–15 and to
Apple Silicon, with one required source tweak for arm64 (see
[Apple Silicon](#apple-silicon-arm64) below).

The engine itself is dependency-free C++20 (only `pthread`, which lives in
`libSystem`). Unlike Linux, **the demos need nothing installed either** —
raylib 5.5 links only frameworks that ship with macOS (Cocoa, IOKit, OpenGL,
CoreFoundation). There is no X11/Mesa/ALSA equivalent to install here.

---

## TL;DR — two commands

```bash
xcode-select --install          # compiler, headers, macOS SDK, git  (~1.5 GB)
brew install cmake ninja        # build system + fast generator
```

That is the complete dependency list, demos included. Then configure with Ninja
and build everything — engine, tests, benchmark, both net binaries and all five
demos:

```bash
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5
ninja -C build
```

Headless server / CI (no demos, no window) — same two commands, then configure
with `-DMC_BUILD_DEMOS=OFF`. That skips the raylib download entirely, so the
build needs no network after the initial clone:

```bash
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DMC_BUILD_DEMOS=OFF
```

No Homebrew? `cmake` and `ninja` are also available from
[cmake.org](https://cmake.org/download/) / MacPorts, or you can drop `ninja` and
use the Command Line Tools' `make` (the default generator) — it is slower on the
raylib build but works.

---

## What each piece is for

### 1. Compiler & build toolchain (always required)

| Component | Install with | Provides | Why the project needs it |
|---|---|---|---|
| **Xcode Command Line Tools** | `xcode-select --install` | `clang`, `clang++`, `make`, `ld`, `git`, `lldb`, libc++ headers, the macOS SDK | The engine is **C++20** (`<thread>`, `<atomic>`, `<numeric>`; no modules/coroutines). `CMAKE_CXX_STANDARD 20` / `CXX_STANDARD_REQUIRED ON` in `CMakeLists.txt`. Apple clang 11 (Xcode 11) is enough — CMake maps C++20 to `-std=c++2a` there; anything newer is better. |
| **CMake ≥ 3.16** | `brew install cmake` | `cmake`, `ctest` | Build system. `cmake_minimum_required(VERSION 3.16)`. `ctest` runs the unit/determinism suite. See the CMake 4.x note in [Troubleshooting](#troubleshooting). |
| **Ninja** | `brew install ninja` | `ninja` | Generator used throughout this document — noticeably faster than `make` on the ~50-target raylib build, and it parallelises across all cores with no `-j` flag. Select it with `-G Ninja` at configure time. Technically optional; drop it and CMake falls back to `make`. |
| **git** | included in the CLT | `git` | **Required when demos are enabled**: `FetchContent` clones `https://github.com/raysan5/raylib.git` at tag `5.5` during `cmake -B build`. |
| **TLS root certificates** | already present | system keychain | The HTTPS clone to GitHub validates against the system trust store. Unlike a minimal Linux container, there is no `ca-certificates` package to install. |
| **The full Xcode app** | *not needed* | — | Command Line Tools alone are sufficient. Install Xcode only if you want Instruments for profiling. |

`pkg-config` is **not** needed on macOS. On Ubuntu raylib uses it to find the
X11/ALSA `-dev` packages; here it resolves everything through frameworks in the
SDK instead.

Threads: `find_package(Threads REQUIRED)` is satisfied by the pthreads
implementation inside `libSystem`. Nothing to install, and no `-lpthread` link
flag is emitted on Darwin.

### 2. Graphics / windowing / audio (only if you build the demos)

**Nothing to install.** This is the one section where macOS is dramatically
simpler than Ubuntu. raylib 5.5 builds its bundled GLFW against the Cocoa
backend and links system frameworks that are part of the OS:

| Framework | Why |
|---|---|
| `Cocoa` (AppKit + Foundation) | GLFW's Cocoa backend — window creation, the event loop, keyboard/mouse input, cursor hiding for the demos' FPS-style mouse look. Replaces `libx11-dev`, `libxi-dev`, `libxcursor-dev`, `libxkbcommon-dev`. |
| `OpenGL` | raylib's renderer targets **OpenGL 3.3 core**. Replaces `libgl1-mesa-dev` / `libglu1-mesa-dev`. |
| `IOKit` | Display/monitor enumeration and video modes. Replaces `libxrandr-dev` / `libxinerama-dev`. |
| `CoreFoundation`, `CoreGraphics`, `CoreServices` | Pulled in transitively by Cocoa/IOKit for display geometry and bundle lookup. |
| CoreAudio (via miniaudio) | raylib always compiles its audio module even though these demos play no sound. miniaudio resolves CoreAudio at **runtime**, so nothing extra is linked or installed. Replaces `libasound2-dev`. |

You can confirm this on a built binary — the whole dependency list is system
libraries:

```bash
otool -L build/demo_stack
```

Expect a configure-time warning, which is normal and harmless:

```
CMake Warning ... OpenGL is deprecated starting with macOS 10.14 (Mojave)!
```

Apple deprecated OpenGL in 2018 and caps it at **4.1**, but never removed it.
3.3 core is well within that, on both Intel and Apple Silicon.

### 3. Networking (`net_server` / `net_client`)

Nothing to install. Plain POSIX UDP sockets (`<sys/socket.h>`,
`<netinet/in.h>`, `<arpa/inet.h>`) from `libSystem`. Default port **UDP 47001**
on loopback.

The macOS Application Firewall does not filter loopback traffic, so the default
same-machine setup never prompts. If you run server and client on **different
machines** and the firewall is on (`--getglobalstate` returns `enabled`), macOS
will show a "Do you want the application `net_server` to accept incoming network
connections?" dialog because the locally built binary is unsigned. Click *Allow*,
or pre-authorize it:

```bash
/usr/libexec/ApplicationFirewall/socketfilterfw --getglobalstate
sudo /usr/libexec/ApplicationFirewall/socketfilterfw --add "$PWD/net_server"
sudo /usr/libexec/ApplicationFirewall/socketfilterfw --unblockapp "$PWD/net_server"
```

This is the macOS equivalent of `sudo ufw allow 47001/udp`.

### 4. Optional / nice to have

| Tool | Install with | Why |
|---|---|---|
| `lldb` | included in the CLT | Debugging (`cmake -DCMAKE_BUILD_TYPE=Debug`). **Use this instead of `gdb`** — gdb on macOS needs a code-signing certificate to control processes and is not worth the setup. |
| `clangd` | `brew install llvm` | Consumes the `compile_commands.json` this project already exports (`CMAKE_EXPORT_COMPILE_COMMANDS ON`). The CLT ships clangd only in recent versions. |
| `sample` / `xctrace` | CLT / Xcode | Profiling `mc_bench` / `demo_pile` beyond the built-in per-phase profiler. This is the `perf` replacement: `sample mc_bench 10 -f out.txt`, or Instruments' Time Profiler. |
| `htop` | `brew install htop` | Watch core utilization while `demo_pile [threads]` runs. |

---

## Hardware / runtime requirements

- **CPU.**
  - **Intel Macs** need AVX2 + FMA for the SIMD benchmark — `bench/bench_main.cpp`
    is compiled with `-mavx2 -mfma`. The kernel is guarded at runtime by
    `__builtin_cpu_supports("avx2")`, but the whole translation unit is built
    with those flags, so a pre-2013 Mac can still fault. Check with:
    ```bash
    sysctl -n machdep.cpu.leaf7_features | tr ' ' '\n' | grep AVX2
    ```
    (This is the macOS equivalent of `grep avx2 /proc/cpuinfo`.) Every Mac from
    2013 on — Haswell and later — has it. Measured on the verification machine:
    **12.9x** over scalar.
  - **Apple Silicon** requires a two-line source change before `mc_bench` will
    compile at all — see below.
- **GPU with OpenGL 3.3** for the demos. Every Mac since roughly 2012 exposes
  OpenGL 4.1, and so does Apple Silicon, so this is effectively always
  satisfied. There is no `glxinfo`; if you want to check, use:
  ```bash
  system_profiler SPDisplaysDataType | grep -i -E "chipset|vendor|metal"
  ```
  Note there is **no software-rendering fallback** on macOS — no Mesa, so
  `LIBGL_ALWAYS_SOFTWARE=1` has no effect. In a VM you must enable 3D
  acceleration on the host.
- **A logged-in graphical (Aqua) session** for the demos. Unlike Linux there is
  no `ssh -X` equivalent — X11 forwarding cannot carry a Cocoa window. Over SSH,
  either use Screen Sharing, or skip the demos with `-DMC_BUILD_DEMOS=OFF` —
  `mc_tests`, `mc_bench`, `net_server` and `net_client --probe` are all headless.
- **Internet access at configure time** for the raylib clone (first configure
  only; cached in `build/_deps/` afterwards).
- **~400 MB of disk** for the build tree (364 MB measured, most of it raylib).

### Apple Silicon (arm64)

The engine, tests, demos and networking are all portable and build natively on
arm64. Only `mc_bench` is x86-specific, and it fails in **two** places:

1. `CMakeLists.txt` passes `-mavx2 -mfma`, which Apple clang rejects outright for
   an arm64 target (`unsupported option`).
2. `bench/bench_main.cpp:14` includes `<immintrin.h>` unconditionally, and that
   header hard-`#error`s on non-x86 targets.

The AVX2 kernel body is already behind `#ifdef __AVX2__`, so guarding those two
lines is all that is needed — the benchmark then builds and prints
`AVX2 : not compiled in`, with the scalar and multithreading numbers intact.

In `CMakeLists.txt`, make the flags conditional:

```cmake
add_executable(mc_bench bench/bench_main.cpp)
target_link_libraries(mc_bench PRIVATE minicollide)
target_compile_options(mc_bench PRIVATE -Wall -Wextra)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
  target_compile_options(mc_bench PRIVATE -mavx2 -mfma)
endif()
```

In `bench/bench_main.cpp`, guard the include:

```cpp
#if defined(__x86_64__) || defined(__i386__)
#include <immintrin.h>
#endif
```

Two further notes for Apple Silicon:

- **Do not reach for Rosetta 2** to keep the AVX2 path. Rosetta implements only
  SSE-level SIMD; an AVX2 instruction traps with `SIGILL`. The native build with
  the guards above is the right answer.
- Homebrew lives at `/opt/homebrew` rather than `/usr/local`, so make sure
  `/opt/homebrew/bin` is on your `PATH` or CMake/Ninja will not be found.
- `std::thread::hardware_concurrency()` counts performance **and** efficiency
  cores. For clean benchmark numbers pass the P-core count explicitly:
  `./demo_pile 8`.

---

## Build & verify

### Configure (once)

```bash
cd rigid-body-lab
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```

- `-G Ninja` selects the Ninja generator. It is recorded in `build/CMakeCache.txt`,
  so every later `ninja` / `cmake --build` call reuses it — you never repeat this flag.
- `-DCMAKE_POLICY_VERSION_MINIMUM=3.5` is **required on CMake 4.x** (what Homebrew
  installs today) and harmless on 3.x. raylib 5.5's bundled GLFW declares
  `cmake_minimum_required(3.4)`, which CMake 4 rejects outright. Omit it there and
  configure fails before a single object is compiled. See
  [Troubleshooting](#troubleshooting).
- Demos are **on by default** (`MC_BUILD_DEMOS=ON`). This step clones raylib 5.5
  into `build/_deps/`, so it needs network the first time. Add `-DMC_BUILD_DEMOS=OFF`
  to skip raylib entirely.

Switching an existing headless build tree back to demos means re-running configure
with `-DMC_BUILD_DEMOS=ON`; the option is cached, so a bare `ninja` will not pick it
up on its own.

### Build everything

```bash
ninja -C build
```

That single command builds all nine binaries — `minicollide`, `mc_tests`,
`mc_bench`, `net_server`, `net_client` and **all five demos**. `-C build` runs
ninja in the build tree without `cd`. No `-j` is needed: ninja defaults to one
job per core, whereas `cmake --build` defaults to serial and needs an explicit
`-j$(sysctl -n hw.logicalcpu)` (macOS has no `nproc`).

Useful variants:

```bash
ninja -C build demo_fort     # one target only — skips tests, bench and net
ninja -C build -n            # dry run: list what is out of date
ninja -C build -t clean      # delete build outputs, keep the raylib clone
```

`cmake --build build` still works and drives the same Ninja files; `ninja` is
just the shorter path to it.

### The five demos

All need a graphical (Aqua) session. Run them from the build tree:

| Binary | What it exercises |
|---|---|
| `./demo_stack` | Stacking credibility — a 10-box stack and a pyramid resting without jitter (warm starting, Baumgarte, slop). SPACE fires a heavy sphere; sleeping bodies render desaturated and wake as the island graph reconnects. |
| `./demo_chain` | Joints — a ball-socket capsule chain swinging a wrecking ball into a box tower, plus a hinge gate. |
| `./demo_pile` | Performance — mixed bodies rain into a pile with a per-phase profile HUD. **The only demo taking an argument:** thread count, default all cores (`./demo_pile 1`, `./demo_pile 8`). |
| `./demo_fort` | Gameplay sandbox — first-person capsule character, grid-snapped build mode (1 wall / 2 floor / 3 ramp, LMB to place), and pooled sphere projectiles. WASD/arrows + mouse, SPACE/BACKSPACE jump, SHIFT sprint, R respawn, ESC menu. |
| `./demo_molecules` | Runtime joint creation — atoms bond via ball-socket joints built from contact manifolds, with union-find over the bond graph naming molecules (H2O, CO2, CH4). SPACE injects thermal energy. |

### Verify

```bash
cd build
ctest --output-on-failure    # unit + integration + determinism tests
./mc_bench                   # SIMD + multithreading benchmarks (no window)
./demo_stack                 # needs a graphical session
```

Quick headless smoke test of the network stack:

```bash
./net_server --frames 600 &
./net_client --probe 100
```

---

## Troubleshooting

**`xcrun: error: invalid active developer path ... missing xcrun`**
The Command Line Tools are absent or were wiped by an OS upgrade. Reinstall with
`xcode-select --install`. If you have full Xcode installed and CMake picks the
wrong toolchain, point it at the right one:
`sudo xcode-select --switch /Library/Developer/CommandLineTools`.

**CMake 4.x rejects raylib's policy version**
Homebrew now ships CMake 4.x, which dropped compatibility with
`cmake_minimum_required` values below 3.5. raylib 5.5's bundled GLFW declares
`3.4`, so it trips this. It is a **configure-time** failure — nothing compiles —
and it only bites when demos are enabled, since GLFW is the offender. The flag
is already in the recommended command above; on its own:
```bash
cmake -B build -G Ninja -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```

**`ninja: error: unknown target 'demo_fort'`**
The build tree was configured with `-DMC_BUILD_DEMOS=OFF`, so no demo targets
exist in `build.ninja`. The option is cached — re-running `ninja` will never
create them. Re-configure the same tree with demos on:
```bash
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DMC_BUILD_DEMOS=ON -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```
Check which way a tree is currently set with
`grep MC_BUILD_DEMOS build/CMakeCache.txt`.

**`unsupported option '-mavx2'` or `"This header is only meant to be used on x86"`**
You are on Apple Silicon. Apply the two guards in
[Apple Silicon](#apple-silicon-arm64) above.

**`fatal: unable to access 'https://github.com/raysan5/raylib.git'`**
No network or a corporate proxy. Either fix connectivity (`git config --global
http.proxy ...`) or build without demos: `cmake -B build -DMC_BUILD_DEMOS=OFF`.

**`CMake Warning: OpenGL is deprecated starting with macOS 10.14`**
Expected and harmless — see section 2. OpenGL 3.3 still works; Apple has only
deprecated it, not removed it.

**Demo runs but its window never comes to the front**
A bare binary run from the terminal has no `.app` bundle, so macOS does not
give it focus automatically. Click the window, or `Cmd-Tab` to it. If it never
appears at all, you are almost certainly in an SSH session with no Aqua session
— GUI demos cannot run that way.

**`ctest` intermittently reports `unit (ILLEGAL)` or `(SEGFAULT)` even though
`mc_tests` prints "All tests passed."**
This is an **engine race, not an install problem** — it reproduced at roughly
5% (11 failures in 200 runs) on the verification machine, always after the tests
had already passed. `JobSystem::parallelFor`
(`src/parallel/job_system.h:45`) keeps `remaining`, `doneMutex` and `doneCv` as
stack locals and has the last worker decrement `remaining` *before* it takes
`doneMutex`. The waiter's predicate can therefore observe `remaining == 0` and
return while that worker is still about to lock the now-destroyed mutex — a
use-after-free that libc++ turns into `std::__throw_system_error` → `abort`.
The macOS crash report confirms it:

```
std::__1::mutex::lock() -> __throw_system_error -> abort
  in JobSystem::parallelFor(...)::'lambda'()  <- worker thread
```

Linux's glibc happens to tolerate the same pattern, which is why the Ubuntu doc
does not mention it. The fix is to notify under the lock — decrement `remaining`
while already holding `doneMutex` — or to have `parallelFor` own the
synchronization state rather than borrowing the stack frame. Until that lands,
re-run `ctest`; a pass is a genuine pass.

**`net_client` receives nothing**
The server must be running (`./net_server &`) and UDP 47001 must be free. macOS
has no `ss`; use:
```bash
lsof -nP -iUDP:47001
```
Use `--port N` on both sides to change it. Cross-machine, also check the
Application Firewall (section 3).
