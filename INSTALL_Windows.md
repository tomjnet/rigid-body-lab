# Installing dependencies (Windows)

Everything needed to build and run **rigid-body-lab / minicollide** on a clean
Windows PC. Verified end to end on **Windows 11 Home 24H2 (build 26200, x64)**,
Intel Core i5-9300H, Intel UHD 630 + NVIDIA GTX 1650, with **MinGW-w64 GCC
16.1.0** (Chocolatey `mingw` package), **CMake 4.3.1**, **Ninja 1.13.2** and
**Git 2.55** — full build, `ctest`, `mc_bench` and all five raylib demos.

The engine itself is dependency-free C++20. As on macOS, **the demos need no
extra libraries**: raylib 5.5 builds GLFW's Win32 backend and links only DLLs
that ship with Windows (`opengl32`, `gdi32`, `winmm`, `user32`, `shell32`).
There is no X11/Mesa/ALSA equivalent to install.

> **Two things are different from Linux/macOS — read these first.**
>
> 1. **Use MinGW-w64 GCC, not MSVC.** `CMakeLists.txt` passes GCC-style flags
>    (`-Wall -Wextra -mavx2 -mfma`) unconditionally, and `bench/bench_main.cpp`
>    calls the GCC builtin `__builtin_cpu_supports`. `cl.exe` rejects the first
>    (`D8021 : invalid numeric argument '/Wextra'`) and has no equivalent of the
>    second. MinGW-w64 GCC understands both, so the project builds **with no
>    source changes**.
> 2. **`net_server` / `net_client` do not build on Windows.** They use POSIX
>    sockets (`<arpa/inet.h>`, `<sys/socket.h>`, `fcntl`, `usleep`), which do
>    not exist on native Windows (MSVC or MinGW). Porting them to Winsock is
>    a code change and has not been made. Build every other target explicitly
>    (see [Build & verify](#build--verify)); 7 of the 9 binaries build.

---

## TL;DR

Open **PowerShell as Administrator** (Start → type *PowerShell* → *Run as
administrator*), then:

```powershell
# 1. Install Chocolatey (skip if `choco --version` already works)
Set-ExecutionPolicy Bypass -Scope Process -Force
[System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072
iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))

# 2. Install the toolchain (compiler, build system, generator, git)
choco install -y mingw ninja git
choco install -y cmake --installargs '"ADD_CMAKE_TO_PATH=System"'

# 3. Pick up the new PATH in this window (or just open a new PowerShell)
Import-Module "$env:ChocolateyInstall\helpers\chocolateyProfile.psm1"; refreshenv
```

That is the complete dependency list, demos included. Then, in a normal
(non-admin) PowerShell, from the repository root:

```powershell
cmake -B build -G Ninja "-DCMAKE_BUILD_TYPE=Release" "-DCMAKE_POLICY_VERSION_MINIMUM=3.5" `
      -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++
ninja -C build minicollide mc_tests mc_bench demo_stack demo_chain demo_pile demo_fort demo_molecules
```

Headless / CI (no demos, no window, no raylib download):

```powershell
cmake -B build -G Ninja "-DCMAKE_BUILD_TYPE=Release" -DMC_BUILD_DEMOS=OFF `
      -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++
ninja -C build minicollide mc_tests mc_bench
```

> **Quote every `-D...` argument that contains a dot.** Windows PowerShell 5.1
> splits `-DCMAKE_POLICY_VERSION_MINIMUM=3.5` into `-DCMAKE_POLICY_VERSION_MINIMUM=3`
> and `.5`, and configure then fails deep inside raylib with
> `Invalid CMAKE_POLICY_VERSION_MINIMUM value "3"`. Writing it as
> `"-DCMAKE_POLICY_VERSION_MINIMUM=3.5"` avoids that. PowerShell 7 and `cmd.exe`
> do not have this problem.

---

## What each piece is for

### 1. Compiler & build toolchain (always required)

| Component | Install with | Provides | Why the project needs it |
|---|---|---|---|
| **Chocolatey** | the `iex ... install.ps1` line above | `choco` | Package manager used for everything below. Needs an **elevated** PowerShell for installs. Official instructions: <https://chocolatey.org/install>. |
| **MinGW-w64 GCC** | `choco install mingw` | `gcc`, `g++`, `mingw32-make`, `objdump`, libstdc++, winpthreads | The engine is **C++20** (`<thread>`, `<atomic>`, `<numeric>`). `CMAKE_CXX_STANDARD 20` / `CXX_STANDARD_REQUIRED ON` in `CMakeLists.txt`. GCC is required rather than MSVC because of the GCC-only flags and builtin described at the top. Installs to `C:\ProgramData\mingw64\mingw64\bin` and adds it to the system `PATH`. The package ships the `posix` threading model, which `std::thread` needs. |
| **CMake ≥ 3.16** | `choco install cmake --installargs '"ADD_CMAKE_TO_PATH=System"'` | `cmake`, `ctest` | Build system. `cmake_minimum_required(VERSION 3.16)`. `ctest` runs the unit/determinism suite. Without `ADD_CMAKE_TO_PATH` the installer does **not** put `cmake` on `PATH`. |
| **Ninja** | `choco install ninja` | `ninja` | The generator used throughout this document: fast and parallel across all cores without a `-j` flag. Select it with `-G Ninja`. (The alternative is `-G "MinGW Makefiles"` with `mingw32-make -j8`, which is slower on the raylib build.) |
| **Git** | `choco install git` | `git` | **Required when demos are enabled**: `FetchContent` clones `https://github.com/raysan5/raylib.git` at tag `5.5` during `cmake -B build`. |
| **TLS root certificates** | already present | Windows certificate store | Git for Windows validates the HTTPS clone against the Windows store (schannel). Nothing to install. |

`pkg-config` is **not** needed on Windows. raylib resolves everything through the
Win32 system libraries.

Threads: `find_package(Threads REQUIRED)` is satisfied by MinGW's
**winpthreads** (`libwinpthread-1.dll`), which ships with the `mingw` package.

**Already have Visual Studio?** Its bundled `cmake.exe` and `ninja.exe`
(4.3.1 / 1.13.2 on the verification machine, from *Visual Studio 2026 Build
Tools*) work fine with MinGW GCC, so you can skip `choco install cmake ninja`.
Keep passing `-DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++`, though: inside a
*Developer PowerShell* CMake otherwise picks `cl.exe` and the build fails (see
[Troubleshooting](#troubleshooting)).

### 2. Graphics / windowing / audio (only if you build the demos)

**Nothing to install.** raylib 5.5 builds its bundled GLFW against the **Win32**
backend and links system DLLs that ship with Windows:

| System library | Why |
|---|---|
| `opengl32.dll` | raylib's renderer targets **OpenGL 3.3 core**. The actual driver comes from your GPU vendor (Intel/NVIDIA/AMD). Replaces `libgl1-mesa-dev`. |
| `gdi32.dll`, `user32.dll` | GLFW's Win32 backend: window creation, the message loop, keyboard/mouse input, cursor hiding for the demos' FPS-style mouse look. Replaces the `libx*-dev` packages. |
| `winmm.dll` | High-resolution timer (`timeBeginPeriod`) so `SetTargetFPS(60)` paces accurately. |
| `shell32.dll` | Used by GLFW for drag-and-drop and paths. |
| WASAPI (via miniaudio) | raylib always compiles its audio module even though these demos play no sound. miniaudio loads WASAPI at **runtime**, so nothing extra is linked or installed. Replaces `libasound2-dev`. |

Expect a few harmless compiler warnings from raylib's bundled `jar_mod.h`
(`-Wstringop-overflow`) during the build; they are in third-party code.

### 3. Networking (`net_server` / `net_client`)

**Not available on Windows.** Both programs include `<arpa/inet.h>`,
`<sys/socket.h>`, `<netinet/in.h>`, `<fcntl.h>` and `<unistd.h>`, and use
`fcntl(O_NONBLOCK)`, `close()` and `usleep()`. None of these exist on native
Windows, so both translation units fail with:

```
fatal error: arpa/inet.h: No such file or directory
```

That is why the build commands in this document list targets explicitly rather
than building `all`. Your options:

- **WSL 2 (recommended):** `wsl --install -d Ubuntu`, then follow
  [INSTALL_Ubuntu.md](INSTALL_Ubuntu.md) inside WSL. `net_server` /
  `net_client --probe` are headless and work there as they do on Linux. Windows 11
  WSLg can also show the demo windows.
- **A Winsock port:** wrap the socket calls behind `#ifdef _WIN32`
  (`winsock2.h`/`ws2tcpip.h`, `WSAStartup`, `ioctlsocket(FIONBIO)`,
  `closesocket`, link `ws2_32`). This is a source change and not part of this
  repository today.

### 4. Optional / nice to have

| Tool | Install with | Why |
|---|---|---|
| `gdb` | included in `mingw` | Debugging (`-DCMAKE_BUILD_TYPE=Debug`). |
| LLVM / `clangd` | `choco install llvm` | Consumes the `compile_commands.json` this project already exports (`CMAKE_EXPORT_COMPILE_COMMANDS ON`) for editor code navigation. |
| Visual Studio Code | `choco install vscode` | With the *C/C++* or *clangd* and *CMake Tools* extensions; point CMake Tools at the MinGW kit. |
| Windows Performance Recorder / Analyzer | *Windows Performance Toolkit* feature of the [Windows ADK](https://learn.microsoft.com/windows-hardware/get-started/adk-install) | Profiling `mc_bench` / `demo_pile` beyond the built-in per-phase profiler. This is the `perf` / `sample` replacement. |
| Sysinternals Process Explorer | `choco install procexp` | Watch per-core utilization while `demo_pile [threads]` runs. |

---

## Hardware / runtime requirements

- **x64 CPU with AVX2 + FMA.** `bench/bench_main.cpp` is compiled with
  `-mavx2 -mfma`. The kernel is guarded at runtime by
  `__builtin_cpu_supports("avx2")`, but the whole translation unit is built with
  those flags, so a pre-2013 CPU can still fault. Check from PowerShell (40 is
  `PF_AVX2_INSTRUCTIONS_AVAILABLE`):
  ```powershell
  Add-Type -Namespace W -Name K -MemberDefinition '[DllImport("kernel32.dll")] public static extern bool IsProcessorFeaturePresent(uint f);'
  [W.K]::IsProcessorFeaturePresent(40)    # True if AVX2 is supported
  ```
  (This is the Windows equivalent of `grep avx2 /proc/cpuinfo`.) **Windows on
  ARM** needs the same two guards described in the
  [macOS Apple Silicon section](INSTALL_macOS.md#apple-silicon-arm64).
- **GPU with OpenGL 3.3.** Any Intel HD 4000 or newer, or any NVIDIA/AMD GPU
  from the last decade, **with the vendor driver installed**. The generic
  *Microsoft Basic Display Adapter* driver only provides OpenGL 1.1, and the
  demos then exit at startup. Each demo prints its OpenGL context to the console
  on launch; look for the `GL: OpenGL device information:` block and its
  `> Version:` line.
- **Laptops with two GPUs (Intel + NVIDIA):** Windows starts unknown `.exe`s on
  the integrated GPU. That is fine for these demos (the verification machine
  ran them at 60 FPS on UHD 630). To force the discrete GPU, use *Settings →
  System → Display → Graphics*, add the `.exe`, and choose *High performance*.
- **An interactive desktop session** for the demos. Over plain SSH there is no
  desktop, so use Remote Desktop, or skip the demos with `-DMC_BUILD_DEMOS=OFF`.
  `mc_tests` and `mc_bench` are headless.
- **Internet access at configure time** for the raylib clone (first configure
  only; cached in `build\_deps\` afterwards).
- **~400 MB of disk** for the build tree, most of it raylib.

---

## Build & verify

### Configure (once)

```powershell
cd rigid-body-lab
cmake -B build -G Ninja "-DCMAKE_BUILD_TYPE=Release" "-DCMAKE_POLICY_VERSION_MINIMUM=3.5" `
      -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++
```

- `-G Ninja` selects the Ninja generator. It is recorded in `build\CMakeCache.txt`,
  so later `ninja` / `cmake --build` calls reuse it.
- `-DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++` pins MinGW GCC. Without it,
  CMake may choose `cl.exe` if a Visual Studio environment is active. The C
  compiler matters too, because raylib is C.
- `"-DCMAKE_POLICY_VERSION_MINIMUM=3.5"` is **required on CMake 4.x** (what
  Chocolatey and VS 2026 install today). raylib 5.5's bundled GLFW declares
  `cmake_minimum_required(3.4)`, which CMake 4 rejects. **Keep the quotes** (see
  the PowerShell note in the TL;DR).
- The backtick `` ` `` at the end of the first line is PowerShell's line
  continuation. In `cmd.exe`, use `^` instead, or write it all on one line.
- Demos are **on by default** (`MC_BUILD_DEMOS=ON`). This step clones raylib 5.5
  into `build\_deps\`, so it needs network the first time.

### Build

```powershell
ninja -C build minicollide mc_tests mc_bench demo_stack demo_chain demo_pile demo_fort demo_molecules
```

This builds seven of the nine binaries: the engine library, tests, benchmark
and all five demos. A bare `ninja -C build` also tries `net_server` /
`net_client`, fails on them (section 3), and stops. If you prefer to build
everything that *can* build, use `ninja -C build -k 0`. It keeps going past
those two failures, but still exits non-zero.

Useful variants:

```powershell
ninja -C build demo_fort     # one target only
ninja -C build -n mc_tests   # dry run: list what is out of date
ninja -C build -t clean      # delete build outputs, keep the raylib clone
```

### Runtime DLLs: run from a shell that has MinGW on PATH, or link statically

By default the executables link dynamically against MinGW's runtime
(`libstdc++-6.dll`, `libgcc_s_seh-1.dll`, `libwinpthread-1.dll`). The `mingw`
package puts those on the system `PATH`, so everything runs from any terminal
on the build machine. **Copied to another PC, or started with a stripped
`PATH`, the programs fail silently** with exit code `-1073741515`
(`0xC0000135`, *DLL not found*). Check what a binary needs with:

```powershell
objdump -p build\mc_tests.exe | Select-String 'DLL Name'
```

To produce self-contained `.exe`s, add static linking at configure time:

```powershell
cmake -B build -G Ninja "-DCMAKE_BUILD_TYPE=Release" "-DCMAKE_POLICY_VERSION_MINIMUM=3.5" `
      -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ "-DCMAKE_EXE_LINKER_FLAGS=-static"
```

Verified: a `-static` `mc_tests.exe` passes with every MinGW directory removed
from `PATH`, while the dynamic one fails with `0xC0000135`.

### The five demos

All need a desktop session. Run them from the build tree (`cd build`):

| Binary | What it exercises |
|---|---|
| `.\demo_stack.exe` | Stacking credibility: a 10-box stack and a pyramid resting without jitter (warm starting, Baumgarte, slop). SPACE fires a heavy sphere; sleeping bodies render desaturated and wake as the island graph reconnects. |
| `.\demo_chain.exe` | Joints: a ball-socket capsule chain swinging a wrecking ball into a box tower, plus a hinge gate. |
| `.\demo_pile.exe` | Performance: mixed bodies rain into a pile with a per-phase profile HUD. **The only demo taking an argument:** thread count, default all cores (`.\demo_pile.exe 1`, `.\demo_pile.exe 8`). |
| `.\demo_fort.exe` | Gameplay sandbox: first-person capsule character, grid-snapped build mode (1 wall / 2 floor / 3 ramp, LMB to place), and pooled sphere projectiles. WASD/arrows + mouse, SPACE/BACKSPACE jump, SHIFT sprint, R respawn, ESC menu. |
| `.\demo_molecules.exe` | Runtime joint creation: atoms bond via ball-socket joints built from contact manifolds, with union-find over the bond graph naming molecules (H2O, CO2, CH4). SPACE injects thermal energy. |

raylib writes its `INFO:` log to the console. That is normal and not an error.

### Verify

```powershell
cd build
ctest --output-on-failure    # unit + integration + determinism tests
.\mc_bench.exe               # SIMD + multithreading benchmarks (no window)
.\demo_stack.exe             # needs a desktop session
```

Measured on the verification machine (i5-9300H, 4C/8T):

```
== SIMD solver kernel: 8192 rows x 400 iterations ==
  scalar :   74.92 ms
  AVX2   :   14.86 ms   speedup 5.04x   max |dLambda| vs scalar: 2.38e-05
== Engine scene: 1297 bodies, 60 frames ==
  1 thread :    4.71 ms/step
  8 threads:    2.51 ms/step   speedup 1.88x
```

To stress-test the threaded code path, run the suite many times:

```powershell
$fail = 0; 1..200 | % { .\mc_tests.exe *> $null; if ($LASTEXITCODE) { $fail++ } }; "$fail / 200 failed"
```

---

## Troubleshooting

**`Invalid CMAKE_POLICY_VERSION_MINIMUM value "3"`**
PowerShell 5.1 split the unquoted argument at the dot. Quote it:
`"-DCMAKE_POLICY_VERSION_MINIMUM=3.5"`, delete the half-configured tree
(`Remove-Item -Recurse -Force build`) and configure again.

**`cl : Command line error D8021 : invalid numeric argument '/Wextra'`**
CMake picked MSVC, which typically happens inside a *Developer PowerShell for
VS* or when `cl.exe` is on `PATH`. The compiler is cached, so start over with it
pinned:
```powershell
Remove-Item -Recurse -Force build
cmake -B build -G Ninja "-DCMAKE_POLICY_VERSION_MINIMUM=3.5" -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++
```
Check which compiler a tree uses with
`Select-String CMAKE_CXX_COMPILER: build\CMakeCache.txt`.

**`fatal error: arpa/inet.h: No such file or directory`**
You built `net_server` / `net_client` (for example with a bare `ninja` or
`cmake --build build`). They are POSIX-only (section 3). Build the explicit
target list from [Build](#build), or use WSL.

**`'gcc' is not recognized` / `cmake: The term 'cmake' is not recognized`**
The window was opened before the Chocolatey install updated `PATH`. Run
`refreshenv` (after importing the Chocolatey profile, as in the TL;DR) or open a
new PowerShell. For CMake, also make sure it was installed with
`ADD_CMAKE_TO_PATH=System`; otherwise add `C:\Program Files\CMake\bin` to `PATH`.

**`choco : ... Access to the path ... is denied`**
Chocolatey installs need an **Administrator** PowerShell. Building does not.

**A program exits immediately with code `-1073741515` (`0xC0000135`)**
A MinGW runtime DLL is not on `PATH` (see
[Runtime DLLs](#runtime-dlls-run-from-a-shell-that-has-mingw-on-path-or-link-statically)).
Either add `C:\ProgramData\mingw64\mingw64\bin` to `PATH` or reconfigure with
`"-DCMAKE_EXE_LINKER_FLAGS=-static"`.

**`fatal: unable to access 'https://github.com/raysan5/raylib.git'`**
No network or a corporate proxy. Either fix connectivity
(`git config --global http.proxy http://proxy:port`) or build without demos:
`-DMC_BUILD_DEMOS=OFF`.

**Demo window opens and closes at once / `Failed to initialize graphics device`**
No OpenGL 3.3 context. Install the GPU vendor driver: with *Microsoft Basic
Display Adapter* in Device Manager you only have OpenGL 1.1. In a VM, enable 3D
acceleration. There is no Mesa `LIBGL_ALWAYS_SOFTWARE` switch on Windows.

**SmartScreen / Defender flags a freshly built `.exe`**
Locally built, unsigned binaries sometimes trigger *Windows protected your PC*
when launched from Explorer. Running them from the terminal avoids the prompt.
If Defender quarantines one, add the `build` folder as an exclusion
(*Windows Security → Virus & threat protection → Exclusions*).

**`ctest` or a demo intermittently dies with
`terminate called after throwing an instance of 'std::system_error'  what(): Not enough space`**
This is the `JobSystem::parallelFor` use-after-free described in
[INSTALL_macOS.md](INSTALL_macOS.md#troubleshooting). It is far more frequent
under MinGW's winpthreads than on macOS: **107 of 200** `mc_tests` runs and
roughly 1 in 5 `demo_molecules` launches crashed on the verification machine.
It is **fixed in `src/parallel/job_system.h`**, where the completion counter is
now decremented under `doneMutex`. After the fix there were 0 crashes in 800
classified test runs (the only failures were the unrelated line-175 check
below), and every demo survived repeated launches. If you still see it, your checkout
predates the fix.

**`ctest` rarely fails with `FAIL tests/test_main.cpp:175  h1 == h3`**
This is an engine bug, not an install problem. It is independent of threading
and happened in about 1 run in 150 on the verification machine. In
`src/collision/collide.cpp`, the degenerate-EPA fallback of `boxCapsule`
declares `float depth;` and passes it to `closestOnBox`, which only writes it
when the capsule's centre is *inside* the box. When the centre is outside, the
contact penetration is read from uninitialized stack memory, and the push-out
normal is zero. The single-threaded determinism run then occasionally diverges.
Building with `-ftrivial-auto-var-init=zero` made 7,200 runs bit-identical,
which confirms the cause. A pass is a genuine pass; re-run `ctest`.
