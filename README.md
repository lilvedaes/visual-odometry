# Visual Odometry

A C++ project using OpenCV and CMake.

## Prerequisites

Install the following tools:

- [CMake](https://cmake.org/download/) 3.21 or newer
- A C++17 compiler
- [OpenCV](https://opencv.org/releases/)

On Windows, [MSYS2](https://www.msys2.org/) with the UCRT64 toolchain is one
possible setup. Make sure the compiler, CMake, and OpenCV installation are
available on `PATH`, or provide OpenCV's CMake package directory explicitly
when configuring the project.

The local `data/` directory contains the dataset used for development. It is
intentionally excluded from Git because of its size and must be obtained
separately.

## Build from a terminal

From the project directory:

```powershell
cmake -S . -B build
cmake --build build
```

If CMake cannot find OpenCV, configure again with the directory containing
OpenCV's `OpenCVConfig.cmake` file:

```powershell
cmake -S . -B build -DOpenCV_DIR="C:\path\to\opencv\build"
cmake --build build
```

The executable is created in the `build/` directory. CMake-generated files and
build outputs are ignored by Git.

## Build with VS Code

VS Code is optional; the project can be built entirely from the terminal.

1. Install the **C/C++** extension from Microsoft.
2. Install the **CMake Tools** extension from Microsoft.
3. Open the project folder in VS Code.
4. Run **CMake: Select Configure Preset** or **CMake: Configure** from the
   Command Palette.
5. Select a C++17-capable compiler kit.
6. If OpenCV is not found automatically, set `OpenCV_DIR` in the CMake Tools
   configuration to the directory containing `OpenCVConfig.cmake`.
7. Run **CMake: Build**.

Personal VS Code settings, tasks, and launch configurations are intentionally
not committed. Each developer can create the configuration appropriate for
their operating system, compiler, and debugger.

## Run

The current sample program reads `test.jpg` from the working directory and
opens it in an OpenCV window:

```powershell
.\build\VisualOdometry.exe
```
