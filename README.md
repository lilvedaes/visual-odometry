# Visual Odometry

A C++17 visual-odometry project using OpenCV and CMake. It implements a
traditional, deterministic computer-vision pipeline that estimates a vehicle's
relative motion and cumulative 3D trajectory from a sequence of images
captured by a single calibrated camera.

The system uses visual feature detection and tracking, optical flow, RANSAC
outlier rejection, epipolar geometry, and essential-matrix decomposition to
recover camera rotation and translation without relying on GPS. These
techniques are relevant to autonomous flight, terrain-referenced navigation,
visual dead reckoning, Visual SLAM, and planetary rover navigation.

The program accepts a KITTI sequence ID from `00` through `21`. If no sequence
ID is provided, it defaults to sequence `00`.

Sequences `00` through `10` include public ground-truth poses. Sequences `11`
through `21` do not, so their trajectories are estimated with unknown scale
and are shown without a ground-truth comparison.

## Prerequisites

Install the following tools:

- [CMake](https://cmake.org/download/) 3.21 or newer
- A C++17 compiler
- [OpenCV](https://opencv.org/releases/)

On Windows, [MSYS2](https://www.msys2.org/) with the UCRT64 toolchain is one
possible setup. Make sure the compiler, CMake, and OpenCV installation are
available on `PATH`, or provide OpenCV's CMake package directory explicitly
when configuring the project.

## Dataset

This project uses the [KITTI Vision Benchmark Suite Odometry
dataset](https://www.cvlibs.net/datasets/kitti/eval_odometry.php). The
following two downloads are needed:

- [Download odometry data set (grayscale, 22 GB)](https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_gray.zip)
  - Contains the grayscale image sequences and their calibration files.
- [Download odometry ground truth poses (4 MB)](https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_poses.zip)
  - Contains the ground-truth pose files used for evaluation.

The dataset is intentionally excluded from Git because of its size. To install
it:

1. Download both ZIP files.
2. Extract the grayscale archive.
3. Extract the ground-truth poses archive.
4. Create a `data` directory in the project root.
5. Copy or move the extracted `sequences` directory into `data`.
6. Copy or move the extracted `poses` directory into `data`.

The project should have this structure:

```text
VisualOdometry/
├── CMakeLists.txt
├── README.md
├── main.cpp
├── src/
└── data/
    ├── poses/
    │   ├── 00.txt
    │   ├── 01.txt
    │   └── ...
    └── sequences/
        ├── 00/
        │   ├── calib.txt
        │   ├── times.txt
        │   ├── image_0/
        │   │   ├── 000000.png
        │   │   ├── 000001.png
        │   │   └── ...
        │   └── image_1/
        │       ├── 000000.png
        │       └── ...
        ├── 01/
        └── ...
```

The paths must be directly under `data/`. For example, the calibration file
for sequence `00` must be located at:

```text
data/sequences/00/calib.txt
```

and the left-camera images must be located at:

```text
data/sequences/00/image_0/
```

Do not leave an extra archive directory level such as
`data/data_odometry_gray/sequences/00/`; the application expects
`data/sequences/00/` directly.

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
their operating system and debugger.

## Run

Run from the project root so the application can resolve the `data/` directory:

```powershell
.\build\VisualOdometry.exe 00
```

The sequence argument is optional. For example, to use sequence `05`:

```powershell
.\build\VisualOdometry.exe 05
```

If no argument is provided, sequence `00` is used.

## Visualization

The program opens two resizeable windows while it runs:

- **Trajectory** shows the estimated trajectory in red and the ground-truth
  trajectory in green when it is available. It also shows the current
  coordinates for each line.
- **Current Image** shows the current KITTI image in color.

The trajectory view automatically rescales as new positions are added. This
keeps the display useful for both short and long sequences, including
sequences without ground truth.

## Possible improvements

The accuracy could be improved by using a stronger feature detector and
descriptor, such as ORB or SIFT, instead of relying only on FAST features.
Better feature matching, adaptive RANSAC thresholds, and rejecting frames with
too few reliable tracks would make the pose estimates more stable.

The current approach estimates motion one frame at a time, so small errors
accumulate into drift. Keyframe selection, local bundle adjustment, loop
closure, and pose-graph optimization could reduce that drift. Stereo images or
an IMU would also provide more reliable depth and scale, especially for
sequences `11` through `21`.

For better efficiency, images could be loaded and processed in worker threads
while the visualization runs separately. Image buffers could be reused,
feature detection could be limited to a region of interest, and the number of
tracked features could be capped. Drawing and coordinate updates could also
run less often than the motion-estimation loop.
