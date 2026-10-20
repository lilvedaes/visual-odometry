cmake_minimum_required(VERSION 3.21)
project(VisualOdometry)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Include OpenCV headers
include_directories(${OpenCV_INCLUDE_DIRS})

# Create the executable
add_executable(VisualOdometry
    main.cpp
    src/DataFileHandler.cpp
    src/Calibration.cpp
    src/VisualOdometry.cpp
    src/TrajectoryDrawer.cpp
)

target_include_directories(VisualOdometry PRIVATE src)

# Link OpenCV libraries to the executable
target_link_libraries(VisualOdometry ${OpenCV_LIBS})
