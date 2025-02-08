# 3R Planar Robot Kinematics Library

A very minimal C++ library implementing forward and inverse kinematics for a 3R planar robot manipulator.

## Features

- Forward kinematics computation
- Fast analytical inverse kinematics computation via closed form solution
- Support for arbitrary link lengths

## Limitations

- Only supports planar 3R serial robots

## Prerequisites

- CMake 3.20 or higher
- C++20 compiler
- Eigen3
- Google Test (only for tests)

## Building

Clone the repository and build (out-of-source build recommended):

```bash
git clone https://github.com/salvioli/kinematics.git kinematics
cmake -S kinematics -B build -DKINEMATICS_BUILD_TESTS=ON
cmake --build build && ctest --test-dir build --verbose
```

Your directory structure should look like:
```
.
├── kinematics/
│   ├── CMakeLists.txt
│   ├── include/
│   ├── src/
│   └── test/
└── build/
    └── ...
```

## Usage

To use this library in your project:

1. Add as a subdirectory in your CMake project:

    ```cmake
    add_subdirectory(path/to/kinematics)
    ```

1. Link against the library:

    ```cmake
    target_link_libraries(your_target PRIVATE kinematics)
    ```

1. Example code:

    ```cpp
    #include <kinematics.h>

    int main() {
        // Create robot with link lengths [meters]
        robotics::PlanarRobot3R robot(1.0, 1.0, 0.5);

        // Forward kinematics
        auto angles = robotics::PlanarRobot3R::JointAngles{0.1, 0.2, 0.3};
        auto pose = robot.forwardKinematics(angles);

        // Inverse kinematics
        auto target = robotics::PlanarRobot3R::Pose{1.0, 1.0, 0.0};
        auto solution = robot.inverseKinematics(target);
        if (solution) {
            // Solution found
            auto joint_angles = *solution;
        }

        return 0;
    }
    ```

## Usage Example

```cpp
#include <kinematics.h>

int main() {
    // Create robot with link lengths (in meters)
    robotics::PlanarRobot3R robot(1.0, 1.0, 0.5);

    // Forward kinematics
    auto angles = robotics::PlanarRobot3R::JointAngles{0.1, 0.2, 0.3}; // radians
    auto pose = robot.forwardKinematics(angles);  // returns {x, y} in meters

    // Inverse kinematics
    auto target = robotics::PlanarRobot3R::Pose{1.0, 1.0, 0.0};  // {x meters, y meters, phi radians}
    auto solution = robot.inverseKinematics(target);
    if (solution) {
        // Solution found
        auto joint_angles = *solution;  // returns angles in radians
    }

    return 0;
}
```

## License

MIT License

## Author

Federico Salvioli
