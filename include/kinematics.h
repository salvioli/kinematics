#pragma once

#include <Eigen/Dense>
#include <optional>

namespace robotics {

// Forward declarations
class Pose2D;
class JointAngles;

// Type aliases
using Transform2D = Eigen::Matrix3d;

// Function declarations
double normalizeAngle(double angle);

/**
 * @brief Implementation of a 3R planar robot kinematics
 *
 * This class implements the forward and inverse kinematics for a 3R planar
 * robot as described in the technical documentation. The robot consists of
 * three revolute joints with link lengths L1, L2, and L3.
 *
 * All linear dimensions (link lengths and positions) are in meters.
 * All angular dimensions are in radians.
 */
class PlanarRobot3R {
public:
  /**
   * @brief Joint angles configuration
   *
   * Represents the three joint angles of the robot
   * theta1: angle of first joint (rad)
   * theta2: angle of second joint (rad)
   * theta3: angle of third joint (rad)
   */
  struct JointAngles {
    double theta1; ///< First joint angle in radians
    double theta2; ///< Second joint angle in radians
    double theta3; ///< Third joint angle in radians
  };

  /**
   * @brief End-effector pose
   *
   * Represents the position and orientation of the end-effector
   * x: x-coordinate in the base frame (meters)
   * y: y-coordinate in the base frame (meters)
   * phi: orientation angle in radians
   */
  struct Pose {
    double x;   ///< X position in base frame (meters)
    double y;   ///< Y position in base frame (meters)
    double phi; ///< Orientation in radians
  };

  /**
   * @brief Construct a new Planar Robot 3R object
   *
   * @param l1 Length of the first link in meters (must be positive)
   * @param l2 Length of the second link in meters (must be positive)
   * @param l3 Length of the third link in meters (must be positive)
   * @throw std::invalid_argument if any link length is non-positive
   */
  PlanarRobot3R(double l1, double l2, double l3);

  /**
   * @brief Compute forward kinematics
   *
   * @param angles Input joint angles configuration
   * @return Pose End-effector pose
   */
  Pose forwardKinematics(const JointAngles &angles) const;

  /**
   * @brief Compute inverse kinematics
   *
   * Calculates joint angles that achieve the desired end-effector pose.
   * Returns nullopt if the pose is not reachable or if no solution exists.
   * When multiple solutions exist, returns the elbow-up configuration.
   * If the elbow down configuration is desired, the user can obtain as
   * q_ed = (theta1, -theta2, theta3).
   *
   * @param pose Target end-effector pose
   * @return std::optional<JointAngles> Solution (if exists)
   */
  std::optional<JointAngles> inverseKinematics(const Pose &pose) const;

private:
  double L1, L2, L3; ///< Link lengths in meters
};

} // namespace robotics
