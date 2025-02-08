#include "kinematics.h"
#include <cmath>
#include <stdexcept>

namespace robotics {

namespace {
/**
 * @brief Check if a point is reachable by the first two links
 *
 * @param x X coordinate of point [m]
 * @param y Y coordinate of point [m]
 * @param L1 Length of first link [m]
 * @param L2 Length of second link [m]
 * @return true if point is reachable
 */
bool isReachable(double x, double y, double L1, double L2) {
  double r = std::hypot(x, y);
  return r >= std::abs(L1 - L2) && r <= (L1 + L2);
}
} // namespace

PlanarRobot3R::PlanarRobot3R(double l1, double l2, double l3)
    : L1(l1), L2(l2), L3(l3) {
  if (l1 <= 0 || l2 <= 0 || l3 <= 0) {
    throw std::invalid_argument("Link lengths must be positive");
  }
}

PlanarRobot3R::Pose
PlanarRobot3R::forwardKinematics(const JointAngles &angles) const {
  // Calculate cumulative angles
  double theta12 = angles.theta1 + angles.theta2;
  double theta123 = theta12 + angles.theta3;

  // Calculate end-effector position
  double x = L1 * std::cos(angles.theta1) + L2 * std::cos(theta12) +
             L3 * std::cos(theta123);

  double y = L1 * std::sin(angles.theta1) + L2 * std::sin(theta12) +
             L3 * std::sin(theta123);

  // End-effector orientation is sum of all angles
  double phi = theta123;

  return Pose{x, y, phi};
}

std::optional<PlanarRobot3R::JointAngles>
PlanarRobot3R::inverseKinematics(const Pose &pose) const {
  // Calculate wrist position from end-effector pose
  double x3 = pose.x - L3 * std::cos(pose.phi);
  double y3 = pose.y - L3 * std::sin(pose.phi);

  // Check if wrist position is reachable
  if (!isReachable(x3, y3, L1, L2)) {
    return std::nullopt;
  }

  // Calculate theta2 using law of cosines
  double r_squared = x3 * x3 + y3 * y3;
  double cos_theta2 = (r_squared - L1 * L1 - L2 * L2) / (2 * L1 * L2);

  // Check if argument to arccos is valid
  if (cos_theta2 < -1 || cos_theta2 > 1) {
    return std::nullopt;
  }

  // We choose the elbow-up configuration (positive theta2)
  double theta2 = std::acos(cos_theta2);

  // Calculate theta1
  double theta1 = std::atan2(y3, x3) -
                  std::atan2(L2 * std::sin(theta2), L1 + L2 * std::cos(theta2));

  // Calculate theta3 from the total orientation constraint
  double theta3 = pose.phi - theta1 - theta2;

  return JointAngles{theta1, theta2, theta3};
}

} // namespace robotics
