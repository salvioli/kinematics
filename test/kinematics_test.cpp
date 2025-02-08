#include "kinematics.h"
#include <gtest/gtest.h>
#include <numbers>

using std::numbers::pi;

class PlanarRobotTest : public ::testing::Test {
protected:
  void SetUp() override {
    robot = std::make_unique<robotics::PlanarRobot3R>(3.0, 2.0, 1.0);
  }

  void expectPosesNear(const robotics::PlanarRobot3R::Pose &p1,
                       const robotics::PlanarRobot3R::Pose &p2,
                       double tolerance = 1e-6) {
    EXPECT_NEAR(p1.x, p2.x, tolerance)
        << "x positions differ: actual=" << p1.x << ", expected=" << p2.x;
    EXPECT_NEAR(p1.y, p2.y, tolerance)
        << "y positions differ: actual=" << p1.y << ", expected=" << p2.y;
    EXPECT_NEAR(p1.phi, p2.phi, tolerance)
        << "orientations differ: actual=" << p1.phi << ", expected=" << p2.phi;
  }

  std::unique_ptr<robotics::PlanarRobot3R> robot;
};

TEST_F(PlanarRobotTest, ForwardKinematicsDefault) {
  robotics::PlanarRobot3R::JointAngles angles{0, 0, 0};
  auto result = robot->forwardKinematics(angles);
  expectPosesNear(result, {6.0, 0.0, 0.0});
}

TEST_F(PlanarRobotTest, ForwardKinematicsFirstJoint90) {
  robotics::PlanarRobot3R::JointAngles angles{pi / 2, 0, 0};
  auto result = robot->forwardKinematics(angles);
  expectPosesNear(result, {0.0, 6.0, pi / 2});
}

TEST_F(PlanarRobotTest, ForwardKinematicsAllJoints45) {
  robotics::PlanarRobot3R::JointAngles angles{pi / 4, pi / 4, pi / 4};
  auto result = robot->forwardKinematics(angles);

  // Expected position calculation based on cumulative angles
  double expected_x = 3.0 * std::cos(pi / 4) + 2.0 * std::cos(pi / 2) +
                      1.0 * std::cos(3 * pi / 4);
  double expected_y = 3.0 * std::sin(pi / 4) + 2.0 * std::sin(pi / 2) +
                      1.0 * std::sin(3 * pi / 4);

  expectPosesNear(result, {expected_x, expected_y, 3 * pi / 4});
}

TEST_F(PlanarRobotTest, InverseKinematicsStartPosition) {
  robotics::PlanarRobot3R::Pose target{6.0, 0.0, 0.0};
  auto solution = robot->inverseKinematics(target);
  ASSERT_TRUE(solution.has_value())
      << "Failed to find inverse kinematics solution for pose: "
      << "x=" << target.x << ", y=" << target.y << ", phi=" << target.phi;

  auto fk_result = robot->forwardKinematics(*solution);
  expectPosesNear(target, fk_result);
}

TEST_F(PlanarRobotTest, InverseKinematicsUnreachablePosition) {
  robotics::PlanarRobot3R::Pose target{10.0, 10.0, 0.0};
  auto solution = robot->inverseKinematics(target);
  EXPECT_FALSE(solution.has_value())
      << "Expected unreachable pose to return no solution: "
      << "x=" << target.x << ", y=" << target.y << ", phi=" << target.phi;
}

TEST_F(PlanarRobotTest, ForwardInverseConsistency) {
  robotics::PlanarRobot3R::JointAngles initial{pi / 6, pi / 4, pi / 3};
  auto fk_result = robot->forwardKinematics(initial);
  auto ik_solution = robot->inverseKinematics(fk_result);

  ASSERT_TRUE(ik_solution.has_value())
      << "Failed to find inverse kinematics solution for pose: "
      << "x=" << fk_result.x << ", y=" << fk_result.y
      << ", phi=" << fk_result.phi << " from joint angles: "
      << "theta1=" << initial.theta1 << ", theta2=" << initial.theta2
      << ", theta3=" << initial.theta3;

  auto fk_check = robot->forwardKinematics(*ik_solution);
  expectPosesNear(fk_result, fk_check);
}

TEST_F(PlanarRobotTest, InvalidLinkLengths) {
  EXPECT_THROW(
      { robotics::PlanarRobot3R(0.0, 1.0, 1.0); }, std::invalid_argument)
      << "Expected throw for l1=0";

  EXPECT_THROW(
      { robotics::PlanarRobot3R(1.0, -1.0, 1.0); }, std::invalid_argument)
      << "Expected throw for negative l2";

  EXPECT_THROW(
      { robotics::PlanarRobot3R(1.0, 1.0, 0.0); }, std::invalid_argument)
      << "Expected throw for l3=0";
}
