// Copyright 2023 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "cobo_arm_control/ik_solver.hpp"
#include <cmath>
#include <limits>
#include <iostream>

namespace cobo_arm_control
{

IKSolver::IKSolver(double l1_length, double l2_length)
: l1_length_(l1_length), l2_length_(l2_length)
{
  // Initialize default joint limits
  joint_limits_[static_cast<size_t>(JointType::J1_UP)] = {1.58, 4.79};
  joint_limits_[static_cast<size_t>(JointType::J1_DOWN)] = {1.58, 4.79};
  joint_limits_[static_cast<size_t>(JointType::J2_UP)] = {1.58, 4.79};
  joint_limits_[static_cast<size_t>(JointType::J2_DOWN)] = {1.58, 4.79};
  joint_limits_[static_cast<size_t>(JointType::J3_UP)] = {-1.58, 1.57};
  joint_limits_[static_cast<size_t>(JointType::J3_DOWN)] = {-1.58, 1.57};

  // Initialize default joint conversions
  joint_conversions_[static_cast<size_t>(JointType::J1_UP)] = 1.57;
  joint_conversions_[static_cast<size_t>(JointType::J1_DOWN)] = 1.57;
  joint_conversions_[static_cast<size_t>(JointType::J2_UP)] = 3.14;
  joint_conversions_[static_cast<size_t>(JointType::J2_DOWN)] = 0.0;
  joint_conversions_[static_cast<size_t>(JointType::J3_UP)] = 0.0;
  joint_conversions_[static_cast<size_t>(JointType::J3_DOWN)] = 0.0;
}

void IKSolver::setArmLengths(double l1, double l2)
{
  l1_length_ = l1;
  l2_length_ = l2;
}

void IKSolver::setJointLimits(JointType joint, double min, double max)
{
  joint_limits_[static_cast<size_t>(joint)] = {min, max};
}

void IKSolver::setJointConversion(JointType joint, double conversion)
{
  joint_conversions_[static_cast<size_t>(joint)] = conversion;
}

double IKSolver::wrapAngle(double angle) const
{
  // Normalize angle to [0, 2*PI)
  double wrapped = std::fmod(angle, 2.0 * M_PI);
  if (wrapped < 0.0) {
    wrapped += 2.0 * M_PI;
  }
  return wrapped;
}

double IKSolver::adjustJointAngle(double angle, JointType joint_type) const
{
  double adjusted = angle + joint_conversions_[static_cast<size_t>(joint_type)];
  return wrapAngle(adjusted);
}

bool IKSolver::isWithinLimits(double angle, JointType joint_type) const
{
  const auto & limits = joint_limits_[static_cast<size_t>(joint_type)];
  return angle >= limits.min && angle <= limits.max;
}

bool IKSolver::validateReachability(double x, double y) const
{
  const double distance = std::sqrt(x * x + y * y);
  const double max_reach = l1_length_ + l2_length_;
  const double min_reach = std::abs(l1_length_ - l2_length_);
  std::cout << "Distance: " << distance << " Max reach: " << max_reach << " Min reach: " << min_reach << std::endl;
  return distance <= max_reach && distance >= min_reach;
}

bool IKSolver::isNearSingularity(double x, double y, double tolerance) const
{
  const double distance = std::sqrt(x * x + y * y);
  const double max_reach = l1_length_ + l2_length_;
  const double min_reach = std::abs(l1_length_ - l2_length_);

  return std::abs(distance - max_reach) < tolerance ||
         std::abs(distance - min_reach) < tolerance;
}

double IKSolver::computeEndEffectorRotation(
  double j1, double j2, double target_rotation) const
{
  // The end effector rotation is relative to the arm's final orientation
  // Joint 3 controls the end effector rotation
  // Total arm angle = j1 + j2
  const double arm_angle = j1 + j2;

  // Joint 3 needs to compensate for arm angle to achieve target rotation
  double j3 = target_rotation - arm_angle;

  // Normalize to [-PI, PI]
  while (j3 > M_PI) {
    j3 -= 2.0 * M_PI;
  }
  while (j3 < -M_PI) {
    j3 += 2.0 * M_PI;
  }

  return j3;
}

bool IKSolver::computeIKSolution(
  const Pose2D & target,
  ElbowConfig config,
  IKSolution & solution) const
{
  const double x = target.x;
  const double y = target.y;
  const double target_rotation = target.rotation;

  // Distance to target
  const double p = std::sqrt(x * x + y * y);

  // Cosine of joint 2 angle using law of cosines
  const double cos_j2 = (p * p - l1_length_ * l1_length_ - l2_length_ * l2_length_) /
                        (2.0 * l1_length_ * l2_length_);

  // Check if solution exists
  if (cos_j2 < -1.0 || cos_j2 > 1.0) {
    std::cout << "no solution " << cos_j2 << std::endl;
    return false;
  }

  // Joint 2 angle (elbow angle) - raw geometric angle
  const double j2_raw = (config == ElbowConfig::UP) ?
              std::acos(cos_j2) : -std::acos(cos_j2);

  // Joint 1 angle (shoulder angle) - raw geometric angle
  const double k1 = l1_length_ + l2_length_ * std::cos(j2_raw);
  const double k2 = l2_length_ * std::sin(j2_raw);
  const double j1_raw = std::atan2(y, x) - std::atan2(k2, k1);

  // Compute joint 3 using raw geometric angles (before motor convention adjustment)
  double j3 = computeEndEffectorRotation(j1_raw, j2_raw, target_rotation);

  // Now adjust j1/j2 for motor conventions
  const JointType j1_type = (config == ElbowConfig::UP) ?
                            JointType::J1_UP : JointType::J1_DOWN;
  const JointType j2_type = (config == ElbowConfig::UP) ?
                            JointType::J2_UP : JointType::J2_DOWN;
  const JointType j3_type = (config == ElbowConfig::UP) ?
                            JointType::J3_UP : JointType::J3_DOWN;

  double j1 = adjustJointAngle(j1_raw, j1_type);
  double j2 = adjustJointAngle(j2_raw, j2_type);

  // Check joint limits (in motor coordinates)
  if (!isWithinLimits(j1, j1_type) || !isWithinLimits(j2, j2_type)) {
    std::cout << "j1/2 limits " << j1 << " " << j2 << std::endl;
    return false;
  }

  // Check joint 3 limits (j3 is already normalized to [-PI, PI])
  if (!isWithinLimits(j3, j3_type)) {
    std::cout << "j3 limits " << j3 << std::endl;
    return false;
  }

  // Populate solution with geometric angles (not motor-converted angles)
  solution.joint_1 = j1_raw;
  solution.joint_2 = j2_raw;
  solution.joint_3 = j3;
  solution.elbow_config = config;
  solution.is_valid = true;

  return true;
}

std::optional<IKSolution> IKSolver::solve(
  const Pose2D & target_pose,
  ArmSide arm_side,
  std::optional<ElbowConfig> preferred_config) const
{
  // Validate reachability
  if (!validateReachability(target_pose.x, target_pose.y)) {
     std::cout << "Target pose unreachable" << std::endl;
     return std::nullopt;
  }

  // Check for singularities
  if (isNearSingularity(target_pose.x, target_pose.y)) {
    // Near singularity - still attempt but may be unstable
    // In a production system, you might want to return nullopt or add damping
  }

  IKSolution solution_up, solution_down;
  bool has_up = computeIKSolution(target_pose, ElbowConfig::UP, solution_up);
  bool has_down = computeIKSolution(target_pose, ElbowConfig::DOWN, solution_down);

  // If preferred configuration is specified and valid, use it
  if (preferred_config.has_value()) {
    if (preferred_config.value() == ElbowConfig::UP && has_up) {
      return solution_up;
    }
    if (preferred_config.value() == ElbowConfig::DOWN && has_down) {
      return solution_down;
    }
  }

  // Otherwise, return any valid solution (prefer UP if both valid)
  if (has_up) {
    return solution_up;
  }
  if (has_down) {
    return solution_down;
  }

  return std::nullopt;
}

}  // namespace cobo_arm_control
