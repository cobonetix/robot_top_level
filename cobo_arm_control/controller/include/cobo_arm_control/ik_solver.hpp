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

#ifndef COBO_ARM_CONTROL__IK_SOLVER_HPP_
#define COBO_ARM_CONTROL__IK_SOLVER_HPP_

#include <array>
#include <cmath>
#include <optional>

namespace cobo_arm_control
{

enum class JointType
{
  J1_UP = 0,
  J1_DOWN = 1,
  J2_UP = 2,
  J2_DOWN = 3,
  J3_UP = 4,
  J3_DOWN = 5
};

enum class ArmSide
{
  LEFT = 0,
  RIGHT = 1
};

enum class ElbowConfig
{
  UP = 0,
  DOWN = 1
};

struct JointLimits
{
  double min;
  double max;
};

struct IKSolution
{
  double joint_1;
  double joint_2;
  double joint_3;
  ElbowConfig elbow_config;
  bool is_valid;
};

struct Pose2D
{
  double x;
  double y;
  double rotation;
};

class IKSolver
{
public:
  IKSolver(double l1_length, double l2_length);

  // Main IK solver function
  std::optional<IKSolution> solve(
    const Pose2D & target_pose,
    ArmSide arm_side,
    std::optional<ElbowConfig> preferred_config = std::nullopt) const;

  // Configuration
  void setArmLengths(double l1, double l2);
  void setJointLimits(JointType joint, double min, double max);
  void setJointConversion(JointType joint, double conversion);

  // Getters
  double getL1() const { return l1_length_; }
  double getL2() const { return l2_length_; }

private:
  // Arm physical parameters
  double l1_length_;  // Upper arm length
  double l2_length_;  // Lower arm length

  // Joint limits and conversions
  static constexpr size_t NUM_JOINT_TYPES = 6;
  std::array<JointLimits, NUM_JOINT_TYPES> joint_limits_;
  std::array<double, NUM_JOINT_TYPES> joint_conversions_;

  // Helper functions
  bool computeIKSolution(
    const Pose2D & target,
    ElbowConfig config,
    IKSolution & solution) const;

  double adjustJointAngle(double angle, JointType joint_type) const;
  bool isWithinLimits(double angle, JointType joint_type) const;
  double wrapAngle(double angle) const;
  double computeEndEffectorRotation(
    double j1, double j2, double target_rotation) const;

  // Validation
  bool validateReachability(double x, double y) const;
  bool isNearSingularity(double x, double y, double tolerance = 1e-6) const;
};

}  // namespace cobo_arm_control

#endif  // COBO_ARM_CONTROL__IK_SOLVER_HPP_
