// Test program for IK Solver
// Usage: ./test_ik_solver <x> <y> <rotation> [l1] [l2] [elbow_config]
//   x, y        - Target position coordinates
//   rotation    - Target end effector rotation (radians)
//   l1, l2      - Optional arm segment lengths (default: 0.3, 0.25)
//   elbow_config - Optional: "up", "down", or "any" (default: any)

#include "cobo_arm_control/ik_solver.hpp"
#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <cstring>
#include <cmath>

void printUsage(const char* program_name)
{
  std::cout << "Usage: " << program_name << " <x> <y> <rotation> [l1] [l2] [elbow_config]\n"
            << "\n"
            << "Required arguments:\n"
            << "  x           Target X position\n"
            << "  y           Target Y position\n"
            << "  rotation    Target end effector rotation (radians)\n"
            << "\n"
            << "Optional arguments:\n"
            << "  l1          Upper arm length (default: 0.3)\n"
            << "  l2          Lower arm length (default: 0.25)\n"
            << "  elbow_config  Elbow configuration: 'up', 'down', or 'any' (default: any)\n"
            << "\n"
            << "Examples:\n"
            << "  " << program_name << " 0.4 0.2 0.0\n"
            << "  " << program_name << " 0.4 0.2 1.57 0.3 0.25 up\n"
            << std::endl;
}

std::string elbowConfigToString(cobo_arm_control::ElbowConfig config)
{
  return config == cobo_arm_control::ElbowConfig::UP ? "UP" : "DOWN";
}

double radToDeg(double rad)
{
  return rad * 180.0 / M_PI;
}

int main(int argc, char* argv[])
{
  if (argc < 4) {
    printUsage(argv[0]);
    return 1;
  }

  // Parse required arguments
  double x = std::atof(argv[1]);
  double y = std::atof(argv[2]);
  double rotation = std::atof(argv[3]);

  // Parse optional arguments
  double l1 = (argc > 4) ? std::atof(argv[4]) : 0.3;
  double l2 = (argc > 5) ? std::atof(argv[5]) : 0.25;

  std::optional<cobo_arm_control::ElbowConfig> preferred_config = std::nullopt;
  if (argc > 6) {
    if (std::strcmp(argv[6], "up") == 0) {
      preferred_config = cobo_arm_control::ElbowConfig::UP;
    } else if (std::strcmp(argv[6], "down") == 0) {
      preferred_config = cobo_arm_control::ElbowConfig::DOWN;
    }
    // "any" or invalid leaves it as nullopt
  }

  // Print input parameters
  std::cout << "========================================\n";
  std::cout << "IK Solver Test\n";
  std::cout << "========================================\n";
  std::cout << "\nInput Parameters:\n";
  std::cout << "  Target Position: (" << x << ", " << y << ")\n";
  std::cout << "  Target Rotation: " << rotation << " rad (" << radToDeg(rotation) << " deg)\n";
  std::cout << "  Arm Lengths: L1=" << l1 << ", L2=" << l2 << "\n";
  std::cout << "  Preferred Elbow Config: ";
  if (preferred_config.has_value()) {
    std::cout << elbowConfigToString(preferred_config.value()) << "\n";
  } else {
    std::cout << "ANY\n";
  }
  std::cout << "\n";

  // Create solver and compute IK
  cobo_arm_control::IKSolver solver(l1, l2);

  cobo_arm_control::Pose2D target{x, y, rotation};
  auto result = solver.solve(target, cobo_arm_control::ArmSide::RIGHT, preferred_config);

  // Display results
  std::cout << "========================================\n";
  std::cout << "Results:\n";
  std::cout << "========================================\n";

  if (result.has_value()) {
    const auto& solution = result.value();
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "\nSolution FOUND!\n\n";
    std::cout << "  Elbow Configuration: " << elbowConfigToString(solution.elbow_config) << "\n";
    std::cout << "\n  Joint Angles:\n";
    std::cout << "    Joint 1: " << std::setw(8) << solution.joint_1 << " rad ("
              << std::setw(8) << radToDeg(solution.joint_1) << " deg)\n";
    std::cout << "    Joint 2: " << std::setw(8) << solution.joint_2 << " rad ("
              << std::setw(8) << radToDeg(solution.joint_2) << " deg)\n";
    std::cout << "    Joint 3: " << std::setw(8) << solution.joint_3 << " rad ("
              << std::setw(8) << radToDeg(solution.joint_3) << " deg)\n";

    // Verify by computing forward kinematics
    std::cout << "\n  Forward Kinematics Verification:\n";
    // Note: This is simplified FK - actual FK would need to account for joint conversions
    double fk_x = l1 * std::cos(solution.joint_1) + l2 * std::cos(solution.joint_1 + solution.joint_2);
    double fk_y = l1 * std::sin(solution.joint_1) + l2 * std::sin(solution.joint_1 + solution.joint_2);
    std::cout << "    Computed Position: (" << fk_x << ", " << fk_y << ")\n";
    std::cout << "    (Note: FK verification is approximate due to joint conversions)\n";
  } else {
    std::cout << "\nNo valid IK solution found!\n";
    std::cout << "\nPossible reasons:\n";
    std::cout << "  - Target is out of reach (too far or too close)\n";
    std::cout << "  - Joint limits prevent reaching the target\n";

    double distance = std::sqrt(x * x + y * y);
    double max_reach = l1 + l2;
    double min_reach = std::abs(l1 - l2);
    std::cout << "\n  Reachability Analysis:\n";
    std::cout << "    Target distance: " << distance << "\n";
    std::cout << "    Max reach: " << max_reach << "\n";
    std::cout << "    Min reach: " << min_reach << "\n";
    if (distance > max_reach) {
      std::cout << "    -> Target is TOO FAR (beyond max reach)\n";
    } else if (distance < min_reach) {
      std::cout << "    -> Target is TOO CLOSE (inside min reach)\n";
    } else {
      std::cout << "    -> Target is within reach, but joint limits may prevent solution\n";
    }
  }

  std::cout << "\n========================================\n";
  return result.has_value() ? 0 : 1;
}
