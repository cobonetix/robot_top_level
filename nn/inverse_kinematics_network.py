import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np


class InverseKinematicsNetwork(nn.Module):
    """
    Neural network for inverse kinematics of a 2-link planar arm with end-effector rotation.

    The network learns to map from end-effector pose (x, y, φ) to joint angles (θ1, θ2, θ3).
    - Links 1 and 2 control position (x, y)
    - Joint 3 controls end-effector rotation (φ)

    Inputs:
        - target_pose: Target end-effector pose (B, 3) - [x, y, φ]
        - (optional) link_lengths: Link lengths (B, 2) - [L1, L2] if they vary

    Output:
        - joint_angles: Predicted joint angles (B, 3) - [θ1, θ2, θ3] in radians
    """

    def __init__(self, link_lengths=(1.0, 1.0), input_link_lengths=False, hidden_dims=[128, 256, 256, 128]):
        """
        Args:
            link_lengths: Default link lengths [L1, L2]
            input_link_lengths: If True, link lengths are provided as input (for variable geometry)
            hidden_dims: List of hidden layer dimensions
        """
        super(InverseKinematicsNetwork, self).__init__()

        self.link_lengths = torch.tensor(link_lengths, dtype=torch.float32)
        self.input_link_lengths = input_link_lengths

        # Calculate input dimension
        input_dim = 3  # x, y position, φ rotation
        if input_link_lengths:
            input_dim += 2  # L1, L2

        # Build the network
        layers = []
        prev_dim = input_dim

        for hidden_dim in hidden_dims:
            layers.extend([
                nn.Linear(prev_dim, hidden_dim),
                nn.ReLU(),
                nn.Dropout(0.1)
            ])
            prev_dim = hidden_dim

        # Output layer - 3 joint angles
        layers.append(nn.Linear(prev_dim, 3))

        self.network = nn.Sequential(*layers)

        # Initialize weights
        self._init_weights()

    def _init_weights(self):
        """Initialize network weights."""
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.xavier_uniform_(m.weight)
                if m.bias is not None:
                    nn.init.zeros_(m.bias)

    def forward(self, target_pose, link_lengths=None):
        """
        Forward pass.

        Args:
            target_pose: Tensor of shape (B, 3) - [x, y, φ] coordinates
            link_lengths: Optional tensor of shape (B, 2) - [L1, L2]

        Returns:
            joint_angles: Tensor of shape (B, 3) - [θ1, θ2, θ3] in radians
        """
        if self.input_link_lengths:
            if link_lengths is None:
                # Use default link lengths
                batch_size = target_pose.size(0)
                link_lengths = self.link_lengths.unsqueeze(0).expand(batch_size, -1).to(target_pose.device)

            # Concatenate pose and link lengths
            network_input = torch.cat([target_pose, link_lengths], dim=1)
        else:
            network_input = target_pose

        # Forward pass through network
        joint_angles = self.network(network_input)

        return joint_angles


class ForwardKinematics:
    """
    Forward kinematics for a 2-link planar arm with end-effector rotation.
    Joint 3 controls end-effector rotation, not position.
    Used for generating training data and validation.
    """

    @staticmethod
    def compute(joint_angles, link_lengths=(1.0, 1.0)):
        """
        Compute forward kinematics for a 2-link planar arm with end-effector rotation.

        Args:
            joint_angles: Tensor or array of shape (..., 3) - [θ1, θ2, θ3]
            link_lengths: Tuple of link lengths [L1, L2]

        Returns:
            end_effector_pose: Tensor or array of shape (..., 3) - [x, y, φ]
            joint_positions: List of joint positions for visualization
        """
        if isinstance(joint_angles, torch.Tensor):
            is_torch = True
            cos = torch.cos
            sin = torch.sin
        else:
            is_torch = False
            joint_angles = np.array(joint_angles)
            cos = np.cos
            sin = np.sin

        L1, L2 = link_lengths
        theta1 = joint_angles[..., 0]
        theta2 = joint_angles[..., 1]
        theta3 = joint_angles[..., 2]  # End-effector rotation

        # Cumulative angles for position (only first 2 joints)
        angle1 = theta1
        angle2 = theta1 + theta2

        # Joint positions
        x1 = L1 * cos(angle1)
        y1 = L1 * sin(angle1)

        # End-effector position (wrist)
        x2 = x1 + L2 * cos(angle2)
        y2 = y1 + L2 * sin(angle2)

        # End-effector orientation
        phi = angle2 + theta3

        # Stack end-effector pose [x, y, φ]
        if is_torch:
            end_effector = torch.stack([x2, y2, phi], dim=-1)
            joint_positions = [
                torch.zeros_like(x1),  # Base at origin
                torch.stack([x1, y1], dim=-1),
                torch.stack([x2, y2], dim=-1)
            ]
        else:
            end_effector = np.stack([x2, y2, phi], axis=-1)
            joint_positions = [
                np.zeros_like(x1),
                np.stack([x1, y1], axis=-1),
                np.stack([x2, y2], axis=-1)
            ]

        return end_effector, joint_positions

    @staticmethod
    def compute_simple(joint_angles, link_lengths=(1.0, 1.0)):
        """
        Simplified version that only returns end-effector pose.

        Args:
            joint_angles: Tensor of shape (..., 3) - [θ1, θ2, θ3]
            link_lengths: Tuple of link lengths [L1, L2]

        Returns:
            end_effector_pose: Tensor of shape (..., 3) - [x, y, φ]
        """
        end_effector, _ = ForwardKinematics.compute(joint_angles, link_lengths)
        return end_effector


def generate_random_configurations(num_samples, link_lengths=(1.0, 1.0),
                                   angle_range=(-np.pi, np.pi)):
    """
    Generate random joint configurations and their corresponding end-effector poses.

    Args:
        num_samples: Number of samples to generate
        link_lengths: Tuple of link lengths [L1, L2]
        angle_range: Tuple of (min_angle, max_angle) for each joint

    Returns:
        joint_angles: Tensor of shape (num_samples, 3)
        end_effector_poses: Tensor of shape (num_samples, 3) - [x, y, φ]
    """
    # Generate random joint angles
    min_angle, max_angle = angle_range
    joint_angles = torch.rand(num_samples, 3) * (max_angle - min_angle) + min_angle

    # Compute forward kinematics
    end_effector_poses = ForwardKinematics.compute_simple(joint_angles, link_lengths)

    return joint_angles, end_effector_poses


def compute_pose_error(predicted_angles, target_poses, link_lengths=(1.0, 1.0)):
    """
    Compute the pose error between predicted joint angles and target poses.

    Args:
        predicted_angles: Tensor of shape (B, 3) - predicted joint angles
        target_poses: Tensor of shape (B, 3) - target end-effector poses [x, y, φ]
        link_lengths: Tuple of link lengths

    Returns:
        position_error: Tensor of shape (B,) - Euclidean distance error for position
        rotation_error: Tensor of shape (B,) - Absolute angular error for rotation
    """
    predicted_poses = ForwardKinematics.compute_simple(predicted_angles, link_lengths)

    # Position error (x, y)
    position_error = torch.norm(predicted_poses[:, :2] - target_poses[:, :2], dim=1)

    # Rotation error (φ) - handle wraparound
    rotation_diff = predicted_poses[:, 2] - target_poses[:, 2]
    rotation_error = torch.abs(torch.atan2(torch.sin(rotation_diff), torch.cos(rotation_diff)))

    return position_error, rotation_error


# Example usage and testing
if __name__ == "__main__":
    print("=" * 60)
    print("2-Link Planar Arm Inverse Kinematics Network")
    print("(with End-Effector Rotation)")
    print("=" * 60)

    # Configuration
    link_lengths = (1.0, 1.0)
    batch_size = 8

    # Initialize network
    print("\nInitializing network...")
    model = InverseKinematicsNetwork(
        link_lengths=link_lengths,
        input_link_lengths=False,
        hidden_dims=[128, 256, 256, 128]
    )

    # Count parameters
    total_params = sum(p.numel() for p in model.parameters())
    print(f"Total parameters: {total_params:,}")

    # Generate test data
    print("\nGenerating test configurations...")
    num_test = 5
    test_angles, test_poses = generate_random_configurations(
        num_test,
        link_lengths=link_lengths,
        angle_range=(-np.pi, np.pi)
    )

    print(f"\nTest Configurations:")
    print(f"{'Joint Angles (rad)':<40} {'End-Effector Pose [x, y, φ]':<30}")
    print("-" * 70)
    for i in range(num_test):
        angles_str = f"[{test_angles[i, 0]:.3f}, {test_angles[i, 1]:.3f}, {test_angles[i, 2]:.3f}]"
        pose_str = f"[{test_poses[i, 0]:.3f}, {test_poses[i, 1]:.3f}, {test_poses[i, 2]:.3f}]"
        print(f"{angles_str:<40} {pose_str:<30}")

    # Test forward pass (untrained)
    print("\nTesting forward pass (untrained network)...")
    model.eval()
    with torch.no_grad():
        predicted_angles = model(test_poses)
        predicted_poses = ForwardKinematics.compute_simple(predicted_angles, link_lengths)
        pos_errors, rot_errors = compute_pose_error(predicted_angles, test_poses, link_lengths)

    print(f"\nPredictions (untrained):")
    print(f"{'Target Pose':<25} {'Predicted Angles (rad)':<40} {'Achieved Pose':<25} {'Pos Err':<10} {'Rot Err':<10}")
    print("-" * 110)
    for i in range(num_test):
        target_str = f"[{test_poses[i, 0]:.3f}, {test_poses[i, 1]:.3f}, {test_poses[i, 2]:.3f}]"
        pred_angles_str = f"[{predicted_angles[i, 0]:.3f}, {predicted_angles[i, 1]:.3f}, {predicted_angles[i, 2]:.3f}]"
        achieved_str = f"[{predicted_poses[i, 0]:.3f}, {predicted_poses[i, 1]:.3f}, {predicted_poses[i, 2]:.3f}]"
        pos_err_str = f"{pos_errors[i]:.4f}"
        rot_err_str = f"{rot_errors[i]:.4f}"
        print(f"{target_str:<25} {pred_angles_str:<40} {achieved_str:<25} {pos_err_str:<10} {rot_err_str:<10}")

    print(f"\nMean position error (untrained): {pos_errors.mean():.4f}")
    print(f"Mean rotation error (untrained): {rot_errors.mean():.4f}")

    # Test workspace coverage
    print("\n" + "=" * 60)
    print("Workspace Analysis")
    print("=" * 60)

    num_workspace_samples = 10000
    workspace_angles, workspace_poses = generate_random_configurations(
        num_workspace_samples,
        link_lengths=link_lengths
    )

    x_coords = workspace_poses[:, 0].numpy()
    y_coords = workspace_poses[:, 1].numpy()
    phi_coords = workspace_poses[:, 2].numpy()

    total_reach = sum(link_lengths)

    print(f"\nLink lengths: L1={link_lengths[0]}, L2={link_lengths[1]}")
    print(f"Maximum reach: {total_reach:.2f}")
    print(f"Minimum reach: {abs(link_lengths[0] - link_lengths[1]):.2f}")
    print(f"\nWorkspace coverage (from {num_workspace_samples:,} samples):")
    print(f"  X range: [{x_coords.min():.3f}, {x_coords.max():.3f}]")
    print(f"  Y range: [{y_coords.min():.3f}, {y_coords.max():.3f}]")
    print(f"  φ range: [{phi_coords.min():.3f}, {phi_coords.max():.3f}]")

    print("\n" + "=" * 60)
    print("Network ready for training!")
    print("Use train_inverse_kinematics.py to train the network.")
    print("=" * 60)
