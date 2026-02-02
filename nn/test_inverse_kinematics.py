#!/usr/bin/env python3
"""
Test and visualize the trained inverse kinematics network.
"""

import torch
import numpy as np
import matplotlib.pyplot as plt
import argparse
import os

from inverse_kinematics_network import (
    InverseKinematicsNetwork,
    ForwardKinematics,
    generate_random_configurations,
    compute_pose_error
)


def load_model(checkpoint_path, device='cpu'):
    """Load a trained model from checkpoint."""
    checkpoint = torch.load(checkpoint_path, map_location=device)

    # Extract link lengths from checkpoint
    link_lengths = checkpoint.get('link_lengths', (1.0, 1.0))

    # Initialize model
    model = InverseKinematicsNetwork(
        link_lengths=link_lengths,
        input_link_lengths=False,
        hidden_dims=[128, 256, 256, 128]
    )

    # Load weights
    model.load_state_dict(checkpoint['model_state_dict'])
    model.to(device)
    model.eval()

    print(f"Loaded model from epoch {checkpoint['epoch']}")
    print(f"Link lengths: {link_lengths}")

    return model, link_lengths


def visualize_arm_configuration(ax, joint_angles, link_lengths, color='blue',
                                linestyle='-', marker='o', label=None, linewidth=2):
    """
    Visualize a single arm configuration.

    Args:
        ax: Matplotlib axis
        joint_angles: Tensor or array of shape (3,) - [θ1, θ2, θ3]
        link_lengths: Tuple of link lengths
        color: Line color
        linestyle: Line style
        marker: Marker style
        label: Legend label
        linewidth: Line width
    """
    if isinstance(joint_angles, torch.Tensor):
        joint_angles = joint_angles.cpu().numpy()

    # Reshape if needed
    if joint_angles.ndim == 1:
        joint_angles = joint_angles.reshape(1, -1)

    # Compute forward kinematics
    _, joint_positions = ForwardKinematics.compute(joint_angles, link_lengths)

    # Extract positions
    x_coords = [0.0]
    y_coords = [0.0]

    for joint_pos in joint_positions[1:]:
        if isinstance(joint_pos, torch.Tensor):
            joint_pos = joint_pos.cpu().numpy()
        x_coords.append(joint_pos[0, 0])
        y_coords.append(joint_pos[0, 1])

    # Plot
    ax.plot(x_coords, y_coords, color=color, linestyle=linestyle,
            marker=marker, linewidth=linewidth, markersize=8, label=label)


def test_random_targets(model, link_lengths, num_tests=10, device='cpu'):
    """Test the model on random target positions."""
    print(f"\nTesting on {num_tests} random targets...")
    print("=" * 80)

    # Generate random test cases
    ground_truth_angles, target_positions = generate_random_configurations(
        num_tests, link_lengths
    )

    # Predict joint angles
    model.eval()
    with torch.no_grad():
        target_positions = target_positions.to(device)
        predicted_angles = model(target_positions).cpu()

    # Compute errors
    # Add dummy rotation component (0) to target positions since they are 2D
    target_poses = torch.cat([target_positions.cpu(), torch.zeros(num_tests, 1)], dim=1)
    position_errors, rotation_errors = compute_pose_error(predicted_angles, target_poses, link_lengths)

    # Print results
    print(f"{'Target Position':<20} {'Predicted Angles (rad)':<40} {'Position Error':<15}")
    print("-" * 80)


    for i in range(num_tests):
        target_str = f"[{target_positions[i, 0]:.3f}, {target_positions[i, 1]:.3f}]"
        angles_str = f"[{predicted_angles[i, 0]:.3f}, {predicted_angles[i, 1]:.3f}, {predicted_angles[i, 2]:.3f}]"
        error_str = f"{position_errors[i]:.6f}"
        print(f"{target_str:<20} {angles_str:<40} {error_str:<15}")

   # print("-" * 80)
    #print(f"Mean error: {position_errors.mean():.6f}")
    #print(f"Max error:  {position_errors.max():.6f}")
    #print(f"Min error:  {position_errors.min():.6f}")
    #print(f"Std error:  {position_errors.std():.6f}")

    return predicted_angles, target_positions.cpu(), ground_truth_angles, position_errors


def visualize_test_results(predicted_angles, target_positions, ground_truth_angles,
                           errors, link_lengths, num_display=6):
    """Create a visualization of test results."""
    num_display = min(num_display, len(predicted_angles))

    # Create figure
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    axes = axes.flatten()

    for i in range(num_display):
        ax = axes[i]

        # Plot ground truth configuration
        visualize_arm_configuration(
            ax, ground_truth_angles[i], link_lengths,
            color='blue', linestyle='-', marker='o',
            label='Ground Truth', linewidth=2
        )

        # Plot predicted configuration
        visualize_arm_configuration(
            ax, predicted_angles[i], link_lengths,
            color='red', linestyle='--', marker='^',
            label='Predicted', linewidth=2
        )

        # Mark target position
        ax.plot(target_positions[i, 0], target_positions[i, 1],
                'g*', markersize=20, label='Target', zorder=10)

        # Formatting
        ax.set_xlabel('X Position', fontsize=10)
        ax.set_ylabel('Y Position', fontsize=10)
        ax.set_title(f'Test {i+1} - Error: {errors[i]:.4f}', fontsize=11, fontweight='bold')
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.axis('equal')

        # Set limits
        total_reach = sum(link_lengths)
        ax.set_xlim(-total_reach*1.1, total_reach*1.1)
        ax.set_ylim(-total_reach*1.1, total_reach*1.1)

        # Add origin marker
        ax.plot(0, 0, 'ko', markersize=10, zorder=5)

    plt.tight_layout()
    return fig


def test_specific_target(model, link_lengths, target_x, target_y, device='cpu'):
    """Test the model on a specific target position."""
    print(f"\nTesting specific target: ({target_x}, {target_y})")
    print("=" * 60)

    # Create target tensor
    target = torch.tensor([[target_x, target_y]], dtype=torch.float32).to(device)

    # Predict
    model.eval()
    with torch.no_grad():
        predicted_angles = model(target).cpu()

    # Compute achieved position
    achieved_pos = ForwardKinematics.compute_simple(predicted_angles, link_lengths)
    # Add dummy rotation component (0) to target since it's 2D
    target_pose = torch.cat([target.cpu(), torch.zeros(1, 1)], dim=1)
    position_error, rotation_error = compute_pose_error(predicted_angles, target_pose, link_lengths)

    print(f"Target position:    [{target_x:.4f}, {target_y:.4f}]")
    print(f"Predicted angles:   [{predicted_angles[0, 0]:.4f}, {predicted_angles[0, 1]:.4f}, {predicted_angles[0, 2]:.4f}] rad")
    print(f"Predicted angles:   [{np.rad2deg(predicted_angles[0, 0].item()):.2f}, "
          f"{np.rad2deg(predicted_angles[0, 1].item()):.2f}, "
          f"{np.rad2deg(predicted_angles[0, 2].item()):.2f}] deg")
    print(f"Achieved position:  [{achieved_pos[0, 0]:.4f}, {achieved_pos[0, 1]:.4f}]")
    print(f"Position error:     {position_error[0]:.6f}")

    # Visualize
    fig, ax = plt.subplots(1, 1, figsize=(8, 8))

    visualize_arm_configuration(
        ax, predicted_angles[0], link_lengths,
        color='blue', linestyle='-', marker='o',
        label='Predicted Configuration', linewidth=3
    )

    ax.plot(target_x, target_y, 'g*', markersize=25, label='Target', zorder=10)
    ax.plot(achieved_pos[0, 0], achieved_pos[0, 1], 'rx', markersize=15,
            markeredgewidth=3, label='Achieved', zorder=10)

    # Formatting
    ax.set_xlabel('X Position', fontsize=12)
    ax.set_ylabel('Y Position', fontsize=12)
    ax.set_title(f'IK Solution for Target ({target_x:.2f}, {target_y:.2f})\n'
                 f'Error: {position_error[0]:.6f}', fontsize=14, fontweight='bold')
    ax.legend(loc='upper right', fontsize=11)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')

    # Set limits
    total_reach = sum(link_lengths)
    ax.set_xlim(-total_reach*1.1, total_reach*1.1)
    ax.set_ylim(-total_reach*1.1, total_reach*1.1)

    # Add origin
    ax.plot(0, 0, 'ko', markersize=12, zorder=5)

    plt.tight_layout()

    return fig, predicted_angles


def evaluate_workspace(model, link_lengths, num_samples=10000, device='cpu'):
    """Evaluate model performance across the workspace."""
    print(f"\nEvaluating workspace performance with {num_samples} samples...")

    # Generate test samples across workspace
    _, target_positions = generate_random_configurations(num_samples, link_lengths)

    # Predict in batches
    batch_size = 256
    all_errors = []

    model.eval()
    with torch.no_grad():
        for i in range(0, num_samples, batch_size):
            batch = target_positions[i:i+batch_size].to(device)
            predicted = model(batch).cpu()
            # Add dummy rotation component (0) to target positions since they are 2D
            batch_targets = target_positions[i:i+batch_size]
            target_poses = torch.cat([batch_targets, torch.zeros(len(batch_targets), 1)], dim=1)
            position_errors, _ = compute_pose_error(predicted, target_poses, link_lengths)
            all_errors.append(position_errors)

    all_errors = torch.cat(all_errors)

    # Statistics
    print("\nWorkspace Performance Statistics:")
    print("=" * 60)
    print(f"Samples tested:     {num_samples}")
    print(f"Mean error:         {all_errors.mean():.6f}")
    print(f"Median error:       {all_errors.median():.6f}")
    print(f"Std error:          {all_errors.std():.6f}")
    print(f"Min error:          {all_errors.min():.6f}")
    print(f"Max error:          {all_errors.max():.6f}")
    print(f"95th percentile:    {torch.quantile(all_errors, 0.95):.6f}")
    print(f"99th percentile:    {torch.quantile(all_errors, 0.99):.6f}")

    # Create error distribution plot
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))

    # Histogram
    ax1.hist(all_errors.numpy(), bins=50, edgecolor='black', alpha=0.7)
    ax1.axvline(all_errors.mean(), color='red', linestyle='--', linewidth=2, label='Mean')
    ax1.axvline(all_errors.median(), color='green', linestyle='--', linewidth=2, label='Median')
    ax1.set_xlabel('Position Error', fontsize=12)
    ax1.set_ylabel('Frequency', fontsize=12)
    ax1.set_title('Error Distribution', fontsize=14, fontweight='bold')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Scatter plot of errors in workspace
    target_pos_np = target_positions.numpy()
    ax2.scatter(target_pos_np[:, 0], target_pos_np[:, 1],
                c=all_errors.numpy(), cmap='jet', s=1, alpha=0.5)
    cbar = plt.colorbar(ax2.collections[0], ax=ax2)
    cbar.set_label('Position Error', fontsize=10)
    ax2.set_xlabel('X Position', fontsize=12)
    ax2.set_ylabel('Y Position', fontsize=12)
    ax2.set_title('Error Distribution in Workspace', fontsize=14, fontweight='bold')
    ax2.axis('equal')
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()

    return fig, all_errors


def main():
    parser = argparse.ArgumentParser(description='Test Inverse Kinematics Network')

    parser.add_argument(
        '--checkpoint', '-c',
        type=str,
        default='checkpoints_ik/best_model.pth',
        help='Path to model checkpoint'
    )

    parser.add_argument(
        '--num-tests', '-n',
        type=int,
        default=10,
        help='Number of random tests to run'
    )

    parser.add_argument(
        '--target', '-t',
        type=float,
        nargs=2,
        metavar=('X', 'Y'),
        help='Test specific target position (e.g., --target 1.5 1.0)'
    )

    parser.add_argument(
        '--workspace',
        action='store_true',
        help='Evaluate performance across entire workspace'
    )

    parser.add_argument(
        '--output-dir', '-o',
        type=str,
        default='test_results',
        help='Output directory for visualizations'
    )

    args = parser.parse_args()

    # Create output directory
    os.makedirs(args.output_dir, exist_ok=True)

    # Device
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(f"Using device: {device}")

    # Load model
    print(f"\nLoading model from: {args.checkpoint}")
    model, link_lengths = load_model(args.checkpoint, device)

    # Test random targets
    predicted, targets, gt_angles, errors = test_random_targets(
        model, link_lengths, args.num_tests, device
    )

    # Visualize
    fig = visualize_test_results(predicted, targets, gt_angles, errors, link_lengths)
    output_path = os.path.join(args.output_dir, 'random_tests.png')
    fig.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\nSaved random tests visualization to: {output_path}")

    # Test specific target if provided
    if args.target is not None:
        target_x, target_y = args.target
        fig, pred_angles = test_specific_target(model, link_lengths, target_x, target_y, device)
        output_path = os.path.join(args.output_dir, f'specific_target_{target_x}_{target_y}.png')
        fig.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"\nSaved specific target visualization to: {output_path}")

    # Evaluate workspace if requested
    if args.workspace:
        fig, workspace_errors = evaluate_workspace(model, link_lengths, num_samples=10000, device=device)
        output_path = os.path.join(args.output_dir, 'workspace_evaluation.png')
        fig.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"\nSaved workspace evaluation to: {output_path}")

    print("\nTesting complete!")
    print(f"All visualizations saved to: {args.output_dir}")


if __name__ == "__main__":
    main()
