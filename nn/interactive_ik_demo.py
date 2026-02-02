#!/usr/bin/env python3
"""
Interactive demo for the inverse kinematics network.
Click on the plot to set target positions and see the IK solution in real-time.
"""

import torch
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import argparse
import sys

from inverse_kinematics_network import (
    InverseKinematicsNetwork,
    ForwardKinematics,
    compute_pose_error
)


class InteractiveIKDemo:
    """Interactive IK demonstration with matplotlib."""

    def __init__(self, model, link_lengths, device='cpu'):
        self.model = model
        self.link_lengths = link_lengths
        self.device = device
        self.total_reach = sum(link_lengths)

        # Current state
        self.current_target = torch.tensor([[0.0, self.total_reach * 0.5]])
        self.current_angles = None
        self.update_angles()

        # Setup plot
        self.setup_plot()

    def setup_plot(self):
        """Initialize the matplotlib figure and axes."""
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.fig.canvas.manager.set_window_title('Interactive IK Demo - Click to set target')

        # Initialize plot elements
        self.arm_line, = self.ax.plot([], [], 'b-o', linewidth=3, markersize=10,
                                       label='Robot Arm', zorder=5)
        self.target_marker, = self.ax.plot([], [], 'g*', markersize=25,
                                            label='Target', zorder=10)
        self.achieved_marker, = self.ax.plot([], [], 'rx', markersize=15,
                                              markeredgewidth=3, label='Achieved', zorder=10)

        # Workspace circle
        workspace_circle = Circle((0, 0), self.total_reach, fill=False,
                                 edgecolor='gray', linestyle='--', linewidth=2,
                                 label='Max Reach')
        self.ax.add_patch(workspace_circle)

        # Min reach circle (if applicable)
        min_reach = abs(self.link_lengths[0] - self.link_lengths[1])
        if min_reach > 0.01:
            min_workspace_circle = Circle((0, 0), min_reach, fill=False,
                                         edgecolor='gray', linestyle=':', linewidth=2,
                                         label='Min Reach')
            self.ax.add_patch(min_workspace_circle)

        # Base marker
        self.ax.plot(0, 0, 'ko', markersize=15, zorder=5, label='Base')

        # Text info
        self.info_text = self.ax.text(0.02, 0.98, '', transform=self.ax.transAxes,
                                      verticalalignment='top', fontsize=10,
                                      bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

        # Formatting
        self.ax.set_xlabel('X Position', fontsize=12)
        self.ax.set_ylabel('Y Position', fontsize=12)
        self.ax.set_title('Interactive Inverse Kinematics Demo\nClick anywhere to set target position',
                         fontsize=14, fontweight='bold')
        self.ax.legend(loc='upper right', fontsize=10)
        self.ax.grid(True, alpha=0.3)
        self.ax.set_aspect('equal')

        # Set limits
        margin = self.total_reach * 1.2
        self.ax.set_xlim(-margin, margin)
        self.ax.set_ylim(-margin, margin)

        # Connect click event
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)

        # Initial update
        self.update_plot()

    def update_angles(self):
        """Compute IK solution for current target."""
        self.model.eval()
        with torch.no_grad():
            target = self.current_target.to(self.device)
            self.current_angles = self.model(target).cpu()

    def update_plot(self):
        """Update the plot with current state."""
        # Compute forward kinematics
        _, joint_positions = ForwardKinematics.compute(self.current_angles, self.link_lengths)

        # Extract coordinates
        x_coords = [0.0]
        y_coords = [0.0]
        for joint_pos in joint_positions[1:]:
            x_coords.append(joint_pos[0, 0].item())
            y_coords.append(joint_pos[0, 1].item())

        # Update arm line
        self.arm_line.set_data(x_coords, y_coords)

        # Update target marker
        self.target_marker.set_data([self.current_target[0, 0].item()],
                                    [self.current_target[0, 1].item()])

        # Update achieved position marker
        achieved_pos = ForwardKinematics.compute_simple(self.current_angles, self.link_lengths)
        self.achieved_marker.set_data([achieved_pos[0, 0].item()],
                                      [achieved_pos[0, 1].item()])

        # Compute error (position only, since target is 2D)
        # Add a dummy rotation component (0) to the target for compatibility
        target_pose = torch.cat([self.current_target, torch.zeros(1, 1)], dim=1)
        position_error, _ = compute_pose_error(self.current_angles, target_pose,
                                              self.link_lengths)
        error = position_error.item()

        # Update info text
        target_x = self.current_target[0, 0].item()
        target_y = self.current_target[0, 1].item()
        theta1 = self.current_angles[0, 0].item()
        theta2 = self.current_angles[0, 1].item()
        theta3 = self.current_angles[0, 2].item()

        info_str = (
            f"Target: ({target_x:.3f}, {target_y:.3f})\n"
            f"Joint Angles (rad): [{theta1:.3f}, {theta2:.3f}, {theta3:.3f}]\n"
            f"Joint Angles (deg): [{np.rad2deg(theta1):.1f}°, "
            f"{np.rad2deg(theta2):.1f}°, {np.rad2deg(theta3):.1f}°]\n"
            f"Position Error: {error:.6f}\n"
            f"\nPress 'r' for random target\n"
            f"Press 'q' to quit"
        )
        self.info_text.set_text(info_str)

        # Redraw
        self.fig.canvas.draw()

    def on_click(self, event):
        """Handle mouse click events."""
        if event.inaxes != self.ax:
            return

        # Get clicked position
        x, y = event.xdata, event.ydata

        # Check if within workspace
        distance = np.sqrt(x**2 + y**2)
        if distance > self.total_reach:
            print(f"Target ({x:.2f}, {y:.2f}) is outside workspace (distance: {distance:.2f} > {self.total_reach:.2f})")
            return

        # Update target
        self.current_target = torch.tensor([[x, y]], dtype=torch.float32)
        self.update_angles()
        self.update_plot()

        print(f"New target: ({x:.3f}, {y:.3f})")

    def on_key(self, event):
        """Handle keyboard events."""
        if event.key == 'q':
            print("Exiting...")
            plt.close(self.fig)
            sys.exit(0)

        elif event.key == 'r':
            # Random target within workspace
            angle = np.random.uniform(0, 2*np.pi)
            radius = np.random.uniform(0, self.total_reach * 0.9)
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)

            self.current_target = torch.tensor([[x, y]], dtype=torch.float32)
            self.update_angles()
            self.update_plot()

            print(f"Random target: ({x:.3f}, {y:.3f})")

    def run(self):
        """Start the interactive demo."""
        plt.show()


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


def main():
    parser = argparse.ArgumentParser(
        description='Interactive Inverse Kinematics Demo',
        epilog='Click on the plot to set target positions. Press "r" for random target, "q" to quit.'
    )

    parser.add_argument(
        '--checkpoint', '-c',
        type=str,
        default='checkpoints_ik/best_model.pth',
        help='Path to model checkpoint (default: checkpoints_ik/best_model.pth)'
    )

    args = parser.parse_args()

    # Device
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(f"Using device: {device}")

    # Load model
    print(f"Loading model from: {args.checkpoint}")
    try:
        model, link_lengths = load_model(args.checkpoint, device)
    except FileNotFoundError:
        print(f"\nError: Checkpoint file not found: {args.checkpoint}")
        print("\nPlease train the model first using:")
        print("  python train_inverse_kinematics.py")
        print("\nOr specify a different checkpoint with --checkpoint")
        sys.exit(1)

    # Print info
    print("\n" + "="*60)
    print("Interactive IK Demo")
    print("="*60)
    print(f"Link lengths: L1={link_lengths[0]}, L2={link_lengths[1]}, L3={link_lengths[2]}")
    print(f"Maximum reach: {sum(link_lengths):.2f}")
    print(f"Minimum reach: {abs(link_lengths[0] - link_lengths[1] - link_lengths[2]):.2f}")
    print("\nControls:")
    print("  - Click anywhere to set target position")
    print("  - Press 'r' for random target")
    print("  - Press 'q' to quit")
    print("="*60 + "\n")

    # Create and run demo
    demo = InteractiveIKDemo(model, link_lengths, device)
    demo.run()


if __name__ == "__main__":
    main()
