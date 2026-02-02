import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
from torch.utils.tensorboard import SummaryWriter
import numpy as np
import os
from datetime import datetime
from tqdm import tqdm
import matplotlib.pyplot as plt

from inverse_kinematics_network import (
    InverseKinematicsNetwork,
    ForwardKinematics,
    generate_random_configurations,
    compute_pose_error
)


class IKDataset(Dataset):
    """
    Dataset for inverse kinematics training with 2-link arm and end-effector rotation.

    Each sample contains:
        - target_pose: Target end-effector pose [x, y, φ]
        - joint_angles: Ground truth joint angles [θ1, θ2, θ3]
    """

    def __init__(self, num_samples=10000, link_lengths=(1.0, 1.0),
                 angle_range=(-np.pi, np.pi)):
        """
        Args:
            num_samples: Number of samples to generate
            link_lengths: Tuple of link lengths [L1, L2]
            angle_range: Tuple of (min_angle, max_angle) for each joint
        """
        self.link_lengths = link_lengths
        print(f"Generating {num_samples} IK training samples...")

        # Generate random configurations
        self.joint_angles, self.target_poses = generate_random_configurations(
            num_samples, link_lengths, angle_range
        )

    def __len__(self):
        return len(self.joint_angles)

    def __getitem__(self, idx):
        return {
            'target_pose': self.target_poses[idx],
            'joint_angles': self.joint_angles[idx]
        }


class IKTrainer:
    """Training manager for the inverse kinematics network."""

    def __init__(
        self,
        model,
        train_dataset,
        val_dataset=None,
        batch_size=64,
        learning_rate=1e-3,
        device='cuda' if torch.cuda.is_available() else 'cpu',
        checkpoint_dir='checkpoints_ik',
        log_dir='logs_ik',
        link_lengths=(1.0, 1.0),
        loss_type='pose'  # 'pose', 'angle', or 'combined'
    ):
        self.model = model.to(device)
        self.device = device
        self.checkpoint_dir = checkpoint_dir
        self.log_dir = log_dir
        self.link_lengths = link_lengths
        self.loss_type = loss_type

        # Create directories
        os.makedirs(checkpoint_dir, exist_ok=True)
        os.makedirs(log_dir, exist_ok=True)

        # Data loaders
        self.train_loader = DataLoader(
            train_dataset,
            batch_size=batch_size,
            shuffle=True,
            num_workers=4,
            pin_memory=True if device == 'cuda' else False
        )

        self.val_loader = None
        if val_dataset:
            self.val_loader = DataLoader(
                val_dataset,
                batch_size=batch_size,
                shuffle=False,
                num_workers=4,
                pin_memory=True if device == 'cuda' else False
            )

        # Optimizer and scheduler
        self.optimizer = optim.AdamW(
            model.parameters(),
            lr=learning_rate,
            weight_decay=1e-5
        )

        self.scheduler = optim.lr_scheduler.ReduceLROnPlateau(
            self.optimizer,
            mode='min',
            factor=0.5,
            patience=10
        )

        # Loss function
        self.angle_criterion = nn.MSELoss()

        # Tensorboard writer
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.writer = SummaryWriter(os.path.join(log_dir, f'run_{timestamp}'))

        # Training state
        self.current_epoch = 0
        self.best_val_loss = float('inf')
        self.train_losses = []
        self.val_losses = []

    def compute_loss(self, predicted_angles, target_poses, ground_truth_angles=None):
        """
        Compute loss based on the specified loss type.

        Args:
            predicted_angles: Predicted joint angles (B, 3)
            target_poses: Target end-effector poses (B, 3) - [x, y, φ]
            ground_truth_angles: Ground truth joint angles (B, 3), optional

        Returns:
            loss: Total loss
            metrics: Dictionary of individual loss components
        """
        metrics = {}

        if self.loss_type == 'pose':
            # Pose-based loss: minimize end-effector pose error
            predicted_poses = ForwardKinematics.compute_simple(
                predicted_angles, self.link_lengths
            )
            # Position loss - use Euclidean distance (L2 norm) for proper spatial error
            position_diff = predicted_poses[:, :2] - target_poses[:, :2]
            position_loss = torch.mean((position_diff ** 2).sum(dim=1))

            # Rotation loss (handle wraparound)
            rotation_diff = predicted_poses[:, 2] - target_poses[:, 2]
            rotation_error_wrapped = torch.atan2(torch.sin(rotation_diff),
                                                  torch.cos(rotation_diff))
            rotation_loss = torch.mean(rotation_error_wrapped ** 2)

            # Combined with balanced weights - rotation normalized to similar scale as position
            # Position is in units of link lengths, rotation in radians (max ~pi)
            # Scale rotation to match position magnitude
            loss = position_loss + 1.0 * rotation_loss

            metrics['position_loss'] = position_loss.item()
            metrics['rotation_loss'] = rotation_loss.item()

            pos_err, rot_err = compute_pose_error(predicted_angles, target_poses, self.link_lengths)
            metrics['position_error'] = torch.mean(pos_err).item()
            metrics['rotation_error'] = torch.mean(rot_err).item()

        elif self.loss_type == 'angle':
            # Angle-based loss: minimize joint angle error
            if ground_truth_angles is None:
                raise ValueError("Ground truth angles required for angle-based loss")
            angle_loss = self.angle_criterion(predicted_angles, ground_truth_angles)
            loss = angle_loss
            metrics['angle_loss'] = angle_loss.item()

            pos_err, rot_err = compute_pose_error(predicted_angles, target_poses, self.link_lengths)
            metrics['position_error'] = torch.mean(pos_err).item()
            metrics['rotation_error'] = torch.mean(rot_err).item()

        elif self.loss_type == 'combined':
            # Combined loss: both pose and angle errors
            if ground_truth_angles is None:
                raise ValueError("Ground truth angles required for combined loss")

            predicted_poses = ForwardKinematics.compute_simple(
                predicted_angles, self.link_lengths
            )
            # Position loss - use Euclidean distance (L2 norm) for proper spatial error
            position_diff = predicted_poses[:, :2] - target_poses[:, :2]
            position_loss = torch.mean((position_diff ** 2).sum(dim=1))

            rotation_diff = predicted_poses[:, 2] - target_poses[:, 2]
            rotation_error_wrapped = torch.atan2(torch.sin(rotation_diff),
                                                  torch.cos(rotation_diff))
            rotation_loss = torch.mean(rotation_error_wrapped ** 2)
            angle_loss = self.angle_criterion(predicted_angles, ground_truth_angles)

            # Weighted combination with balanced weights
            loss = position_loss + 1.0 * rotation_loss + 0.1 * angle_loss

            metrics['position_loss'] = position_loss.item()
            metrics['rotation_loss'] = rotation_loss.item()
            metrics['angle_loss'] = angle_loss.item()

            pos_err, rot_err = compute_pose_error(predicted_angles, target_poses, self.link_lengths)
            metrics['position_error'] = torch.mean(pos_err).item()
            metrics['rotation_error'] = torch.mean(rot_err).item()

        else:
            raise ValueError(f"Unknown loss type: {self.loss_type}")

        return loss, metrics

    def train_epoch(self):
        """Train for one epoch."""
        self.model.train()
        epoch_loss = 0.0
        epoch_position_error = 0.0
        epoch_rotation_error = 0.0

        progress_bar = tqdm(self.train_loader, desc=f'Epoch {self.current_epoch+1}')

        for batch_idx, batch in enumerate(progress_bar):
            # Move data to device
            target_poses = batch['target_pose'].to(self.device)
            ground_truth_angles = batch['joint_angles'].to(self.device)

            # Forward pass
            self.optimizer.zero_grad()
            predicted_angles = self.model(target_poses)

            # Compute loss
            loss, metrics = self.compute_loss(
                predicted_angles,
                target_poses,
                ground_truth_angles
            )

            # Backward pass
            loss.backward()

            # Gradient clipping
            torch.nn.utils.clip_grad_norm_(self.model.parameters(), max_norm=1.0)

            self.optimizer.step()

            # Track metrics
            epoch_loss += loss.item()
            epoch_position_error += metrics['position_error']
            epoch_rotation_error += metrics['rotation_error']

            # Update progress bar
            progress_bar.set_postfix({
                'loss': loss.item(),
                'pos_err': metrics['position_error'],
                'rot_err': metrics['rotation_error']
            })

            # Log to tensorboard
            global_step = self.current_epoch * len(self.train_loader) + batch_idx
            self.writer.add_scalar('train/batch_loss', loss.item(), global_step)
            for key, value in metrics.items():
                self.writer.add_scalar(f'train/{key}', value, global_step)

        avg_loss = epoch_loss / len(self.train_loader)
        avg_position_error = epoch_position_error / len(self.train_loader)
        avg_rotation_error = epoch_rotation_error / len(self.train_loader)
        self.train_losses.append(avg_loss)

        return avg_loss, avg_position_error, avg_rotation_error

    def validate(self):
        """Validate the model."""
        if self.val_loader is None:
            return None, None, None

        self.model.eval()
        val_loss = 0.0
        val_position_error = 0.0
        val_rotation_error = 0.0

        with torch.no_grad():
            for batch in tqdm(self.val_loader, desc='Validation'):
                # Move data to device
                target_poses = batch['target_pose'].to(self.device)
                ground_truth_angles = batch['joint_angles'].to(self.device)

                # Forward pass
                predicted_angles = self.model(target_poses)

                # Compute loss
                loss, metrics = self.compute_loss(
                    predicted_angles,
                    target_poses,
                    ground_truth_angles
                )

                val_loss += loss.item()
                val_position_error += metrics['position_error']
                val_rotation_error += metrics['rotation_error']

        avg_val_loss = val_loss / len(self.val_loader)
        avg_val_position_error = val_position_error / len(self.val_loader)
        avg_val_rotation_error = val_rotation_error / len(self.val_loader)
        self.val_losses.append(avg_val_loss)

        return avg_val_loss, avg_val_position_error, avg_val_rotation_error

    def save_checkpoint(self, filename='checkpoint.pth', is_best=False):
        """Save training checkpoint."""
        checkpoint = {
            'epoch': self.current_epoch,
            'model_state_dict': self.model.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
            'scheduler_state_dict': self.scheduler.state_dict(),
            'train_losses': self.train_losses,
            'val_losses': self.val_losses,
            'best_val_loss': self.best_val_loss,
            'link_lengths': self.link_lengths
        }

        filepath = os.path.join(self.checkpoint_dir, filename)
        torch.save(checkpoint, filepath)

        if is_best:
            best_filepath = os.path.join(self.checkpoint_dir, 'best_model.pth')
            torch.save(checkpoint, best_filepath)
            print(f"Saved best model with validation loss: {self.best_val_loss:.6f}")

    def load_checkpoint(self, filename='checkpoint.pth'):
        """Load training checkpoint."""
        filepath = os.path.join(self.checkpoint_dir, filename)

        if not os.path.exists(filepath):
            print(f"Checkpoint {filepath} not found.")
            return False

        checkpoint = torch.load(filepath, map_location=self.device)

        self.model.load_state_dict(checkpoint['model_state_dict'])
        self.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
        self.scheduler.load_state_dict(checkpoint['scheduler_state_dict'])
        self.current_epoch = checkpoint['epoch']
        self.train_losses = checkpoint['train_losses']
        self.val_losses = checkpoint['val_losses']
        self.best_val_loss = checkpoint['best_val_loss']

        print(f"Loaded checkpoint from epoch {self.current_epoch}")
        return True

    def train(self, num_epochs, save_freq=10):
        """
        Train the model for multiple epochs.

        Args:
            num_epochs: Number of epochs to train
            save_freq: Save checkpoint every N epochs
        """
        print(f"Training on device: {self.device}")
        print(f"Training samples: {len(self.train_loader.dataset)}")
        if self.val_loader:
            print(f"Validation samples: {len(self.val_loader.dataset)}")
        print(f"Batch size: {self.train_loader.batch_size}")
        print(f"Loss type: {self.loss_type}")
        print("-" * 50)

        for _ in range(num_epochs):
            # Train
            train_loss, train_pos_error, train_rot_error = self.train_epoch()

            # Validate
            val_loss, val_pos_error, val_rot_error = self.validate()

            # Log to tensorboard
            self.writer.add_scalar('epoch/train_loss', train_loss, self.current_epoch)
            self.writer.add_scalar('epoch/train_position_error', train_pos_error, self.current_epoch)
            self.writer.add_scalar('epoch/train_rotation_error', train_rot_error, self.current_epoch)

            if val_loss is not None:
                self.writer.add_scalar('epoch/val_loss', val_loss, self.current_epoch)
                self.writer.add_scalar('epoch/val_position_error', val_pos_error, self.current_epoch)
                self.writer.add_scalar('epoch/val_rotation_error', val_rot_error, self.current_epoch)
                self.scheduler.step(val_loss)

            # Print epoch summary
            log_msg = f"Epoch {self.current_epoch+1}/{num_epochs} - "
            log_msg += f"Train Loss: {train_loss:.6f}, Pos Err: {train_pos_error:.6f}, Rot Err: {train_rot_error:.6f}"
            if val_loss is not None:
                log_msg += f" | Val Loss: {val_loss:.6f}, Pos Err: {val_pos_error:.6f}, Rot Err: {val_rot_error:.6f}"
            print(log_msg)

            # Save checkpoint
            if (self.current_epoch + 1) % save_freq == 0:
                self.save_checkpoint(f'checkpoint_epoch_{self.current_epoch+1}.pth')

            # Save best model
            if val_loss is not None and val_loss < self.best_val_loss:
                self.best_val_loss = val_loss
                self.save_checkpoint(is_best=True)

            self.current_epoch += 1

        print("-" * 50)
        print("Training completed!")
        print(f"Best validation loss: {self.best_val_loss:.6f}")

        self.writer.close()


def visualize_predictions(model, link_lengths, num_samples=5, device='cpu'):
    """Visualize IK predictions."""
    model.eval()

    # Generate test samples
    test_angles, test_poses = generate_random_configurations(
        num_samples, link_lengths
    )

    with torch.no_grad():
        test_poses = test_poses.to(device)
        predicted_angles = model(test_poses)
        predicted_angles = predicted_angles.cpu()

    # Create visualization
    fig, axes = plt.subplots(1, num_samples, figsize=(4*num_samples, 4))
    if num_samples == 1:
        axes = [axes]

    for i in range(num_samples):
        ax = axes[i]

        # Ground truth configuration
        _, gt_joints = ForwardKinematics.compute(test_angles[i:i+1], link_lengths)
        gt_x = [0] + [j[0, 0].item() for j in gt_joints[1:]]
        gt_y = [0] + [j[0, 1].item() for j in gt_joints[1:]]

        # Predicted configuration
        _, pred_joints = ForwardKinematics.compute(predicted_angles[i:i+1], link_lengths)
        pred_x = [0] + [j[0, 0].item() for j in pred_joints[1:]]
        pred_y = [0] + [j[0, 1].item() for j in pred_joints[1:]]

        # Plot
        ax.plot(gt_x, gt_y, 'b-o', linewidth=2, markersize=8, label='Ground Truth')
        ax.plot(pred_x, pred_y, 'r--^', linewidth=2, markersize=8, label='Predicted')
        ax.plot(test_poses[i, 0].cpu(), test_poses[i, 1].cpu(),
                'g*', markersize=15, label='Target')

        # Draw end-effector orientation
        gt_phi = test_poses[i, 2].cpu().item()
        pred_phi = ForwardKinematics.compute_simple(predicted_angles[i:i+1], link_lengths)[0, 2].item()
        arrow_len = 0.3

        ax.arrow(test_poses[i, 0].cpu(), test_poses[i, 1].cpu(),
                arrow_len * np.cos(gt_phi), arrow_len * np.sin(gt_phi),
                head_width=0.1, head_length=0.1, fc='green', ec='green', alpha=0.5)
        ax.arrow(pred_x[-1], pred_y[-1],
                arrow_len * np.cos(pred_phi), arrow_len * np.sin(pred_phi),
                head_width=0.1, head_length=0.1, fc='red', ec='red', alpha=0.5)

        # Calculate errors
        pos_err, rot_err = compute_pose_error(
            predicted_angles[i:i+1],
            test_poses[i:i+1].cpu(),
            link_lengths
        )

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title(f'Sample {i+1}\nPos Err: {pos_err.item():.4f}, Rot Err: {rot_err.item():.4f}')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.axis('equal')

    plt.tight_layout()
    return fig


def main():
    """Main training script."""

    # Configuration
    CONFIG = {
        'link_lengths': (1.0, 1.0),
        'batch_size': 64,
        'learning_rate': 1e-3,
        'num_epochs': 100,
        'train_samples': 50000,
        'val_samples': 5000,
        'checkpoint_dir': 'checkpoints_ik',
        'log_dir': 'logs_ik',
        'hidden_dims': [128, 256, 256, 128],
        'loss_type': 'pose'  # 'pose', 'angle', or 'combined'
    }

    print("=" * 60)
    print("Inverse Kinematics Network Training")
    print("2-Link Planar Arm with End-Effector Rotation")
    print("=" * 60)
    print(f"\nLink lengths: {CONFIG['link_lengths']}")
    print(f"Loss type: {CONFIG['loss_type']}")

    # Create datasets
    print("\nCreating datasets...")
    train_dataset = IKDataset(
        num_samples=CONFIG['train_samples'],
        link_lengths=CONFIG['link_lengths']
    )

    val_dataset = IKDataset(
        num_samples=CONFIG['val_samples'],
        link_lengths=CONFIG['link_lengths']
    )

    # Initialize model
    print("\nInitializing model...")
    model = InverseKinematicsNetwork(
        link_lengths=CONFIG['link_lengths'],
        input_link_lengths=False,
        hidden_dims=CONFIG['hidden_dims']
    )

    # Count parameters
    total_params = sum(p.numel() for p in model.parameters())
    print(f"Total parameters: {total_params:,}")

    # Initialize trainer
    trainer = IKTrainer(
        model=model,
        train_dataset=train_dataset,
        val_dataset=val_dataset,
        batch_size=CONFIG['batch_size'],
        learning_rate=CONFIG['learning_rate'],
        checkpoint_dir=CONFIG['checkpoint_dir'],
        log_dir=CONFIG['log_dir'],
        link_lengths=CONFIG['link_lengths'],
        loss_type=CONFIG['loss_type']
    )

    # Train
    print("\nStarting training...\n")
    trainer.train(num_epochs=CONFIG['num_epochs'], save_freq=10)

    # Visualize results
    print("\nGenerating visualization...")
    fig = visualize_predictions(
        model,
        CONFIG['link_lengths'],
        num_samples=5,
        device=trainer.device
    )
    viz_path = os.path.join(CONFIG['checkpoint_dir'], 'predictions.png')
    fig.savefig(viz_path, dpi=150, bbox_inches='tight')
    print(f"Saved visualization to: {viz_path}")

    print("\nTraining complete!")
    print(f"Checkpoints saved to: {CONFIG['checkpoint_dir']}")
    print(f"Logs saved to: {CONFIG['log_dir']}")
    print("\nTo view training progress, run:")
    print(f"  tensorboard --logdir {CONFIG['log_dir']}")


if __name__ == "__main__":
    main()
