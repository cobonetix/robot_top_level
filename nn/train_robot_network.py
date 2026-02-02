import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
from torch.utils.tensorboard import SummaryWriter
import numpy as np
import h5py
import os
from datetime import datetime
from tqdm import tqdm

from robot_control_network import RobotControlNetwork


class RobotDataset(Dataset):
    """
    Dataset for robot control training.

    Each sample contains:
        - camera1_img: Image from camera 1
        - camera2_img: Image from camera 2
        - current_joints: Current joint positions
        - current_suction_onoff: Current suction on/off status (0=off, 1=on)
        - current_suction_contact: Current suction contact status (0=no contact, 1=contact)
        - next_joints: Target next joint positions
        - next_suction_onoff: Target next suction on/off command (0=off, 1=on)
    """

    def __init__(self, data_path=None, num_samples=1000, img_height=224, img_width=224, use_hdf5=False):
        """
        Args:
            data_path: Path to saved dataset (numpy .npz or HDF5 .h5/.hdf5 format)
            num_samples: Number of synthetic samples to generate if data_path is None
            img_height: Image height
            img_width: Image width
            use_hdf5: If True, use HDF5 lazy loading (memory efficient for large datasets)
        """
        self.img_height = img_height
        self.img_width = img_width
        self.use_hdf5 = use_hdf5
        self.h5_file = None
        self.data_path = data_path

        if data_path and os.path.exists(data_path):
            # Determine file format
            file_ext = os.path.splitext(data_path)[1].lower()

            if file_ext in ['.h5', '.hdf5']:
                # Load HDF5 data
                print(f"Loading HDF5 dataset from {data_path}...")
                if use_hdf5:
                    # Lazy loading mode - keep file open and load on demand
                    self.h5_file = h5py.File(data_path, 'r')
                    self.num_samples = len(self.h5_file['camera1'])
                    print(f"Loaded {self.num_samples} samples (lazy loading mode)")
                else:
                    # Load entire dataset into memory
                    with h5py.File(data_path, 'r') as f:
                        self.camera1_imgs = torch.from_numpy(f['camera1'][:]).float()
                        self.camera2_imgs = torch.from_numpy(f['camera2'][:]).float()
                        self.current_joints = torch.from_numpy(f['current_joints'][:]).float()
                        self.current_suction_onoff = torch.from_numpy(f['current_suction_onoff'][:]).float()
                        self.current_suction_contact = torch.from_numpy(f['current_suction_contact'][:]).float()
                        self.next_joints = torch.from_numpy(f['next_joints'][:]).float()
                        self.next_suction_onoff = torch.from_numpy(f['next_suction_onoff'][:]).float()
                    print(f"Loaded {len(self.camera1_imgs)} samples into memory")

            elif file_ext == '.npz':
                # Load numpy data
                print(f"Loading numpy dataset from {data_path}...")
                data = np.load(data_path)
                self.camera1_imgs = torch.from_numpy(data['camera1']).float()
                self.camera2_imgs = torch.from_numpy(data['camera2']).float()
                self.current_joints = torch.from_numpy(data['current_joints']).float()
                self.current_suction_onoff = torch.from_numpy(data['current_suction_onoff']).float()
                self.current_suction_contact = torch.from_numpy(data['current_suction_contact']).float()
                self.next_joints = torch.from_numpy(data['next_joints']).float()
                self.next_suction_onoff = torch.from_numpy(data['next_suction_onoff']).float()
                print(f"Loaded {len(self.camera1_imgs)} samples from numpy")

            else:
                raise ValueError(f"Unsupported file format: {file_ext}. Use .h5, .hdf5, or .npz")
        else:
            # Generate synthetic data for demonstration
            print(f"Generating {num_samples} synthetic samples...")
            self._generate_synthetic_data(num_samples)

    def _generate_synthetic_data(self, num_samples):
        """Generate synthetic training data."""
        # Random images (in practice, these would be real camera images)
        self.camera1_imgs = torch.randn(num_samples, 3, self.img_height, self.img_width)
        self.camera2_imgs = torch.randn(num_samples, 3, self.img_height, self.img_width)

        # Random joint positions (normalized to [-1, 1])
        self.current_joints = torch.rand(num_samples, 3) * 2 - 1

        # Simulate next joints as current + small random delta
        # This creates a simple pattern for the network to learn
        delta = torch.randn(num_samples, 3) * 0.1
        self.next_joints = torch.clamp(self.current_joints + delta, -1, 1)

        # Generate suction cup data
        # Current suction on/off status (binary: 0 or 1)
        self.current_suction_onoff = torch.randint(0, 2, (num_samples, 1)).float()

        # Current suction contact status (binary: 0 or 1)
        # Contact can only be 1 if suction is on
        self.current_suction_contact = torch.zeros(num_samples, 1).float()
        contact_possible = self.current_suction_onoff == 1.0
        self.current_suction_contact[contact_possible] = torch.randint(0, 2, (contact_possible.sum().item(), 1)).float()

        # Simulate next suction on/off: mostly stays the same, sometimes toggles
        # 80% chance to stay the same, 20% chance to toggle
        toggle_mask = torch.rand(num_samples, 1) < 0.2
        self.next_suction_onoff = self.current_suction_onoff.clone()
        self.next_suction_onoff[toggle_mask] = 1.0 - self.next_suction_onoff[toggle_mask]

    def __len__(self):
        if self.use_hdf5 and self.h5_file is not None:
            return self.num_samples
        return len(self.current_joints)

    def __getitem__(self, idx):
        if self.use_hdf5 and self.h5_file is not None:
            # Lazy loading from HDF5
            return {
                'camera1': torch.from_numpy(self.h5_file['camera1'][idx]).float(),
                'camera2': torch.from_numpy(self.h5_file['camera2'][idx]).float(),
                'current_joints': torch.from_numpy(self.h5_file['current_joints'][idx]).float(),
                'current_suction_onoff': torch.from_numpy(self.h5_file['current_suction_onoff'][idx]).float(),
                'current_suction_contact': torch.from_numpy(self.h5_file['current_suction_contact'][idx]).float(),
                'next_joints': torch.from_numpy(self.h5_file['next_joints'][idx]).float(),
                'next_suction_onoff': torch.from_numpy(self.h5_file['next_suction_onoff'][idx]).float()
            }
        else:
            # Load from memory
            return {
                'camera1': self.camera1_imgs[idx],
                'camera2': self.camera2_imgs[idx],
                'current_joints': self.current_joints[idx],
                'current_suction_onoff': self.current_suction_onoff[idx],
                'current_suction_contact': self.current_suction_contact[idx],
                'next_joints': self.next_joints[idx],
                'next_suction_onoff': self.next_suction_onoff[idx]
            }

    def __del__(self):
        """Close HDF5 file when dataset is destroyed."""
        if self.h5_file is not None:
            self.h5_file.close()


class RobotTrainer:
    """Training manager for the robot control network."""

    def __init__(
        self,
        model,
        train_dataset,
        val_dataset=None,
        batch_size=16,
        learning_rate=1e-4,
        device='cuda' if torch.cuda.is_available() else 'cpu',
        checkpoint_dir='checkpoints',
        log_dir='logs'
    ):
        self.model = model.to(device)
        self.device = device
        self.checkpoint_dir = checkpoint_dir
        self.log_dir = log_dir

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
            patience=5
        )

        # Loss functions
        self.joint_criterion = nn.MSELoss()  # For joint positions
        self.suction_onoff_criterion = nn.BCELoss()  # For suction on/off binary classification

        # Tensorboard writer
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.writer = SummaryWriter(os.path.join(log_dir, f'run_{timestamp}'))

        # Training state
        self.current_epoch = 0
        self.best_val_loss = float('inf')
        self.train_losses = []
        self.val_losses = []

    def train_epoch(self):
        """Train for one epoch."""
        self.model.train()
        epoch_loss = 0.0

        progress_bar = tqdm(self.train_loader, desc=f'Epoch {self.current_epoch+1}')

        for batch_idx, batch in enumerate(progress_bar):
            # Move data to device
            camera1 = batch['camera1'].to(self.device)
            camera2 = batch['camera2'].to(self.device)
            current_joints = batch['current_joints'].to(self.device)
            current_suction_onoff = batch['current_suction_onoff'].to(self.device)
            current_suction_contact = batch['current_suction_contact'].to(self.device)
            target_joints = batch['next_joints'].to(self.device)
            target_suction_onoff = batch['next_suction_onoff'].to(self.device)

            # Forward pass
            self.optimizer.zero_grad()
            predicted_joints, predicted_suction_onoff = self.model(
                camera1, camera2, current_joints, current_suction_onoff, current_suction_contact
            )

            # Compute losses
            joint_loss = self.joint_criterion(predicted_joints, target_joints)
            suction_onoff_loss = self.suction_onoff_criterion(predicted_suction_onoff, target_suction_onoff)

            # Combined loss (weighted)
            loss = joint_loss + 0.5 * suction_onoff_loss

            # Backward pass
            loss.backward()

            # Gradient clipping to prevent exploding gradients
            torch.nn.utils.clip_grad_norm_(self.model.parameters(), max_norm=1.0)

            self.optimizer.step()

            # Track loss
            epoch_loss += loss.item()

            # Update progress bar
            progress_bar.set_postfix({
                'loss': loss.item(),
                'joint_loss': joint_loss.item(),
                'suction_onoff_loss': suction_onoff_loss.item()
            })

            # Log to tensorboard
            global_step = self.current_epoch * len(self.train_loader) + batch_idx
            self.writer.add_scalar('train/batch_loss', loss.item(), global_step)
            self.writer.add_scalar('train/joint_loss', joint_loss.item(), global_step)
            self.writer.add_scalar('train/suction_onoff_loss', suction_onoff_loss.item(), global_step)

        avg_loss = epoch_loss / len(self.train_loader)
        self.train_losses.append(avg_loss)

        return avg_loss

    def validate(self):
        """Validate the model."""
        if self.val_loader is None:
            return None

        self.model.eval()
        val_loss = 0.0

        with torch.no_grad():
            for batch in tqdm(self.val_loader, desc='Validation'):
                # Move data to device
                camera1 = batch['camera1'].to(self.device)
                camera2 = batch['camera2'].to(self.device)
                current_joints = batch['current_joints'].to(self.device)
                current_suction_onoff = batch['current_suction_onoff'].to(self.device)
                current_suction_contact = batch['current_suction_contact'].to(self.device)
                target_joints = batch['next_joints'].to(self.device)
                target_suction_onoff = batch['next_suction_onoff'].to(self.device)

                # Forward pass
                predicted_joints, predicted_suction_onoff = self.model(
                    camera1, camera2, current_joints, current_suction_onoff, current_suction_contact
                )

                # Compute losses
                joint_loss = self.joint_criterion(predicted_joints, target_joints)
                suction_onoff_loss = self.suction_onoff_criterion(predicted_suction_onoff, target_suction_onoff)

                # Combined loss
                loss = joint_loss + 0.5 * suction_onoff_loss
                val_loss += loss.item()

        avg_val_loss = val_loss / len(self.val_loader)
        self.val_losses.append(avg_val_loss)

        return avg_val_loss

    def save_checkpoint(self, filename='checkpoint.pth', is_best=False):
        """Save training checkpoint."""
        checkpoint = {
            'epoch': self.current_epoch,
            'model_state_dict': self.model.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
            'scheduler_state_dict': self.scheduler.state_dict(),
            'train_losses': self.train_losses,
            'val_losses': self.val_losses,
            'best_val_loss': self.best_val_loss
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

    def train(self, num_epochs, save_freq=5):
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
        print("-" * 50)

        for _ in range(num_epochs):
            # Train
            train_loss = self.train_epoch()

            # Validate
            val_loss = self.validate()

            # Log to tensorboard
            self.writer.add_scalar('epoch/train_loss', train_loss, self.current_epoch)
            if val_loss is not None:
                self.writer.add_scalar('epoch/val_loss', val_loss, self.current_epoch)
                self.scheduler.step(val_loss)

            # Print epoch summary
            log_msg = f"Epoch {self.current_epoch+1}/{num_epochs} - Train Loss: {train_loss:.6f}"
            if val_loss is not None:
                log_msg += f" - Val Loss: {val_loss:.6f}"
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


def main():
    """Main training script."""

    # Configuration
    CONFIG = {
        'img_height': 224,
        'img_width': 224,
        'joint_dim': 3,
        'batch_size': 16,
        'learning_rate': 1e-4,
        'num_epochs': 50,
        'train_samples': 5000,
        'val_samples': 1000,
        'checkpoint_dir': 'checkpoints',
        'log_dir': 'logs'
    }

    print("=" * 50)
    print("Robot Control Network Training")
    print("=" * 50)

    # Create datasets
    print("\nCreating datasets...")
    train_dataset = RobotDataset(
        num_samples=CONFIG['train_samples'],
        img_height=CONFIG['img_height'],
        img_width=CONFIG['img_width']
    )

    val_dataset = RobotDataset(
        num_samples=CONFIG['val_samples'],
        img_height=CONFIG['img_height'],
        img_width=CONFIG['img_width']
    )

    # Initialize model
    print("\nInitializing model...")
    model = RobotControlNetwork(
        img_height=CONFIG['img_height'],
        img_width=CONFIG['img_width'],
        joint_dim=CONFIG['joint_dim']
    )

    # Count parameters
    total_params = sum(p.numel() for p in model.parameters())
    print(f"Total parameters: {total_params:,}")

    # Initialize trainer
    trainer = RobotTrainer(
        model=model,
        train_dataset=train_dataset,
        val_dataset=val_dataset,
        batch_size=CONFIG['batch_size'],
        learning_rate=CONFIG['learning_rate'],
        checkpoint_dir=CONFIG['checkpoint_dir'],
        log_dir=CONFIG['log_dir']
    )

    # Train
    print("\nStarting training...\n")
    trainer.train(num_epochs=CONFIG['num_epochs'], save_freq=5)

    print("\nTraining complete!")
    print(f"Checkpoints saved to: {CONFIG['checkpoint_dir']}")
    print(f"Logs saved to: {CONFIG['log_dir']}")
    print("\nTo view training progress, run:")
    print(f"  tensorboard --logdir {CONFIG['log_dir']}")


if __name__ == "__main__":
    main()
