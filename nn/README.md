# Robot Control Neural Network

Multi-modal neural network for robotic arm control with vision and proprioception inputs.

## Overview

This package contains:
- `robot_control_network.py`: Neural network architecture
- `train_robot_network.py`: Training pipeline with HDF5 and numpy support
- `create_hdf5_dataset.py`: Utility to create HDF5 dataset files

## Network Architecture

**Inputs:**
- 2 camera images (224x224 RGB)
- 3 joint positions (current state)
- 1 suction cup status (on/off)

**Outputs:**
- 3 joint positions (predicted next state)
- 1 suction command (probability 0-1)

## Data Format

### HDF5 Format (Recommended for large datasets)

HDF5 files should contain the following datasets:

```
/camera1          (N, 3, H, W) - Camera 1 images, float32
/camera2          (N, 3, H, W) - Camera 2 images, float32
/current_joints   (N, 3)       - Current joint positions, float32
/current_suction  (N, 1)       - Current suction status (0 or 1), float32
/next_joints      (N, 3)       - Target next joint positions, float32
/next_suction     (N, 1)       - Target next suction status (0 or 1), float32
```

### NumPy Format

NumPy `.npz` files should contain arrays with the same keys as above.

## Usage

### 1. Create a Dataset

#### Create HDF5 dataset (recommended):

```bash
# Create a small dataset for testing
python create_hdf5_dataset.py --output train_data.h5 --samples 1000

# Create a large dataset with compression
python create_hdf5_dataset.py --output train_data.h5 --samples 100000 --compression gzip

# Inspect an existing dataset
python create_hdf5_dataset.py --inspect train_data.h5
```

#### Or prepare your own data:

```python
import h5py
import numpy as np

# Your real robot data
camera1_data = ...  # Shape: (N, 3, 224, 224)
camera2_data = ...
current_joints = ...  # Shape: (N, 3)
current_suction = ...  # Shape: (N, 1)
next_joints = ...
next_suction = ...

# Save to HDF5
with h5py.File('my_robot_data.h5', 'w') as f:
    f.create_dataset('camera1', data=camera1_data, compression='gzip')
    f.create_dataset('camera2', data=camera2_data, compression='gzip')
    f.create_dataset('current_joints', data=current_joints, compression='gzip')
    f.create_dataset('current_suction', data=current_suction, compression='gzip')
    f.create_dataset('next_joints', data=next_joints, compression='gzip')
    f.create_dataset('next_suction', data=next_suction, compression='gzip')
```

### 2. Train the Network

#### Using synthetic data (for testing):

```bash
python train_robot_network.py
```

#### Using HDF5 data (memory efficient, lazy loading):

```python
from train_robot_network import RobotDataset, RobotTrainer
from robot_control_network import RobotControlNetwork

# Create datasets with lazy loading (memory efficient)
train_dataset = RobotDataset(
    data_path='train_data.h5',
    use_hdf5=True  # Enable lazy loading
)

val_dataset = RobotDataset(
    data_path='val_data.h5',
    use_hdf5=True
)

# Initialize and train
model = RobotControlNetwork()
trainer = RobotTrainer(
    model=model,
    train_dataset=train_dataset,
    val_dataset=val_dataset,
    batch_size=16
)

trainer.train(num_epochs=50)
```

#### Using HDF5 data (load into memory):

```python
# Load entire dataset into memory (faster but requires more RAM)
train_dataset = RobotDataset(
    data_path='train_data.h5',
    use_hdf5=False  # Load all data into memory
)
```

### 3. Monitor Training

View training progress with TensorBoard:

```bash
tensorboard --logdir logs
```

### 4. Use Trained Model

```python
import torch
from robot_control_network import RobotControlNetwork

# Load trained model
model = RobotControlNetwork()
checkpoint = torch.load('checkpoints/best_model.pth')
model.load_state_dict(checkpoint['model_state_dict'])
model.eval()

# Inference
with torch.no_grad():
    next_joints, next_suction = model(
        camera1_img,
        camera2_img,
        current_joints,
        current_suction
    )

    # Convert suction probability to binary command
    suction_command = (next_suction > 0.5).float()
```

## Training Configuration

Key parameters in `train_robot_network.py`:

```python
CONFIG = {
    'img_height': 224,           # Image dimensions
    'img_width': 224,
    'joint_dim': 3,              # Number of joints
    'batch_size': 16,            # Batch size
    'learning_rate': 1e-4,       # Learning rate
    'num_epochs': 50,            # Training epochs
    'checkpoint_dir': 'checkpoints',
    'log_dir': 'logs'
}
```

## Memory Considerations

- **Lazy loading (`use_hdf5=True`)**: Memory efficient, loads data on-demand. Slower due to disk I/O.
- **Memory loading (`use_hdf5=False`)**: Fast training, but requires enough RAM to hold entire dataset.

For large datasets (>10GB), use lazy loading. For small datasets, loading into memory is faster.

## Loss Functions

- **Joint Loss**: MSE (Mean Squared Error) for continuous joint positions
- **Suction Loss**: BCE (Binary Cross Entropy) for binary suction command
- **Combined**: `total_loss = joint_loss + 0.5 * suction_loss`

## Model Architecture

```
Inputs → [Vision Encoder] → 256 features (cam1)
      → [Vision Encoder] → 256 features (cam2)
      → [Joint Encoder] → 128 features
      → [Suction Encoder] → 32 features

Combined (672 features) → [Fusion Network] → 128 features
                       → [Joint Head] → 3 joint positions
                       → [Suction Head] → 1 suction probability
```

## Requirements

```
torch
numpy
h5py
tqdm
tensorboard
```

Install with:
```bash
pip install torch numpy h5py tqdm tensorboard
```
