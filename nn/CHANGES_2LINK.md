# Changes to 2-Link Arm Configuration

## Overview

The inverse kinematics system has been updated to work with a **2-link planar arm** where the **third joint controls only end-effector rotation**, not position.

## Key Changes

### Kinematics Model

**Previous (3-link arm):**
- 3 links contribute to position: L1, L2, L3
- End-effector position: (x, y)
- Input/Output: position only

**New (2-link arm with rotation):**
- 2 links contribute to position: L1, L2
- Third joint (θ3) controls end-effector rotation (φ)
- End-effector pose: (x, y, φ)
- Input/Output: position + rotation

### Mathematical Formulation

#### Forward Kinematics:
```
x = L1*cos(θ1) + L2*cos(θ1 + θ2)
y = L1*sin(θ1) + L2*sin(θ1 + θ2)
φ = θ1 + θ2 + θ3
```

Where:
- θ1, θ2: Joint angles that control position
- θ3: Joint angle that controls end-effector rotation
- φ: End-effector orientation

#### Inverse Kinematics:
```
[θ1, θ2, θ3] = f_NN(x, y, φ)
```

The neural network learns the mapping from desired pose (x, y, φ) to joint angles.

## Modified Files

### 1. `inverse_kinematics_network.py`

#### Changes:
- **`InverseKinematicsNetwork`**:
  - Input dimension changed from 2 to 3 (added φ)
  - Link lengths parameter changed from (L1, L2, L3) to (L1, L2)
  - Default: `link_lengths=(1.0, 1.0)`

- **`ForwardKinematics.compute()`**:
  - Only uses 2 link lengths
  - Returns pose (x, y, φ) instead of just position (x, y)
  - Joint positions list only includes base, joint1, and wrist (3 positions)

- **`compute_pose_error()` (renamed from `compute_position_error`)**:
  - Returns two error values: position error and rotation error
  - Handles angular wraparound for rotation error

- **`generate_random_configurations()`**:
  - Returns poses (x, y, φ) instead of just positions (x, y)
  - Uses 2 link lengths

### 2. `train_inverse_kinematics.py`

#### Changes:
- **`IKDataset`**:
  - Stores `target_poses` (3D) instead of `target_positions` (2D)
  - Uses 2 link lengths

- **`IKTrainer.compute_loss()`**:
  - Handles 3D pose input
  - Computes both position and rotation losses
  - Loss types:
    - `'pose'`: MSE on position + weighted MSE on rotation
    - `'angle'`: MSE on joint angles
    - `'combined'`: Weighted combination of both

- **`visualize_predictions()`**:
  - Draws orientation arrows to show end-effector rotation
  - Reports both position and rotation errors

### 3. `test_inverse_kinematics.py` *(needs updating)*

Will need updates to:
- Use `compute_pose_error` instead of `compute_position_error`
- Handle 3D poses instead of 2D positions
- Visualize end-effector orientation
- Report both position and rotation errors

### 4. `interactive_ik_demo.py` *(needs updating)*

Will need updates to:
- Allow user to specify target rotation
- Display end-effector orientation
- Show rotation error alongside position error

### 5. `README_IK.md` *(needs updating)*

Will need updates to:
- Document the 2-link + rotation configuration
- Update examples with 3D poses
- Explain rotation error metric

## Network Architecture

**Input**: 3 values (x, y, φ)
**Hidden Layers**: [128, 256, 256, 128] (unchanged)
**Output**: 3 values (θ1, θ2, θ3)

**Parameters**: ~132,611 (slightly reduced from 3-link version)

## Workspace

**Position workspace** (2D circle):
- Maximum reach: L1 + L2
- Minimum reach: |L1 - L2|

**Rotation workspace**:
- Full 360° rotation: φ ∈ (-∞, +∞)
- Network handles wraparound in loss function

## Loss Functions

### 1. Pose Loss (default)
```python
position_loss = MSE(predicted_xy, target_xy)
rotation_loss = MSE(angle_wrap(predicted_φ - target_φ))
total_loss = position_loss + 0.5 * rotation_loss
```

### 2. Angle Loss
```python
total_loss = MSE(predicted_angles, ground_truth_angles)
```

### 3. Combined Loss
```python
total_loss = position_loss + 0.5 * rotation_loss + 0.1 * angle_loss
```

## Error Metrics

**Position Error**: Euclidean distance in (x, y)
```python
pos_error = ||predicted_xy - target_xy||
```

**Rotation Error**: Absolute angular difference with wraparound
```python
rot_diff = predicted_φ - target_φ
rot_error = |atan2(sin(rot_diff), cos(rot_diff))|
```

## Usage Examples

### Generate Training Data
```python
from inverse_kinematics_network import generate_random_configurations

# Generate 1000 samples
joint_angles, end_effector_poses = generate_random_configurations(
    num_samples=1000,
    link_lengths=(1.0, 1.0)
)

# joint_angles: (1000, 3) - [θ1, θ2, θ3]
# end_effector_poses: (1000, 3) - [x, y, φ]
```

### Forward Kinematics
```python
from inverse_kinematics_network import ForwardKinematics

joint_angles = torch.tensor([[0.5, 0.3, -0.2]])  # [θ1, θ2, θ3]
link_lengths = (1.0, 1.0)

pose, joint_positions = ForwardKinematics.compute(joint_angles, link_lengths)
# pose: [x, y, φ]
# joint_positions: [base, joint1, wrist]
```

### Inverse Kinematics (trained model)
```python
model = InverseKinematicsNetwork(link_lengths=(1.0, 1.0))
model.load_state_dict(checkpoint['model_state_dict'])
model.eval()

target_pose = torch.tensor([[1.5, 0.5, 0.785]])  # [x, y, φ] (φ ≈ 45°)
joint_angles = model(target_pose)
# joint_angles: [θ1, θ2, θ3]
```

### Compute Errors
```python
from inverse_kinematics_network import compute_pose_error

pos_error, rot_error = compute_pose_error(
    predicted_angles,
    target_poses,
    link_lengths=(1.0, 1.0)
)
```

## Training

```bash
python train_inverse_kinematics.py
```

**Configuration**:
- Link lengths: (1.0, 1.0)
- Training samples: 50,000
- Validation samples: 5,000
- Loss type: 'pose'
- Epochs: 100

**Expected Performance** (with pose loss):
- Position error: < 0.01
- Rotation error: < 0.1 rad (< 6°)

## Benefits of This Approach

1. **Decoupled Control**: Position (θ1, θ2) and rotation (θ3) are somewhat decoupled
2. **Full Orientation Control**: Can achieve any orientation at reachable positions
3. **Realistic Model**: Matches many real robotic arms with wrist rotation
4. **Efficient**: Only 2 links for positioning reduces complexity

## Comparison: 3-Link vs 2-Link+Rotation

| Feature | 3-Link Arm | 2-Link + Rotation |
|---------|------------|-------------------|
| Position DOF | 3 | 2 |
| Orientation DOF | Implicit | 1 (explicit) |
| Workspace shape | 2D (circular) | 2D (circular) |
| Max reach | L1+L2+L3 | L1+L2 |
| Redundancy | Yes (infinite solutions) | No (finite solutions) |
| Network input | (x, y) | (x, y, φ) |
| Network output | (θ1, θ2, θ3) | (θ1, θ2, θ3) |
| Typical accuracy | < 0.001 | < 0.01 pos, < 0.1 rad |

## Next Steps

To complete the transition, still need to update:
1. `test_inverse_kinematics.py` - Testing script
2. `interactive_ik_demo.py` - Interactive demo
3. `README_IK.md` - Documentation

All these files need to be updated to handle 3D poses and display rotation information.

## Testing

Run the test to verify changes:
```bash
python3 inverse_kinematics_network.py
```

Should output:
- Network parameters: ~132,611
- Test configurations with poses [x, y, φ]
- Both position and rotation errors
- Workspace analysis including φ range
