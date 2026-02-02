# Neural Network Files Summary

## Important Note

**All neural networks are built with PyTorch, NOT TensorFlow!**

The only TensorFlow-related dependency was TensorBoard (used for logging visualization in some training scripts). The "no TensorBoard" versions eliminate this dependency entirely.

## File Organization

### 1. Inverse Kinematics (2-Link Arm with Rotation)

**Network Definition:**
- `inverse_kinematics_network.py` - Core IK network (PyTorch only)
  - Input: End-effector pose [x, y, φ]
  - Output: Joint angles [θ1, θ2, θ3]
  - 2 links control position, 3rd joint controls rotation

**Training Scripts:**
- `train_inverse_kinematics.py` - With TensorBoard logging
- `train_inverse_kinematics_no_tensorboard.py` - **Without TensorBoard** (JSON + matplotlib)

**Testing Scripts:**
- `test_inverse_kinematics.py` - Evaluation and testing
- `interactive_ik_demo.py` - Interactive visualization demo

**Documentation:**
- `README_IK.md` - IK network documentation
- `CHANGES_2LINK.md` - Details on 2-link configuration

### 2. Robot Control Network (Dual Camera)

**Network Definition:**
- `robot_control_network.py` - Multi-modal control network (PyTorch only)
  - Input: 2 camera images + joint states + suction states
  - Output: Next joint positions + suction command

**Training Scripts:**
- `train_robot_network.py` - With TensorBoard logging
- `train_robot_network_no_tensorboard.py` - **Without TensorBoard** (JSON + matplotlib)

**Dataset Creation:**
- `create_hdf5_dataset.py` - Generate HDF5 training data

### 3. Robot Control Network (Single Camera)

**Network Definition:**
- `robot_control_network_single_camera.py` - Single camera version (PyTorch only)
  - Input: 1 camera image + joint states + suction states
  - Output: Next joint positions + suction command

**Training Scripts:**
- `train_robot_network_single_camera.py` - With TensorBoard logging
- *(No TensorBoard version not yet created)*

**Dataset Creation:**
- `create_hdf5_dataset_single_camera.py` - Generate HDF5 data

### 4. Documentation

- `README.md` - General overview
- `README_IK.md` - Inverse kinematics details
- `README_NO_TENSORBOARD.md` - **No TensorBoard versions guide**
- `CHANGES_2LINK.md` - 2-link arm modifications
- `SUMMARY.md` - This file

## Quick Start

### Inverse Kinematics (2-Link + Rotation)

```bash
# Train with TensorBoard
python3 train_inverse_kinematics.py

# Train without TensorBoard (recommended if TF not installed)
python3 train_inverse_kinematics_no_tensorboard.py

# Test trained model
python3 test_inverse_kinematics.py --checkpoint checkpoints_ik/best_model.pth

# Interactive demo
python3 interactive_ik_demo.py --checkpoint checkpoints_ik/best_model.pth
```

### Robot Control Network

```bash
# Train with TensorBoard
python3 train_robot_network.py

# Train without TensorBoard (recommended if TF not installed)
python3 train_robot_network_no_tensorboard.py

# Create dataset first (optional, trains on synthetic data by default)
python3 create_hdf5_dataset.py --output robot_data.h5 --samples 10000
```

## Dependencies

### All Networks (Core):
```bash
pip install torch torchvision tqdm h5py numpy matplotlib
```

### With TensorBoard:
```bash
pip install tensorboard  # Requires TensorFlow backend (~500MB)
```

### Without TensorBoard:
No additional dependencies needed!

## Key Features

### Inverse Kinematics Network
- ✅ 2-link planar arm with end-effector rotation
- ✅ Learns mapping from pose (x, y, φ) to angles (θ1, θ2, θ3)
- ✅ ~132K parameters
- ✅ Interactive demo with click-to-target
- ✅ Multiple loss functions (pose, angle, combined)

### Robot Control Network
- ✅ Multi-modal: vision + proprioception
- ✅ Dual camera or single camera versions
- ✅ Predicts next robot state
- ✅ Suction cup control
- ✅ HDF5 dataset support for large datasets

## TensorBoard vs No TensorBoard

| Feature | With TensorBoard | Without TensorBoard |
|---------|------------------|---------------------|
| Network | PyTorch | PyTorch |
| Training | Same | Same |
| TensorFlow required | Yes (for TensorBoard) | No |
| Log format | Event files | JSON |
| Visualization | Web UI | PNG plots |
| Real-time monitoring | Yes | No |
| Dependencies | +500MB | Minimal |

**Recommendation**: Use "no TensorBoard" versions unless you specifically need real-time monitoring.

## File Count

- **Total Python files**: 14
- **Network definitions**: 3 (all PyTorch)
- **Training scripts**: 5 (2 with TensorBoard, 3 without)
- **Utility scripts**: 4 (testing, demo, dataset creation)
- **Documentation**: 5 files

## Which Files to Use

### For Inverse Kinematics:
1. **Network**: `inverse_kinematics_network.py`
2. **Training**: Choose one:
   - `train_inverse_kinematics.py` (with TensorBoard)
   - `train_inverse_kinematics_no_tensorboard.py` (without)
3. **Testing**: `test_inverse_kinematics.py`
4. **Demo**: `interactive_ik_demo.py`

### For Robot Control (Dual Camera):
1. **Network**: `robot_control_network.py`
2. **Training**: Choose one:
   - `train_robot_network.py` (with TensorBoard)
   - `train_robot_network_no_tensorboard.py` (without)
3. **Dataset**: `create_hdf5_dataset.py`

### For Robot Control (Single Camera):
1. **Network**: `robot_control_network_single_camera.py`
2. **Training**: `train_robot_network_single_camera.py` (with TensorBoard)
3. **Dataset**: `create_hdf5_dataset_single_camera.py`

## Framework Used

**PyTorch** is used for all neural networks:
- Network definitions: `torch.nn.Module`
- Training loops: `torch.optim`, `torch.nn`
- Data loading: `torch.utils.data.Dataset`, `DataLoader`
- Tensors: `torch.Tensor`

**No TensorFlow** is used in network code. TensorFlow was only needed as a backend for TensorBoard in some training scripts.

## Trained Model Compatibility

All trained models (.pth files) are compatible between TensorBoard and no-TensorBoard versions because:
- Same PyTorch networks
- Same training algorithms
- Same checkpoint format
- Only logging differs

## Next Steps

1. Choose your network type (IK or Robot Control)
2. Choose training version (with or without TensorBoard)
3. Run training script
4. Test with provided testing/demo scripts

See individual README files for detailed documentation!
