import torch
import torch.nn as nn
import torch.nn.functional as F


class RobotControlNetworkSingleCamera(nn.Module):
    """
    Multi-modal neural network for robotic arm control with single camera.

    Inputs:
        - camera_img: Image from camera (B, 3, H, W)
        - current_joints: Current joint positions (B, 3)
        - current_suction_onoff: Current suction on/off status (B, 1) - 0=off, 1=on
        - current_suction_contact: Current suction contact status (B, 1) - 0=no contact, 1=contact

    Output:
        - next_joints: Predicted next joint positions (B, 3)
        - next_suction_onoff: Predicted next suction on/off command (B, 1) - probability in [0, 1]
    """

    def __init__(self, img_height=224, img_width=224, joint_dim=3):
        super(RobotControlNetworkSingleCamera, self).__init__()

        self.joint_dim = joint_dim

        # Vision encoder for camera
        self.vision_encoder = self._build_vision_encoder()

        # Calculate flattened vision features size
        # After conv layers: 224->112->56->28->14, channels=256
        self.vision_feature_size = 256 * 14 * 14

        # Vision feature compressor
        self.vision_fc = nn.Sequential(
            nn.Linear(self.vision_feature_size, 512),
            nn.ReLU(),
            nn.Dropout(0.3),
            nn.Linear(512, 256),
            nn.ReLU()
        )

        # Joint state encoder
        self.joint_encoder = nn.Sequential(
            nn.Linear(joint_dim, 64),
            nn.ReLU(),
            nn.Linear(64, 128),
            nn.ReLU()
        )

        # Suction state encoders (separate for on/off and contact)
        self.suction_onoff_encoder = nn.Sequential(
            nn.Linear(1, 16),
            nn.ReLU(),
            nn.Linear(16, 32),
            nn.ReLU()
        )

        self.suction_contact_encoder = nn.Sequential(
            nn.Linear(1, 16),
            nn.ReLU(),
            nn.Linear(16, 32),
            nn.ReLU()
        )

        # Fusion network
        # Input: 256 (camera) + 128 (joints) + 32 (suction_onoff) + 32 (suction_contact) = 448
        self.fusion_network = nn.Sequential(
            nn.Linear(448, 512),
            nn.ReLU(),
            nn.Dropout(0.3),
            nn.Linear(512, 256),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(256, 128),
            nn.ReLU()
        )

        # Output heads
        self.joint_head = nn.Linear(128, joint_dim)
        self.suction_onoff_head = nn.Sequential(
            nn.Linear(128, 1),
            nn.Sigmoid()  # Output probability [0, 1]
        )

    def _build_vision_encoder(self):
        """Build CNN encoder for processing camera images."""
        return nn.Sequential(
            # Conv block 1: 224x224 -> 112x112
            nn.Conv2d(3, 64, kernel_size=7, stride=2, padding=3),
            nn.BatchNorm2d(64),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2),  # 112x112 -> 56x56

            # Conv block 2
            nn.Conv2d(64, 128, kernel_size=3, padding=1),
            nn.BatchNorm2d(128),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2),  # 56x56 -> 28x28

            # Conv block 3
            nn.Conv2d(128, 256, kernel_size=3, padding=1),
            nn.BatchNorm2d(256),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2),  # 28x28 -> 14x14
        )

    def forward(self, camera_img, current_joints, current_suction_onoff, current_suction_contact):
        """
        Forward pass.

        Args:
            camera_img: Tensor of shape (B, 3, H, W)
            current_joints: Tensor of shape (B, 3)
            current_suction_onoff: Tensor of shape (B, 1) - current suction on/off (0 or 1)
            current_suction_contact: Tensor of shape (B, 1) - current suction contact status (0 or 1)

        Returns:
            next_joints: Tensor of shape (B, 3)
            next_suction_onoff: Tensor of shape (B, 1) - probability in [0, 1]
        """
        # Process camera
        cam_features = self.vision_encoder(camera_img)
        cam_features = cam_features.view(cam_features.size(0), -1)
        cam_features = self.vision_fc(cam_features)

        # Process joint state
        joint_features = self.joint_encoder(current_joints)

        # Process suction states
        suction_onoff_features = self.suction_onoff_encoder(current_suction_onoff)
        suction_contact_features = self.suction_contact_encoder(current_suction_contact)

        # Concatenate all features
        combined_features = torch.cat([cam_features, joint_features,
                                      suction_onoff_features, suction_contact_features], dim=1)

        # Shared feature processing
        shared_features = self.fusion_network(combined_features)

        # Predict next joint positions and suction on/off command
        next_joints = self.joint_head(shared_features)
        next_suction_onoff = self.suction_onoff_head(shared_features)

        return next_joints, next_suction_onoff


# Example usage
if __name__ == "__main__":
    # Initialize network
    model = RobotControlNetworkSingleCamera(img_height=224, img_width=224, joint_dim=3)

    # Create dummy inputs
    batch_size = 4
    camera = torch.randn(batch_size, 3, 224, 224)
    current_joints = torch.randn(batch_size, 3)
    current_suction_onoff = torch.randint(0, 2, (batch_size, 1)).float()
    current_suction_contact = torch.randint(0, 2, (batch_size, 1)).float()

    # Forward pass
    next_joints, next_suction_onoff = model(
        camera, current_joints, current_suction_onoff, current_suction_contact
    )

    print(f"Model initialized successfully!")
    print(f"Input shapes:")
    print(f"  Camera: {camera.shape}")
    print(f"  Current joints: {current_joints.shape}")
    print(f"  Current suction on/off: {current_suction_onoff.shape}")
    print(f"  Current suction contact: {current_suction_contact.shape}")
    print(f"Output shapes:")
    print(f"  Next joints: {next_joints.shape}")
    print(f"  Next suction on/off: {next_suction_onoff.shape}")

    # Count parameters
    total_params = sum(p.numel() for p in model.parameters())
    trainable_params = sum(p.numel() for p in model.parameters() if p.requires_grad)
    print(f"\nTotal parameters: {total_params:,}")
    print(f"Trainable parameters: {trainable_params:,}")
