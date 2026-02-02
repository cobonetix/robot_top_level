"""
Utility script to create HDF5 dataset files for robot control training.

This script demonstrates how to create properly formatted HDF5 files
that can be used with the RobotDataset class.
"""

import h5py
import numpy as np
import argparse


def create_hdf5_dataset(
    output_path,
    num_samples,
    img_height=224,
    img_width=224,
    joint_dim=3,
    compression='gzip'
):
    """
    Create an HDF5 dataset file with synthetic robot control data.

    Args:
        output_path: Path where the HDF5 file will be saved
        num_samples: Number of training samples to generate
        img_height: Height of camera images
        img_width: Width of camera images
        joint_dim: Number of joint dimensions
        compression: Compression algorithm ('gzip', 'lzf', or None)

    Expected HDF5 structure:
        /camera1: (N, 3, H, W) - Camera 1 images
        /camera2: (N, 3, H, W) - Camera 2 images
        /current_joints: (N, joint_dim) - Current joint positions
        /current_suction_onoff: (N, 1) - Current suction on/off status
        /current_suction_contact: (N, 1) - Current suction contact status
        /next_joints: (N, joint_dim) - Next joint positions
        /next_suction_onoff: (N, 1) - Next suction on/off status
    """

    print(f"Creating HDF5 dataset with {num_samples} samples...")
    print(f"Output path: {output_path}")
    print(f"Image size: {img_height}x{img_width}")
    print(f"Joint dimensions: {joint_dim}")
    print(f"Compression: {compression}")

    with h5py.File(output_path, 'w') as f:
        # Create datasets with compression
        camera1 = f.create_dataset(
            'camera1',
            shape=(num_samples, 3, img_height, img_width),
            dtype='float32',
            compression=compression
        )

        camera2 = f.create_dataset(
            'camera2',
            shape=(num_samples, 3, img_height, img_width),
            dtype='float32',
            compression=compression
        )

        current_joints = f.create_dataset(
            'current_joints',
            shape=(num_samples, joint_dim),
            dtype='float32',
            compression=compression
        )

        current_suction_onoff = f.create_dataset(
            'current_suction_onoff',
            shape=(num_samples, 1),
            dtype='float32',
            compression=compression
        )

        current_suction_contact = f.create_dataset(
            'current_suction_contact',
            shape=(num_samples, 1),
            dtype='float32',
            compression=compression
        )

        next_joints = f.create_dataset(
            'next_joints',
            shape=(num_samples, joint_dim),
            dtype='float32',
            compression=compression
        )

        next_suction_onoff = f.create_dataset(
            'next_suction_onoff',
            shape=(num_samples, 1),
            dtype='float32',
            compression=compression
        )

        # Generate synthetic data in chunks to save memory
        chunk_size = 100
        for i in range(0, num_samples, chunk_size):
            end_idx = min(i + chunk_size, num_samples)
            batch_size = end_idx - i

            print(f"Generating samples {i} to {end_idx-1}...")

            # Random camera images
            camera1[i:end_idx] = np.random.randn(batch_size, 3, img_height, img_width).astype('float32')
            camera2[i:end_idx] = np.random.randn(batch_size, 3, img_height, img_width).astype('float32')

            # Random joint positions (normalized to [-1, 1])
            curr_joints = np.random.rand(batch_size, joint_dim).astype('float32') * 2 - 1
            current_joints[i:end_idx] = curr_joints

            # Next joints as current + small delta
            delta = np.random.randn(batch_size, joint_dim).astype('float32') * 0.1
            next_joints[i:end_idx] = np.clip(curr_joints + delta, -1, 1)

            # Suction cup data
            # Current suction on/off status
            curr_suction_onoff = np.random.randint(0, 2, (batch_size, 1)).astype('float32')
            current_suction_onoff[i:end_idx] = curr_suction_onoff

            # Current suction contact status (can only be 1 if suction is on)
            curr_suction_contact = np.zeros((batch_size, 1), dtype='float32')
            contact_possible = curr_suction_onoff == 1.0
            curr_suction_contact[contact_possible[:, 0]] = np.random.randint(0, 2, (contact_possible.sum(), 1)).astype('float32')
            current_suction_contact[i:end_idx] = curr_suction_contact

            # Next suction on/off (80% same, 20% toggle)
            toggle_mask = np.random.rand(batch_size, 1) < 0.2
            nxt_suction_onoff = curr_suction_onoff.copy()
            nxt_suction_onoff[toggle_mask] = 1.0 - nxt_suction_onoff[toggle_mask]
            next_suction_onoff[i:end_idx] = nxt_suction_onoff

        # Add metadata as attributes
        f.attrs['num_samples'] = num_samples
        f.attrs['img_height'] = img_height
        f.attrs['img_width'] = img_width
        f.attrs['joint_dim'] = joint_dim
        f.attrs['created_date'] = np.string_(str(np.datetime64('today')))

    print(f"\nDataset created successfully!")
    print(f"File size: {get_file_size_mb(output_path):.2f} MB")


def get_file_size_mb(filepath):
    """Get file size in megabytes."""
    import os
    return os.path.getsize(filepath) / (1024 * 1024)


def inspect_hdf5_file(filepath):
    """
    Inspect and print information about an HDF5 file.

    Args:
        filepath: Path to the HDF5 file
    """
    print(f"\n{'='*60}")
    print(f"Inspecting HDF5 file: {filepath}")
    print(f"{'='*60}")

    with h5py.File(filepath, 'r') as f:
        print("\nDatasets:")
        for name in f.keys():
            dataset = f[name]
            print(f"  {name}:")
            print(f"    Shape: {dataset.shape}")
            print(f"    Dtype: {dataset.dtype}")
            print(f"    Compression: {dataset.compression}")
            if len(dataset) > 0:
                print(f"    Min: {dataset[0].min():.4f}, Max: {dataset[0].max():.4f}")

        print("\nAttributes:")
        for key, value in f.attrs.items():
            print(f"  {key}: {value}")

        print(f"\nFile size: {get_file_size_mb(filepath):.2f} MB")


def main():
    parser = argparse.ArgumentParser(
        description='Create HDF5 datasets for robot control training'
    )

    parser.add_argument(
        '--output', '-o',
        type=str,
        default='robot_dataset.h5',
        help='Output HDF5 file path (default: robot_dataset.h5)'
    )

    parser.add_argument(
        '--samples', '-n',
        type=int,
        default=1000,
        help='Number of samples to generate (default: 1000)'
    )

    parser.add_argument(
        '--img-height',
        type=int,
        default=224,
        help='Image height (default: 224)'
    )

    parser.add_argument(
        '--img-width',
        type=int,
        default=224,
        help='Image width (default: 224)'
    )

    parser.add_argument(
        '--joint-dim',
        type=int,
        default=3,
        help='Number of joint dimensions (default: 3)'
    )

    parser.add_argument(
        '--compression',
        type=str,
        choices=['gzip', 'lzf', 'none'],
        default='gzip',
        help='Compression algorithm (default: gzip)'
    )

    parser.add_argument(
        '--inspect', '-i',
        type=str,
        help='Inspect an existing HDF5 file instead of creating a new one'
    )

    args = parser.parse_args()

    if args.inspect:
        # Inspect mode
        inspect_hdf5_file(args.inspect)
    else:
        # Create mode
        compression = None if args.compression == 'none' else args.compression

        create_hdf5_dataset(
            output_path=args.output,
            num_samples=args.samples,
            img_height=args.img_height,
            img_width=args.img_width,
            joint_dim=args.joint_dim,
            compression=compression
        )

        # Automatically inspect after creation
        inspect_hdf5_file(args.output)


if __name__ == "__main__":
    main()
