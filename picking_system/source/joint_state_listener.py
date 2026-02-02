#!/usr/bin/env python3
"""
ROS2 node that listens to /joint_states and provides joint values per arm.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dataclasses import dataclass, field
from typing import Dict, List, Optional
from threading import Lock


@dataclass
class ArmJointState:
    """Container for joint values of a single arm."""
    name: str
    joint_names: List[str] = field(default_factory=list)
    positions: List[float] = field(default_factory=list)
    velocities: List[float] = field(default_factory=list)
    efforts: List[float] = field(default_factory=list)
    timestamp: Optional[float] = None

    def as_dict(self) -> Dict[str, float]:
        """Return joint positions as a name->value dictionary."""
        return dict(zip(self.joint_names, self.positions))


class JointStateListener(Node):
    """ROS2 node that listens to /joint_states and organizes data by arm."""

    def __init__(self, arm_prefixes: Optional[List[str]] = None):
        """
        Initialize the joint state listener.

        Args:
            arm_prefixes: List of prefixes to identify arms (e.g., ['left_', 'right_']).
                         If None, defaults to ['left_', 'right_'].
        """
        super().__init__('joint_state_listener')

        self.arm_prefixes = arm_prefixes or ['left_', 'right_']
        self._lock = Lock()
        self._arm_states: Dict[str, ArmJointState] = {
            prefix.rstrip('_'): ArmJointState(name=prefix.rstrip('_'))
            for prefix in self.arm_prefixes
        }
        # For joints that don't match any prefix
        self._arm_states['other'] = ArmJointState(name='other')

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )
        self.get_logger().info('Joint state listener initialized')

    def _joint_state_callback(self, msg: JointState) -> None:
        """Process incoming joint state messages."""
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Group joints by arm prefix
        arm_joints: Dict[str, Dict[str, tuple]] = {
            arm: {} for arm in self._arm_states
        }

        for i, name in enumerate(msg.name):
            pos = msg.position[i] if i < len(msg.position) else 0.0
            vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
            eff = msg.effort[i] if i < len(msg.effort) else 0.0

            # Find matching arm prefix
            matched_arm = 'other'
            for prefix in self.arm_prefixes:
                if name.startswith(prefix):
                    matched_arm = prefix.rstrip('_')
                    break

            arm_joints[matched_arm][name] = (pos, vel, eff)

        # Update arm states
        with self._lock:
            for arm_name, joints in arm_joints.items():
                if joints:  # Only update if there are joints for this arm
                    state = self._arm_states[arm_name]
                    state.joint_names = list(joints.keys())
                    state.positions = [j[0] for j in joints.values()]
                    state.velocities = [j[1] for j in joints.values()]
                    state.efforts = [j[2] for j in joints.values()]
                    state.timestamp = timestamp

    def get_arm_state(self, arm_name: str) -> Optional[ArmJointState]:
        """Get the latest joint state for a specific arm."""
        with self._lock:
            if arm_name in self._arm_states:
                state = self._arm_states[arm_name]
                # Return a copy to avoid race conditions
                return ArmJointState(
                    name=state.name,
                    joint_names=state.joint_names.copy(),
                    positions=state.positions.copy(),
                    velocities=state.velocities.copy(),
                    efforts=state.efforts.copy(),
                    timestamp=state.timestamp
                )
        return None

    def get_all_arm_states(self) -> Dict[str, ArmJointState]:
        """Get the latest joint states for all arms."""
        with self._lock:
            return {
                name: ArmJointState(
                    name=state.name,
                    joint_names=state.joint_names.copy(),
                    positions=state.positions.copy(),
                    velocities=state.velocities.copy(),
                    efforts=state.efforts.copy(),
                    timestamp=state.timestamp
                )
                for name, state in self._arm_states.items()
                if state.joint_names  # Only include arms with data
            }


def main(args=None):
    """Example usage of the JointStateListener."""
    rclpy.init(args=args)

    # Create listener with custom arm prefixes if needed
    listener = JointStateListener(arm_prefixes=['left_', 'right_'])

    # Example: spin in a separate thread and query states
    from threading import Thread
    import time

    spin_thread = Thread(target=rclpy.spin, args=(listener,), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            time.sleep(1.0)

            # Get all arm states
            arm_states = listener.get_all_arm_states()
            for arm_name, state in arm_states.items():
                print(f"\n{arm_name} arm:")
                print(f"  Joints: {state.joint_names}")
                print(f"  Positions: {state.positions}")
                print(f"  As dict: {state.as_dict()}")

            # Or get a specific arm
            left_arm = listener.get_arm_state('left')
            if left_arm and left_arm.timestamp:
                print(f"\nLeft arm positions: {left_arm.as_dict()}")

    except KeyboardInterrupt:
        pass
    finally:
        listener.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
