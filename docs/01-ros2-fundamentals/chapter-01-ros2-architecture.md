---
sidebar_position: 1
---

# Chapter 1: ROS 2 Architecture

## Learning Objectives

By the end of this chapter, you will:
- Understand ROS 2 node architecture and communication patterns
- Implement publishers, subscribers, services, and clients
- Configure DDS middleware for real-time systems
- Apply lifecycle management for production robots

---

## 1.1 Introduction to ROS 2 Architecture

### What is ROS 2?

The Robot Operating System 2 (ROS 2) is a set of software libraries and tools for building robot applications. Unlike its predecessor ROS 1, ROS 2 was redesigned from the ground up to address critical requirements for production robotic systems:

- **Real-time performance**: Deterministic latency for control loops
- **Security**: Built-in encryption and authentication
- **Reliability**: Quality of Service (QoS) policies for guaranteed delivery
- **Life cycle management**: Proper state machines for node startup/shutdown

### The DDS Foundation

At the core of ROS 2 is the Data Distribution Service (DDS), an industry-standard middleware that provides:

```
┌─────────────────────────────────────────────────────────────────┐
│                        ROS 2 Application                         │
├─────────────────────────────────────────────────────────────────┤
│                      rclpy / rclcpp / rcljava                    │
├─────────────────────────────────────────────────────────────────┤
│                     ROS 2 Client Library (RCL)                   │
├─────────────────────────────────────────────────────────────────┤
│                    DDS Implementation (RTI, eProsima)            │
├─────────────────────────────────────────────────────────────────┤
│                      Network Transport (UDP/IP)                  │
└─────────────────────────────────────────────────────────────────┘
```

### Key Architectural Concepts

#### Nodes

A ROS 2 **node** is a single-purpose executable that performs a specific task. In humanoid robotics, typical nodes include:

- **Joint Controller Node**: Handles low-level motor control at 500Hz+
- **Sensor Fusion Node**: Combines IMU, joint encoders for state estimation
- **Motion Planner Node**: Computes trajectories for walking and manipulation
- **Perception Node**: Processes camera/LIDAR data for object detection

```python
#!/usr/bin/env python3
"""
ROS 2 Node Architecture for Humanoid Robots

Implements:
- Lifecycle node for state management
- Publisher/subscriber patterns
- Service clients for synchronous operations

Hardware Reality Note:
    - Control loop frequency: 500Hz for joint control
    - DDS latency: 1-10ms depending on QoS settings
    - Memory per node: ~50-100MB base overhead
"""

import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.lifecycle import publisher as lc_publisher
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from typing import Optional
import numpy as np


class JointControllerNode(LifecycleNode):
    """
    Lifecycle-based joint controller for humanoid robot.

    States:
    - unconfigured: Initial state, not ready
    - inactive: Configured but not controlling
    - active: Actively controlling joints

    The lifecycle state machine ensures proper initialization and
    cleanup, critical for safety-critical humanoid robots.
    """

    def __init__(self, node_name: str = "joint_controller"):
        super().__init__(node_name)

        # Declare parameters with defaults for humanoid
        self.declare_parameter('control_frequency', 500.0)
        self.declare_parameter('joint_names', [
            'left_hip_yaw', 'left_hip_roll', 'left_hip_pitch',
            'left_knee', 'left_ankle_pitch', 'left_ankle_roll',
            'right_hip_yaw', 'right_hip_roll', 'right_hip_pitch',
            'right_knee', 'right_ankle_pitch', 'right_ankle_roll',
            'torso_yaw', 'torso_pitch', 'torso_roll',
            'left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw',
            'left_elbow', 'left_wrist_roll', 'left_wrist_pitch',
            'right_shoulder_pitch', 'right_shoulder_roll', 'right_shoulder_yaw',
            'right_elbow', 'right_wrist_roll', 'right_wrist_pitch',
            'neck_pitch', 'neck_roll'
        ])
        self.declare_parameter('Kp', 100.0)
        self.declare_parameter('Kd', 10.0)

        # Get parameters
        self.joint_names = self.get_parameter('joint_names').value
        self.num_joints = len(self.joint_names)
        self.Kp = self.get_parameter('Kp').value
        self.Kd = self.get_parameter('Kd').value

        # State variables
        self.joint_positions = np.zeros(self.num_joints)
        self.joint_velocities = np.zeros(self.num_joints)
        self.joint_efforts = np.zeros(self.num_joints)
        self.target_positions = np.zeros(self.num_joints)
        self.target_efforts = np.zeros(self.num_joints)

    def on_configure(self, state) -> TransitionCallbackReturn:
        """
        Configure the node - set up publishers, subscribers, timers.

        Hardware Reality Note:
            - Motor initialization takes 100-500ms
            - Calibrate joint limits before first use
            - Verify communication with motor controllers
        """
        self.get_logger().info('Configuring joint controller...')

        # Publishers
        self._cmd_publisher = self.create_publisher(
            Float32MultiArray,
            '/joint_commands',
            10  # QoS depth
        )

        self._state_publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # Subscribers
        self._target_sub = self.create_subscription(
            Float32MultiArray,
            '/joint_targets',
            self._target_callback,
            10
        )

        self._ft_sub = self.create_subscription(
            WrenchStamped,
            '/ft_sensors',
            self._ft_callback,
            10
        )

        # Timer for control loop
        control_freq = self.get_parameter('control_frequency').value
        self._control_timer = self.create_timer(
            1.0 / control_freq,
            self._control_loop
        )

        self.get_logger().info(f'Configured for {self.num_joints} joints at {control_freq}Hz')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        """Cleanup resources."""
        self.get_logger().info('Cleaning up...')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        """Shutdown the node."""
        self.get_logger().info('Shutting down...')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        """Activate - start control loop."""
        self.get_logger().info('Activating controller...')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        """Deactivate - stop control loop."""
        self.get_logger().info('Deactivating controller...')
        return TransitionCallbackReturn.SUCCESS

    def _target_callback(self, msg: Float32MultiArray):
        """Receive target positions."""
        data = np.array(msg.data)
        if len(data) == self.num_joints:
            self.target_positions = data

    def _ft_callback(self, msg: WrenchStamped):
        """Process force-torque sensor data."""
        # Hardware Reality: F/T sensors saturate at ~100Nm
        # Apply anti-windup if readings exceed limits
        self.joint_efforts = np.clip(
            self.joint_efforts,
            -100.0,
            100.0
        )

    def _control_loop(self):
        """
        Main control loop - PD control with feedforward.

        Hardware Reality Note:
            - Motor current limits: ~30A peak per joint
            - Thermal derating after ~5 minutes at max torque
            - Position resolution: 0.001 radians
        """
        # Compute position error
        position_error = self.target_positions - self.joint_positions

        # PD control law: torque = Kp * error + Kd * velocity_error
        effort_commands = (
            self.Kp * position_error -
            self.Kd * self.joint_velocities
        )

        # Hardware Reality: Torque limits per joint type
        # Hip/knee: 150Nm max, Wrist: 10Nm max
        max_torques = np.array([
            150 if 'hip' in name or 'knee' in name else
            30 if 'shoulder' in name else
            10 for name in self.joint_names
        ])
        effort_commands = np.clip(effort_commands, -max_torques, max_torques)

        # Publish commands
        cmd_msg = Float32MultiArray()
        cmd_msg.data = effort_commands.tolist()
        self._cmd_publisher.publish(cmd_msg)

        # Publish state
        state_msg = JointState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.name = self.joint_names
        state_msg.position = self.joint_positions.tolist()
        state_msg.velocity = self.joint_velocities.tolist()
        state_msg.effort = self.joint_efforts.tolist()
        self._state_publisher.publish(state_msg)


class SensorFusionNode(Node):
    """IMU and sensor fusion for state estimation."""

    def __init__(self):
        super().__init__('sensor_fusion')

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        # Publisher for fused state
        self.state_pub = self.create_publisher(
            Odometry,
            '/robot_state',
            10
        )

        # State estimation (simplified Kalman filter)
        self.state_estimate = np.zeros(15)  # [x, y, z, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz]
        self.covariance = np.eye(15) * 0.01

    def imu_callback(self, msg: Imu):
        """Process IMU data."""
        # Accelerometer + gyroscope fusion
        pass

    def joint_callback(self, msg: JointState):
        """Update joint state."""
        pass


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    # Multi-threaded executor for parallel processing
    executor = MultiThreadedExecutor(num_threads=4)

    # Create nodes
    controller = JointControllerNode()
    sensor_fusion = SensorFusionNode()

    executor.add_node(controller)
    executor.add_node(sensor_fusion)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        controller.destroy_node()
        sensor_fusion.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 1.2 DDS Communication Patterns

### Understanding Quality of Service (QoS)

DDS QoS policies determine how data is transmitted over the network. For humanoid robots, proper QoS configuration is critical:

```python
"""
DDS Quality of Service Configuration for Real-Time Control

Hardware Reality Note:
    - Reliability: Use RELIABLE for state, BEST_EFFORT for high-frequency control
    - Durability: VOLATILE for real-time, TRANSIENT_LOCAL for configuration
    - Deadline: Must match control loop frequency
"""

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import rclpy.qos

# High-frequency control QoS (500Hz)
# Use BEST_EFFORT for control loops where dropping old data is acceptable
control_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=1  # Only keep latest sample
)

# Reliable state QoS for logging/debugging
# Use RELIABLE for data that must not be lost
state_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10  # Keep last 10 messages
)

# History settings for sensor data
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
    depth=100  # Keep last 100 samples
)
```

### Communication Patterns in ROS 2

#### Publishers and Subscribers

For continuous data streams like sensor data:

```
┌─────────────┐     /topic     ┌─────────────┐
│  Publisher  │──────────────▶│  Subscriber │
│  (Sensor)   │               │  (Processor)│
└─────────────┘               └─────────────┘
```

#### Services (Synchronous Request-Response)

For one-time operations:

```
┌─────────────┐    Request    ┌─────────────┐
│   Client    │──────────────▶│   Server    │
│             │◀──────────────│             │
└─────────────┘    Response   └─────────────┘
```

#### Actions (Long-running Tasks)

For tasks that take time and need feedback:

```
┌─────────────┐    Goal      ┌─────────────┐
│  Action     │──────────────▶│  Action     │
│   Client    │               │   Server    │
│             │◀──────────────│             │
│             │   Feedback    │             │
│             │◀──────────────│             │
│             │   Result      │             │
└─────────────┘               └─────────────┘
```

---

## 1.3 Executor and Callback Groups

### Multi-threaded Executor

For humanoid robots with parallel sensor streams:

```python
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# Reentrant callback group allows concurrent callbacks
callback_group = ReentrantCallbackGroup()

# Create node with callback group
node = MyNode(callback_group=callback_group)

# Multi-threaded executor for parallel execution
executor = MultiThreadedExecutor(num_threads=4)
executor.add_node(node)
```

### Callback Groups

- **MutuallyExclusiveCallbackGroup**: Callbacks in group execute one at a time
- **ReentrantCallbackGroup**: Callbacks can execute concurrently

---

## 1.4 Lifecycle Nodes

Lifecycle nodes provide a state machine for proper robot software management:

```
    ┌─────────┐
    │Unconfigured│
    └────┬────┘
         │ on_configure()
    ┌────▼────┐
    │Inactive │
    └────┬────┘
         │ on_activate()
    ┌────▼────┐
    │ Active  │◀──────────────────┐
    └────┬────┘                   │
         │ on_deactivate()        │
    ┌────▼────┐           ┌───────▼───────┐
    │Errored  │◀─────────│TransitionError │
    └─────────┘           └───────────────┘
         │ on_cleanup() / on_shutdown()
    ┌────▼────┐
    │Finalized│
    └─────────┘
```

---

## 1.5 Topic Naming Conventions

For humanoid robots, consistent naming is critical:

| Topic | Type | Purpose |
|-------|------|---------|
| `/joint_states` | JointState | Current joint positions |
| `/joint_commands` | Float64MultiArray | Desired positions |
| `/joint_efforts` | Float64MultiArray | Effort commands |
| `/imu/data` | Imu | IMU readings |
| `/ft_sensors` | WrenchStamped | Force-torque data |
| `/odom` | Odometry | Wheel odometry |
| `/cmd_vel` | Twist | Velocity commands |

---

## 1.6 Chapter Summary

| Concept | Key Points |
|---------|------------|
| ROS 2 Nodes | Single-purpose executables with lifecycle |
| DDS Middleware | Real-time, reliable communication |
| QoS Policies | Control reliability, latency, history |
| Lifecycle States | unconfigured → inactive → active |
| Executors | Multi-threaded for parallel processing |

---

## 1.7 Hardware Reality Checklist

Before deploying to real hardware:

- [ ] Control loop frequency matches motor driver capability
- [ ] QoS settings appropriate for data criticality
- [ ] Lifecycle states properly implemented
- [ ] Emergency stop has highest priority
- [ ] Node memory usage within limits
- [ ] Network latency measured and acceptable

---

## 1.8 Exercises

### Basic Exercise: Simple Publisher/Subscriber
Create a node that publishes joint angles at 100Hz and another that subscribes and prints them.

### Intermediate Exercise: Lifecycle Implementation
Implement a lifecycle node for a force-torque sensor with proper configuration and cleanup.

### Advanced Exercise: QoS Tuning
Experiment with different QoS settings for a high-frequency IMU stream and measure latency/throughput tradeoffs.

---

*Next: Chapter 2 - Robot Modeling*
