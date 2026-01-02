---
sidebar_position: 4
---

# Chapter 4: Gazebo Simulation

## Learning Objectives

By the end of this chapter, you will:
- Create Gazebo worlds with SDF models
- Implement Gazebo plugins for sensor simulation
- Bridge ROS 2 with Gazebo simulation
- Apply sim-to-real transfer techniques

## 4.1 Gazebo-ROS 2 Bridge

```python
#!/usr/bin/env python3
"""
Gazebo Simulation Bridge for Humanoid Robot

Implements:
- ROS 2 to Gazebo communication
- Sensor plugin integration
- Physics parameter tuning

Hardware Reality Note:
    - Simulation timestep: 1ms (1000Hz)
    - Real-time factor: Target 1.0
    - Physics solver: 50-100 iterations per step
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import WrenchStamped, Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import Contacts
from typing import Dict, Optional
import numpy as np


class GazeboBridgeNode(Node):
    """Bridge ROS 2 commands to Gazebo simulation."""

    def __init__(self):
        super().__init__('gazebo_bridge')

        # Joint command publishers (Gazebo expects specific topics)
        self._joint_pub = self.create_publisher(
            Float64MultiArray,
            '/humanoid/joint_effort_controller/commands',
            10
        )

        # State subscribers (from Gazebo)
        self._joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self._imu_sub = self.create_subscription(
            Imu,
            '/imu_sensor',
            self.imu_callback,
            10
        )

        self._ft_sub = self.create_subscription(
            WrenchStamped,
            '/ft_sensor',
            self.ft_callback,
            10
        )

        self._contact_sub = self.create_subscription(
            Contacts,
            '/bumper_contacts',
            self.contact_callback,
            10
        )

        # Publishers for simulated data
        self._joint_pub_sim = self.create_publisher(
            JointState,
            '/humanoid/joint_states',
            10
        )

        # Timer for simulation loop
        self._timer = self.create_timer(0.001, self._simulation_step)

        # State
        self.joint_positions: Dict[str, float] = {}
        self.joint_velocities: Dict[str, float] = {}
        self.joint_efforts: Dict[str, float] = {}
        self.imu_data: Optional[Imu] = None
        self.ft_data: Optional[WrenchStamped] = None
        self.contacts: Contacts = Contacts()

        # Physics parameters
        self.max_effort = 150.0  # Nm
        self.velocity_limit = 10.0  # rad/s

        self.get_logger().info('Gazebo bridge initialized')

    def joint_state_callback(self, msg: JointState):
        """Process joint state from Gazebo."""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.joint_velocities[name] = msg.velocity[i]
            if i < len(msg.effort):
                self.joint_efforts[name] = msg.effort[i]

    def imu_callback(self, msg: Imu):
        """Process IMU data."""
        self.imu_data = msg

    def ft_callback(self, msg: WrenchStamped):
        """Process force-torque sensor data."""
        self.ft_data = msg

    def contact_callback(self, msg: Contacts):
        """Process contact detection."""
        self.contacts = msg

    def _simulation_step(self):
        """
        Simulation step - run at 1000Hz.

        Hardware Reality Note:
            - Joint dynamics include friction and damping
            - Contact dynamics require small timestep
            - Solver convergence affects accuracy
        """
        # Publish simulated joint states
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_positions.keys())
        msg.position = [self.joint_positions[n] for n in msg.name]
        msg.velocity = [self.joint_velocities.get(n, 0) for n in msg.name]
        msg.effort = [self.joint_efforts.get(n, 0) for n in msg.name]
        self._joint_pub_sim.publish(msg)

    def send_effort_command(self, joint_name: str, effort: float):
        """
        Send effort command to joint.

        Hardware Reality Note:
            - Effort saturation at motor limits
            - Thermal model affects max effort
            - Back-EMF limits velocity
        """
        # Clamp effort
        effort = np.clip(effort, -self.max_effort, self.max_effort)

        msg = Float64MultiArray()
        msg.data = [effort]
        # Would publish to joint-specific topic
        self._joint_pub.publish(msg)


class SimulationParameters:
    """Tunable simulation parameters."""

    def __init__(self):
        # Physics solver parameters
        self.solver_type = 'pgs'  # Projected Gauss-Seidel
        self.solver_iterations = 50
        self.solver_tolerance = 1e-6

        # Contact parameters
        self.contact_max_corrective_vel = 100.0
        self.contact_erp = 0.2  # Error reduction parameter
        self.contact_cfm = 0.0  # Constraint force mixing

        # Friction parameters
        self.friction_coefficient = 0.5
        self.rolling_friction = 0.01

    def to_sdf(self) -> str:
        """Generate SDF physics parameters."""
        return f"""
        <physics name="physics" type="ode">
            <max_step_size>0.001</max_step_size>
            <real_time_factor>1.0</real_time_factor>
            <real_time_update_rate>1000</real_time_update_rate>
            <solver type="{self.solver_type}">
                <iters>{self.solver_iterations}</iters>
                <tolerance>{self.solver_tolerance}</tolerance>
            </solver>
            <contact>
                <max_corrective_vel>{self.contact_max_corrective_vel}</max_corrective_vel>
                <erp>{self.contact_erp}</erp>
                <cfm>{self.contact_cfm}</cfm>
            </contact>
            <friction>
                <coefficient>{self.friction_coefficient}</coefficient>
                <rolling_friction>{self.rolling_friction}</rolling_friction>
            </friction>
        </physics>
        """


def main():
    """Start Gazebo bridge."""
    rclpy.init()
    node = GazeboBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 4.2 Chapter Summary

| Concept | Key Points |
|---------|------------|
| Gazebo Bridge | ROS 2 to simulation |
| Physics Solver | ODE parameters |
| Contact Dynamics | Friction, ERP, CFM |
| Sim-to-Real | Domain gap analysis |

## 4.3 Exercises

1. **Basic**: Create simple Gazebo world
2. **Intermediate**: Tune physics parameters
3. **Advanced**: Implement contact simulation

---

*Module 1 Complete: Proceed to Module 2*
