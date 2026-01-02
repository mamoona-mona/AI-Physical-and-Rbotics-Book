---
sidebar_position: 8
---

# Chapter 8: Digital Twin Architecture

## Learning Objectives

By the end of this chapter, you will:
- Design digital twin architecture
- Implement Isaac Sim to Gazebo synchronization
- Build state estimation pipeline
- Create real-time simulation loop

## 8.1 Digital Twin System

```python
#!/usr/bin/env python3
"""
Digital Twin Architecture for Humanoid Robot

Implements:
- Real-time simulation sync
- State estimation from sensors
- Physics-based prediction
- Performance monitoring

Hardware Reality Note:
    - Sync latency: <5ms target
    - Simulation real-time factor: 1.0
    - State estimation rate: 100-500Hz
"""

import numpy as np
from typing import Dict, Optional
from dataclasses import dataclass
import asyncio


@dataclass
class TwinState:
    """Digital twin state."""
    joint_positions: np.ndarray
    joint_velocities: np.ndarray
    base_pose: np.ndarray  # 4x4 transform
    base_velocity: np.ndarray  # 6D velocity
    imu_reading: np.ndarray  # 6D IMU
    contact_states: Dict[str, bool]


class DigitalTwin:
    """Digital twin for humanoid robot."""

    def __init__(self):
        # State
        self.state = TwinState(
            joint_positions=np.zeros(28),
            joint_velocities=np.zeros(28),
            base_pose=np.eye(4),
            base_velocity=np.zeros(6),
            imu_reading=np.zeros(6),
            contact_states={}
        )

        # Physics model
        self.mass_matrix = np.eye(28 + 6)  # Joints + base
        self.gravity_vector = np.zeros(28 + 6)
        self.coriolis_vector = np.zeros(28 + 6)

        # Synchronization
        self.last_sync_time = None
        self.sync_latency = 0.0
        self.realtime_factor = 1.0

        # Performance
        self.compute_time = 0.0
        self.fps = 0.0

    def sync_from_sensors(self, sensor_data: Dict):
        """Synchronize state from real sensors."""
        import time
        start = time.time()

        # Update from sensor data
        if 'joint_positions' in sensor_data:
            self.state.joint_positions = np.array(sensor_data['joint_positions'])

        if 'joint_velocities' in sensor_data:
            self.state.joint_velocities = np.array(sensor_data['joint_velocities'])

        if 'imu' in sensor_data:
            self.state.imu_reading = np.array(sensor_data['imu'])

        if 'contacts' in sensor_data:
            self.state.contact_states = sensor_data['contacts']

        self.sync_latency = time.time() - start

    def predict(self, dt: float) -> TwinState:
        """Predict next state using physics."""
        # Simple forward integration (would use full dynamics)
        new_positions = self.state.joint_positions + self.state.joint_velocities * dt
        return TwinState(
            joint_positions=new_positions,
            joint_velocities=self.state.joint_velocities,
            base_pose=self.state.base_pose,
            base_velocity=self.state.base_velocity,
            imu_reading=self.state.imu_reading,
            contact_states=self.state.contact_states
        )

    def get_performance_metrics(self) -> Dict:
        """Get twin performance metrics."""
        return {
            'sync_latency_ms': self.sync_latency * 1000,
            'compute_time_ms': self.compute_time * 1000,
            'realtime_factor': self.realtime_factor,
            'fps': self.fps,
        }
```

## Module 2 Summary

| Chapter | Key Technologies |
|---------|-----------------|
| 5 | ZMP, LIP, COM dynamics |
| 6 | Sensor noise, IMU/LiDAR |
| 7 | Domain randomization |
| 8 | Digital twin sync |

---

*Module 2 Complete: Proceed to Module 3*
