---
sidebar_position: 5
---

# Chapter 5: Rigid Body Dynamics

## Learning Objectives

By the end of this chapter, you will:
- Implement ZMP (Zero Moment Point) calculations
- Design stable walking patterns
- Model joint dynamics with friction and damping
- Handle balance disturbances

## 5.1 ZMP Dynamics

```python
#!/usr/bin/env python3
"""
Rigid Body Dynamics for Humanoid Walking

Implements:
- ZMP computation and stability margin
- Center of Mass (COM) trajectory generation
- Walking pattern generation

Hardware Reality Note:
    - ZMP must stay within support polygon
    - COM velocity limited to prevent falls
    - Disturbance rejection requires ~100ms response
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple, List
import numpy as np


@dataclass
class ZMPState:
    """ZMP computation state."""
    com_position: np.ndarray  # 3D COM
    com_velocity: np.ndarray  # 3D COM velocity
    zmp_position: np.ndarray  # 2D ZMP on ground
    stability_margin: float   # Margin to support edge


class HumanoidDynamics:
    """Rigid body dynamics for humanoid robot."""

    def __init__(self, total_mass: float = 60.0):
        self.total_mass = total_mass  # kg

        # Link masses (approximate humanoid)
        self.link_masses = {
            'torso': 15.0,
            'head': 3.0,
            'upper_arm': 2.0,
            'lower_arm': 1.5,
            'upper_leg': 5.0,
            'lower_leg': 3.0,
            'foot': 0.5,
        }

        # COM offsets from joint frames
        self.com_offsets = {
            'torso': np.array([0, 0, 0]),
            'head': np.array([0, 0, 0.1]),
            'upper_arm': np.array([0, 0, -0.14]),
            'lower_arm': np.array([0, 0, -0.12]),
            'upper_leg': np.array([0, 0, -0.17]),
            'lower_leg': np.array([0, 0, -0.17]),
            'foot': np.array([0, 0, 0]),
        }

        # Gravity
        self.g = 9.81  # m/s^2

    def compute_com(
        self,
        joint_positions: dict,
        link_positions: dict
    ) -> np.ndarray:
        """
        Compute Center of Mass from link positions.

        COM = sum(m_i * p_i) / sum(m_i)
        """
        total_mass = 0.0
        weighted_sum = np.zeros(3)

        for link_name, mass in self.link_masses.items():
            if link_name in link_positions:
                pos = link_positions[link_name]
                offset = self.com_offsets.get(link_name, np.zeros(3))
                com_pos = pos + offset

                weighted_sum += mass * com_pos
                total_mass += mass

        if total_mass > 0:
            return weighted_sum / total_mass
        return np.zeros(3)

    def compute_zmp(
        self,
        com: np.ndarray,
        com_acceleration: np.ndarray,
        total_mass: float = None
    ) -> np.ndarray:
        """
        Compute Zero Moment Point.

        ZMP_x = (g * COM_x - COM_z * COM_accel_x) / (g + COM_accel_z)
        ZMP_y = (g * COM_y - COM_z * COM_accel_y) / (g + COM_accel_z)

        Hardware Reality Note:
            - Measurement noise: ±2cm
            - Update rate: 100-500Hz
            - Requires accurate COM estimation
        """
        if total_mass is None:
            total_mass = self.total_mass

        g = self.g

        # ZMP formula from linear inverted pendulum
        com_x, com_y, com_z = com
        com_accel_x, com_accel_y, com_accel_z = com_acceleration

        # Avoid division by zero
        denom = g + com_accel_z
        if abs(denom) < 1e-6:
            denom = 1e-6

        zmp_x = (g * com_x - com_z * com_accel_x) / denom
        zmp_y = (g * com_y - com_z * com_accel_y) / denom

        return np.array([zmp_x, zmp_y])

    def compute_stability_margin(
        self,
        zmp: np.ndarray,
        support_polygon: List[np.ndarray]
    ) -> float:
        """
        Compute distance from ZMP to support polygon edge.

        Hardware Reality Note:
            - Minimum margin for stability: 2-3cm
            - Walking requires margin > 1cm
            - External disturbance reduces margin
        """
        if len(support_polygon) < 3:
            return float('inf')

        # Compute minimum distance to each edge
        min_distance = float('inf')

        n = len(support_polygon)
        for i in range(n):
            p1 = support_polygon[i]
            p2 = support_polygon[(i + 1) % n]

            # Distance from point to line segment
            edge = p2 - p1
            edge_len = np.linalg.norm(edge)
            if edge_len < 1e-6:
                continue

            # Project ZMP onto edge
            t = np.dot(zmp - p1, edge) / (edge_len ** 2)
            t = np.clip(t, 0, 1)

            closest = p1 + t * edge
            distance = np.linalg.norm(zmp - closest)

            min_distance = min(min_distance, distance)

        return min_distance

    def linear_inverted_pendulum(
        self,
        com_height: float,
        zmp_ref: np.ndarray,
        initial_com: np.ndarray,
        initial_com_vel: np.ndarray,
        dt: float,
        steps: int
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Generate COM trajectory using LIP model.

        Hardware Reality Note:
            - LIP height must remain constant
            - Valid for quasi-static walking
            - Walking speed limited by COM height
        """
        # LIP dynamics: COM_x'' = (g/h) * (COM_x - ZMP_x)
        omega = np.sqrt(self.g / com_height)

        com_positions = [initial_com.copy()]
        com_velocities = [initial_com_vel.copy()]

        for _ in range(steps):
            com_pos = com_positions[-1]
            com_vel = com_velocities[-1]

            # Acceleration from LIP
            accel = omega**2 * (com_pos[:2] - zmp_ref)

            # Euler integration
            new_vel = com_vel.copy()
            new_vel[:2] += accel * dt

            new_pos = com_pos.copy()
            new_pos[:2] += new_vel[:2] * dt

            com_positions.append(new_pos.copy())
            com_velocities.append(new_vel.copy())

        return np.array(com_positions), np.array(com_velocities)


class WalkingController:
    """Walking pattern generator and controller."""

    def __init__(self):
        self.step_length = 0.15  # m
        self.step_width = 0.10   # m
        self.step_height = 0.05  # m
        self.step_duration = 0.5  # s
        self.double_support_ratio = 0.2  # % of step in double support

        # Walking parameters
        self.com_height = 0.45   # m
        self.cycle_time = 1.0    # s
        self.step_frequency = 1.0 / self.cycle_time

    def generate_walk_trajectory(
        self,
        num_steps: int,
        direction: str = 'forward'
    ) -> List[dict]:
        """Generate walking trajectory."""
        trajectory = []

        # Walking direction
        if direction == 'forward':
            step_direction = np.array([1, 0])
        elif direction == 'backward':
            step_direction = np.array([-1, 0])
        elif direction == 'left':
            step_direction = np.array([0, 1])
        elif direction == 'right':
            step_direction = np.array([0, -1])
        else:
            step_direction = np.array([1, 0])

        # Generate foot placements
        left_foot_pos = np.array([0, self.step_width / 2, 0])
        right_foot_pos = np.array([0, -self.step_width / 2, 0])

        for step in range(num_steps):
            # Alternate feet
            if step % 2 == 0:
                # Swing left foot
                swing_foot = 'left'
                stance_foot = right_foot_pos.copy()
                new_swing_pos = left_foot_pos + self.step_length * step_direction

                # Foot trajectory (swing phase)
                for t in np.linspace(0, 1, 10):
                    foot_pos = (1 - t) * left_foot_pos + t * new_swing_pos
                    foot_pos[2] = self.step_height * np.sin(np.pi * t)

                    trajectory.append({
                        'step': step,
                        'phase': t,
                        'left_foot': foot_pos.copy(),
                        'right_foot': stance_foot.copy(),
                        'support': 'left' if swing_foot == 'left' else 'right'
                    })

                left_foot_pos = new_swing_pos
            else:
                # Swing right foot
                swing_foot = 'right'
                stance_foot = left_foot_pos.copy()
                new_swing_pos = right_foot_pos + self.step_length * step_direction

                for t in np.linspace(0, 1, 10):
                    foot_pos = (1 - t) * right_foot_pos + t * new_swing_pos
                    foot_pos[2] = self.step_height * np.sin(np.pi * t)

                    trajectory.append({
                        'step': step,
                        'phase': t,
                        'left_foot': stance_foot.copy(),
                        'right_foot': foot_pos.copy(),
                        'support': 'left' if swing_foot == 'left' else 'right'
                    })

                right_foot_pos = new_swing_pos

        return trajectory


def main():
    """Demo dynamics computation."""
    dynamics = HumanoidDynamics()

    # Test COM computation
    link_positions = {
        'torso': np.array([0, 0, 0.5]),
        'head': np.array([0, 0, 0.7]),
        'upper_leg': np.array([0, 0.05, 0.25]),
        'lower_leg': np.array([0, 0.05, 0.0]),
    }

    com = dynamics.compute_com({}, link_positions)
    print(f"COM: {com}")

    # Test ZMP computation
    zmp = dynamics.compute_zmp(
        com=np.array([0, 0, 0.45]),
        com_acceleration=np.array([0, 0, 0])
    )
    print(f"ZMP: {zmp}")


if __name__ == '__main__':
    main()
```

## 5.2 Chapter Summary

| Concept | Key Points |
|---------|------------|
| ZMP | Zero Moment Point stability |
| COM | Center of Mass tracking |
| LIP | Linear Inverted Pendulum |
| Stability Margin | Distance to support edge |

## 5.3 Exercises

1. **Basic**: Compute ZMP for static pose
2. **Intermediate**: Generate walking trajectory
3. **Advanced**: Implement disturbance rejection
