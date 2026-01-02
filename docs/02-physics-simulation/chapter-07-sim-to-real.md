---
sidebar_position: 7
---

# Chapter 7: Sim-to-Real Transfer

## Learning Objectives

By the end of this chapter, you will:
- Apply domain randomization for transfer
- Identify and close sim-to-real gaps
- Use system identification for parameter tuning
- Implement adaptive control for transfer

## 7.1 Domain Randomization

```python
#!/usr/bin/env python3
"""
Domain Randomization for Sim-to-Real Transfer

Implements:
- Randomization of physics parameters
- Visual randomization of textures/lighting
- Sensor noise injection
- Randomization schedule

Hardware Reality Note:
    - Randomization range: ±20-50% of nominal values
    - Training: 10,000+ randomizations needed
    - Transfer success: Depends on randomization coverage
"""

import numpy as np
from dataclasses import dataclass
from typing import Dict, List, Callable
import random


@dataclass
class RandomizationConfig:
    """Domain randomization configuration."""
    # Physics randomization
    friction_range: tuple = (0.3, 0.8)
    mass_variation: float = 0.1  # 10% variation
    damping_variation: float = 0.2  # 20% variation

    # Visual randomization
    lighting_variance: float = 0.3
    texture_noise: float = 0.1

    # Sensor randomization
    imu_noise_scale: float = 1.5
    joint_offset_range: tuple = (-0.02, 0.02)  # radians


class DomainRandomizer:
    """Domain randomization for sim-to-real transfer."""

    def __init__(self, config: RandomizationConfig = None):
        self.config = config or RandomizationConfig()
        self.rng = np.random.default_rng(42)

    def randomize_physics(self) -> Dict:
        """Randomize physics parameters."""
        return {
            'friction': self.rng.uniform(*self.config.friction_range),
            'mass_multiplier': 1.0 + self.rng.uniform(
                -self.config.mass_variation,
                self.config.mass_variation
            ),
            'damping_multiplier': 1.0 + self.rng.uniform(
                -self.config.damping_variation,
                self.config.damping_variation
            ),
            'gravity': self.rng.uniform(9.7, 9.9),  # Slight gravity variation
        }

    def randomize_sensors(self) -> Dict:
        """Randomize sensor parameters."""
        return {
            'imu_noise_scale': self.rng.uniform(
                1.0, self.config.imu_noise_scale
            ),
            'joint_offset': self.rng.uniform(
                *self.config.joint_offset_range
            ),
            'camera_exposure': self.rng.uniform(0.005, 0.02),
            'camera_gain': self.rng.uniform(1.0, 4.0),
        }

    def randomize_visual(self) -> Dict:
        """Randomize visual parameters."""
        return {
            'ambient_light': self.rng.uniform(0.3, 0.7),
            'directional_light': self.rng.uniform(0.5, 1.0),
            'texture_variation': self.rng.uniform(0, self.config.texture_noise),
            'background_color': self.rng.uniform(0, 1, 3).tolist(),
        }

    def get_randomization(self) -> Dict:
        """Get complete randomization set."""
        return {
            'physics': self.randomize_physics(),
            'sensors': self.randomize_sensors(),
            'visual': self.randomize_visual(),
        }


class SimToRealGapAnalyzer:
    """Analyze and quantify sim-to-real gaps."""

    def __init__(self):
        self.sim_metrics = {}
        self.real_metrics = {}

    def measure_sim(self, robot_state: Dict) -> Dict:
        """Measure metrics in simulation."""
        return {
            'joint_velocity_variance': np.var(robot_state.get('velocities', [])),
            'position_accuracy': 0.0,  # Perfect in sim
            'force_accuracy': 0.0,
        }

    def measure_real(self, robot_state: Dict) -> Dict:
        """Measure metrics on real robot."""
        return {
            'joint_velocity_variance': np.var(robot_state.get('velocities', [0])),
            'position_accuracy': 0.001,  # Encoder resolution
            'force_accuracy': 0.5,  # Nm
        }

    def compute_gap(self, metric: str) -> float:
        """Compute sim-to-real gap for metric."""
        sim_val = self.sim_metrics.get(metric, 0)
        real_val = self.real_metrics.get(metric, 0)

        if abs(real_val) < 1e-6:
            return 0

        return abs(real_val - sim_val) / (abs(real_val) + 1e-6)
