---
sidebar_position: 6
---

# Chapter 6: Sensor Simulation

## Learning Objectives

By the end of this chapter, you will:
- Simulate camera, LiDAR, and IMU sensors
- Add realistic noise models
- Implement sensor calibration simulation
- Model sensor latency and dropout

## 6.1 Sensor Noise Models

```python
#!/usr/bin/env python3
"""
Realistic Sensor Simulation for Humanoid Robots

Implements:
- Camera noise (read noise, dark current, PRNU)
- IMU noise (gyroscope, accelerometer)
- LiDAR noise (range, angle)
- Force-torque sensor simulation

Hardware Reality Note:
    - IMU noise density: 0.01-0.1 deg/s/√Hz
    - Camera rolling shutter: 1-33ms
    - LiDAR range accuracy: ±3cm
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple, Optional
import cv2


@dataclass
class CameraNoiseModel:
    """Camera sensor noise parameters."""
    read_noise_std: float = 1.5  # electrons
    dark_current: float = 0.1    # e-/pixel/s
    exposure_time: float = 0.01  # seconds
    prnu_std: float = 0.01       # % non-uniformity
    bit_depth: int = 12
    resolution: Tuple[int, int] = (640, 480)
    gain: float = 1.0


@dataclass
class IMUNoiseModel:
    """IMU sensor noise parameters."""
    # Gyroscope
    gyro_noise_density: float = 0.005  # deg/s/√Hz
    gyro_random_walk: float = 0.001    # deg/s/√Hz
    gyro_scale_factor: float = 1.0
    gyro_bias_instability: float = 0.1  # deg/s

    # Accelerometer
    accel_noise_density: float = 0.002  # m/s²/√Hz
    accel_random_walk: float = 0.001    # m/s²/√Hz
    accel_scale_factor: float = 1.0
    accel_bias_instability: float = 0.01  # m/s²

    # Update rate
    sampling_rate: float = 500.0  # Hz


class CameraNoise:
    """Realistic camera noise simulation."""

    def __init__(self, params: CameraNoiseModel = None):
        self.params = params or CameraNoiseModel()
        self rng = np.random.default_rng(42)

    def add_noise(self, image: np.ndarray) -> np.ndarray:
        """
        Add realistic noise to camera image.

        Hardware Reality Note:
            - Read noise: Gaussian distribution
            - Dark current: Poisson, temperature dependent
            - PRNU: Pixel-to-pixel variation
        """
        # Convert to electrons
        if image.dtype != np.float64:
            image = image.astype(np.float64)

        max_value = (2 ** self.params.bit_depth) - 1
        electrons = image / max_value * 16384  # Assume 14-bit ADC base

        # Shot noise (Poisson)
        shot_noise = np.sqrt(np.maximum(electrons, 0))
        electrons = self.rng.normal(electrons, shot_noise)

        # Read noise
        read_noise = self.rng.normal(0, self.params.read_noise_std, electrons.shape)
        electrons += read_noise

        # Dark current noise
        dark_electrons = self.params.dark_current * self.params.exposure_time * 1000
        dark_noise = self.rng.normal(0, np.sqrt(max(dark_electrons, 0)), electrons.shape)
        electrons += dark_noise

        # PRNU (Photo-Response Non-Uniformity)
        prnu = 1 + self.rng.normal(0, self.params.prnu_std, electrons.shape)
        electrons *= prnu

        # Convert back to pixel values
        electrons = np.clip(electrons, 0, 16384 * 100)
        result = (electrons / 16384) * max_value
        result = np.clip(result, 0, max_value)

        return result.astype(np.uint8)


class IMUSimulator:
    """Realistic IMU sensor simulation."""

    def __init__(self, params: IMUNoiseModel = None):
        self.params = params or IMUNoiseModel()
        self.rng = np.random.default_rng(42)

        # State variables
        self.gyro_bias = np.zeros(3)
        self.accel_bias = np.zeros(3)
        self.last_time = None

        # Initialize biases
        self._init_biases()

    def _init_biases(self):
        """Initialize sensor biases."""
        self.gyro_bias = self.rng.normal(0, 0.1, 3) * self.params.gyro_bias_instability
        self.accel_bias = self.rng.normal(0, 0.01, 3) * self.params.accel_bias_instability

    def add_imu_noise(
        self,
        true_gyro: np.ndarray,
        true_accel: np.ndarray,
        dt: float
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Add realistic noise to IMU measurements.

        Hardware Reality Note:
            - Noise density: White noise at sensor output
            - Random walk: Integrated noise (bias drift)
            - Scale factor: Linear gain error
        """
        # Convert noise densities to per-sample values
        gyro_noise_std = self.params.gyro_noise_density * np.sqrt(
            self.params.sampling_rate / 2
        )
        accel_noise_std = self.params.accel_noise_density * np.sqrt(
            self.params.sampling_rate / 2
        )

        # Add white noise
        gyro_noise = self.rng.normal(0, gyro_noise_std, 3)
        accel_noise = self.rng.normal(0, accel_noise_std, 3)

        # Update bias random walk
        gyro_random_walk = self.rng.normal(
            0, self.params.gyro_random_walk * np.sqrt(dt), 3
        )
        accel_random_walk = self.rng.normal(
            0, self.params.accel_random_walk * np.sqrt(dt), 3
        )

        self.gyro_bias += gyro_random_walk
        self.accel_bias += accel_random_walk

        # Apply noise and biases
        noisy_gyro = self.params.gyro_scale_factor * true_gyro + self.gyro_bias + gyro_noise
        noisy_accel = self.params.accel_scale_factor * true_accel + self.accel_bias + accel_noise

        return noisy_gyro, noisy_accel


class LiDARSimulator:
    """Realistic LiDAR sensor simulation."""

    def __init__(self):
        self.rng = np.random.default_rng(42)

        # LiDAR parameters
        self.range_accuracy = 0.03  # ±3cm
        self.angular_resolution = 0.5  # degrees
        self.range_min = 0.1  # meters
        self.range_max = 50.0  # meters
        self.num_beams = 32

    def add_lidar_noise(
        self,
        point_cloud: np.ndarray,
        surface_normals: np.ndarray = None
    ) -> np.ndarray:
        """
        Add realistic noise to LiDAR point cloud.

        Hardware Reality Note:
            - Range noise: Depends on reflectivity
            - Angular noise: Encoder quantization
            - Multi-path: False returns from reflections
        """
        if point_cloud.shape[0] == 0:
            return point_cloud

        noisy_points = point_cloud.copy()

        # Distance noise (Gaussian)
        distances = np.linalg.norm(point_cloud, axis=1)
        range_noise = self.rng.normal(0, self.range_accuracy, len(distances))

        # Noise increases with distance
        range_noise *= (1 + distances / 20.0)

        # Add noise along ray direction
        directions = point_cloud / (distances[:, np.newaxis] + 1e-6)
        noise_3d = directions * range_noise[:, np.newaxis]

        noisy_points += noise_3d

        # Simulate dropout (missing returns)
        dropout_mask = self.rng.random(len(distances)) < 0.01
        noisy_points[dropout_mask] = np.nan

        return noisy_points
```

## 6.2 Chapter Summary

| Sensor | Noise Type | Realistic Value |
|--------|------------|-----------------|
| Camera | Read noise | 1-10 e- |
| IMU | Gyro noise | 0.01 deg/s/√Hz |
| LiDAR | Range | ±3cm |
| F/T | Force | ±0.5N |

## 6.3 Exercises

1. **Basic**: Implement camera noise model
2. **Intermediate**: Add IMU bias drift
3. **Advanced**: Model sensor dropout

---

*Next: Chapter 7 - Sim-to-Real Transfer*
