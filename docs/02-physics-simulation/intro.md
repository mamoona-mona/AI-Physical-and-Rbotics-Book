---
sidebar_position: 0
---

# Module 2: Physics Simulation

Master physics-based simulation for humanoid robots.

## Module Overview

Physics simulation is the bridge between software algorithms and real-world robot behavior. This module covers the fundamental physics that govern humanoid robot motion and how to simulate them accurately.

### Why Physics Simulation?

Simulation is essential for humanoid robot development:

1. **Safety**: Test dangerous scenarios without risk
2. **Cost Reduction**: Iterate without hardware wear
3. **Development Speed**: Parallel development of software
4. **Algorithm Development**: Test and tune controllers
5. **Sim-to-Real Transfer**: Bridge the reality gap

### The Reality Gap

Simulation and reality differ in important ways:

| Aspect | Simulation | Reality |
|--------|------------|---------|
| Contact Dynamics | Perfect contact, instant resolution | Deformation, hysteresis |
| Sensor Noise | Gaussian noise models | Complex, correlated noise |
| Delays | Zero latency | 1-10ms sensor-to-actuator |
| Torque | Exact commands | Saturation, windup |
| Friction | Simple models | Stiction, hysteresis |

Understanding and bridging this gap is critical for successful deployment.

---

## 2.1 Physics Fundamentals

### Rigid Body Dynamics

Humanoid robots consist of rigid bodies connected by joints:

```
┌─────────────────────────────────────────────────────────────┐
│                    Rigid Body Model                          │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│    Link i-1 ───┬── Joint i ───┬── Link i+1                  │
│               │               │                              │
│         ┌─────▼─────┐   ┌─────▼─────┐                        │
│         │  Mass mi  │   │  Mass mi+1│                        │
│         │  Inertia I│   │  Inertia I│                        │
│         │  Pose Ti  │   │  Pose Ti+1│                        │
│         └───────────┘   └───────────┘                        │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

#### Equations of Motion

The dynamics of a robotic system are described by:

```
tau = M(q) * d2q + C(q, dq) * dq + G(q) + J(q)^T * F_ext
```

Where:
- **tau**: Joint torques (N·m)
- **M(q)**: Mass (inertia) matrix
- **C(q, dq)**: Coriolis/centrifugal matrix
- **G(q)**: Gravitational torques (N·m)
- **J(q)**: Jacobian matrix
- **F_ext**: External forces (contacts, N)

### Center of Mass (COM)

The Center of Mass is critical for balance:

```
COM = (sum of m_i * p_i) / (sum of m_i)
```

For bipedal walking, the COM height relative to the support foot determines stability.

### Zero Moment Point (ZMP)

The ZMP is the point where the total horizontal moment equals zero:

```
ZMP_x = (sum of m_i * (g * COM_x - d2COM_x * COM_z)) / (sum of m_i * (g - d2COM_z))
ZMP_y = (sum of m_i * (g * COM_y - d2COM_y * COM_z)) / (sum of m_i * (g - d2COM_z))
```

Where:
- g = gravitational acceleration (9.81 m/s²)
- COM_x, COM_y, COM_z = Center of Mass position
- d2COM_x, d2COM_y, d2COM_z = COM acceleration

**Stability Condition**: ZMP must lie within the support polygon.

---

## 2.2 Walking Dynamics

### Linear Inverted Pendulum (LIP)

The LIP model simplifies humanoid walking:

```
          COM (mass point)
              *
             /|
            / |
           /  | h (constant height)
          /   |
         /    |
        *─────*
      ZMP    Support Point
```

**Dynamics Equation**:
```
d2x/dt^2 = (g/h) * (x - x_ZMP)
```

Where:
- g = gravitational acceleration (9.81 m/s²)
- h = constant COM height (m)
- x = horizontal COM position
- x_ZMP = ZMP horizontal position

This simple model captures essential walking dynamics while being computationally tractable.

### Gait Phases

| Phase | Duration | Description |
|-------|----------|-------------|
| Double Support | 20-30% | Both feet on ground |
| Single Support | 70-80% | One foot on ground |
| Swing | 40-50% | Foot moving through air |

---

## 2.3 Sensor Simulation

### IMU Simulation

IMUs measure:

- **Accelerometer**: Linear acceleration (m/s²)
- **Gyroscope**: Angular velocity (rad/s)
- **Magnetometer**: Magnetic field (optional)

#### Noise Models

Real IMU data includes:

- **White Noise**: Random fluctuations
- **Bias Drift**: Slowly varying offset
- **Scale Factor Error**: Proportionality error
- **Misalignment**: Cross-axis sensitivity

```python
# IMU noise parameters (typical MEMS IMU)
IMU_NOISE = {
    'accelerometer': {
        'noise_density': 0.002,  # m/s²/√Hz
        'bias_stability': 0.001,  # m/s²
        'scale_factor': 0.001,    # % FS
    },
    'gyroscope': {
        'noise_density': 0.005,   # rad/s/√Hz
        'bias_stability': 0.001,  # rad/s
        'scale_factor': 0.001,    # % FS
    }
}
```

### Force-Torque Sensors

F/T sensors measure:

- **Force**: F_x, F_y, F_z (3-axis, Newtons)
- **Torque**: tau_x, tau_y, tau_z (3-axis, N·m)

Common applications:
- **Grounded contacts**: Foot-ground interaction
- **Wrist sensors**: Grip force estimation
- **Joint torque**: Direct torque sensing

---

## 2.4 Simulation Platforms

### Gazebo

Gazebo is the most common robotics simulator:

- **Physics Engines**: ODE, Bullet, DART, Simbody
- **Sensor Models**: Camera, LiDAR, IMU, etc.
- **Plugin System**: Custom sensors and controllers
- **ROS Integration**: Native ROS 2 support

### NVIDIA Isaac Sim

Isaac Sim provides:

- **USD Framework**: Universal Scene Description
- **PhysX 5**: High-fidelity physics
- **RTX Rendering**: Photorealistic visuals
- **Synthetic Data**: Automated labeling

### Comparison

| Feature | Gazebo | Isaac Sim |
|---------|--------|-----------|
| Physics Fidelity | Good | Excellent |
| Rendering | Basic | RTX |
| Synthetic Data | Limited | Native |
| Performance | Real-time | GPU-accelerated |
| ROS 2 Support | Native | Excellent |

---

## 2.5 Sim-to-Real Transfer

### Domain Randomization

Randomize simulation parameters during training:

| Parameter | Range |
|-----------|-------|
| Friction coefficient | 0.3 - 1.0 |
| Contact stiffness | 1000 - 10000 N/m |
| Damping ratio | 0.1 - 1.0 |
| Mass variation | +/- 10% |
| Delay | 0 - 20 ms |

### System Identification

Identify simulation parameters from real robot data:

1. Collect real robot trajectories
2. Match simulation parameters
3. Minimize trajectory error
4. Validate on held-out data

---

## 2.6 Digital Twins

A digital twin maintains a real-time synchronized simulation:

```
┌──────────────┐     Real-time Sync      ┌──────────────┐
│  Real Robot  │◀──────────────────────▶│ Digital Twin │
└──────────────┘                        └──────────────┘
       │                                       │
       │ Sensor Data                    State Estimation
       ▼                                       │
┌──────────────┐                        ┌──────────────┐
│  Control     │                        │   Simulation │
│  Commands    │                        │   & Prediction│
└──────────────┘                        └──────────────┘
```

### Applications

- **State Estimation**: Fill sensor gaps
- **Prediction**: Anticipate future states
- **Anomaly Detection**: Detect sensor/actuator faults
- **Offline Analysis**: Replay and analyze incidents

---

## 2.7 Hardware Reality

### Simulation vs. Reality Gap

| Parameter | Simulation | Real Robot | Gap |
|-----------|------------|------------|-----|
| Control Rate | 1000 Hz | 500 Hz | 2x |
| Torque Noise | 0 | +/- 0.5 Nm | Real only |
| Position Resolution | Float64 | 0.001 rad | 1e-15 |
| Sensor Latency | 0 ms | 1-10 ms | Real only |
| Contact Stiffness | Infinite | Finite | Physics |

### Mitigating the Gap

1. **Accurate Modeling**: Detailed robot description
2. **Realistic Noise**: Sensor-specific noise models
3. **Domain Randomization**: Robustness through variation
4. **System ID**: Parameter tuning from real data
5. **Progressive Sim-to-Real**: Gradual complexity increase

---

## 2.8 Prerequisites

Before starting this module:

- [ ] Complete Module 1 (ROS 2 Fundamentals)
- [ ] Understanding of linear algebra
- [ ] Basic physics (forces, torques, dynamics)
- [ ] Python programming
- [ ] Gazebo installed (optional for simulation)

### Installation Verification

```bash
# Verify Gazebo installation
gz --version

# Check physics plugins
ls /usr/lib/x86_64-linux-gnu/libgazebo_*.so

# Verify Isaac Sim (if installed)
echo $ISAAC_SIM_PATH
```

---

## 2.9 Module Structure

Each chapter follows this structure:

1. **Learning Objectives**: What you'll achieve
2. **Theory**: Mathematical foundations
3. **Implementation**: Code examples
4. **Hardware Reality**: Real-world constraints
5. **Summary**: Key takeaways
6. **Exercises**: Progressive challenges

---

## 2.10 Assessment

After completing this module, you should be able to:

- [ ] Implement ZMP-based walking controllers
- [ ] Create realistic sensor models
- [ ] Configure simulation environments
- [ ] Apply sim-to-real transfer techniques
- [ ] Build digital twin systems

---

## Next Steps

Proceed to [Chapter 5: Rigid Body Dynamics](chapter-05-rigid-body-dynamics) to learn the mathematical foundations of humanoid robot motion.
