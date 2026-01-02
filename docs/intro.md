---
sidebar_position: 1
slug: /
---

# Physical AI & Humanoid Robotics

## The Embodied Intelligence Textbook

A comprehensive 16-week journey through embodied intelligence, from ROS 2 fundamentals to autonomous humanoid systems.

---

## Course Description

This textbook represents a carefully designed curriculum for mastering Physical AI - the intersection of artificial intelligence and robotic embodiment. Physical AI represents a paradigm shift from traditional software AI to systems that interact physically with the world through sensors, actuators, and real-time control loops.

### What is Physical AI?

Physical AI extends classical machine learning concepts into the physical world:

- **Embodiment**: The robot's body is not just a vessel for intelligence but an integral part of cognitive processing
- **Sensory-Motor Loops**: Real-time feedback between perception and action at 100-1000Hz
- **Physical Constraints**: Gravity, friction, torque limits, and environmental interactions
- **Safety-Critical Systems**: Failures can cause physical damage or injury

### The Textbook Philosophy

This course follows a **bottom-up then top-down** approach:

1. **Foundations (Weeks 1-4)**: Build solid understanding of ROS 2 and robot modeling
2. **Physics (Weeks 5-8)**: Master simulation and digital twin concepts
3. **Perception (Weeks 9-12)**: Learn computer vision and synthetic data
4. **Cognition (Weeks 13-16)**: Integrate VLA models for autonomous behavior

---

## Course Modules

### Module 1: ROS 2 Fundamentals (Weeks 1-4)

ROS 2 forms the backbone of modern robot software. Unlike ROS 1, ROS 2 was redesigned from the ground up for production systems with:

- **DDS Middleware**: Real-time, reliable communication
- **Lifecycle Management**: Proper state machines for node control
- **Security**: DDS-Security plugin for encrypted communication
- **Quality of Service**: Configurable reliability and latency

### Module 2: Physics Simulation (Weeks 5-8)

Simulation bridges the gap between software and hardware:

- **Rigid Body Dynamics**: Understanding forces and motion
- **Sensor Simulation**: Creating realistic sensor data with noise
- **Sim-to-Real Transfer**: Techniques to close the reality gap
- **Digital Twins**: Real-time synchronization with physical systems

### Module 3: NVIDIA Isaac Sim (Weeks 9-12)

Isaac Sim provides cutting-edge simulation capabilities:

- **USD Framework**: Universal Scene Description for robotics
- **OmniGraph**: Visual programming for sensor/actuator graphs
- **Synthetic Data**: Generating millions of labeled images
- **VSLAM**: Visual-inertial odometry for localization

### Module 4: VLA & Autonomy (Weeks 13-16)

Vision-Language-Action models represent the future of robotics:

- **LLM Planning**: Natural language task decomposition
- **Speech Recognition**: Voice command understanding
- **VLA Integration**: End-to-end perception-to-action models
- **Capstone**: Complete autonomous humanoid system

---

## Hardware Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| **GPU** | NVIDIA RTX 3060 (12GB) | RTX 3090+ (24GB) |
| **CPU** | AMD Ryzen 7 / Intel i7 | AMD Ryzen 9 / Intel i9 |
| **RAM** | 32GB | 64GB DDR5 |
| **Storage** | 1TB NVMe | 2TB+ NVMe |
| **OS** | Ubuntu 22.04 | Ubuntu 22.04/24.04 |

### Real Robot Comparison

| Parameter | Simulation | Real Hardware |
|-----------|------------|---------------|
| Control Rate | 1000 Hz | 500 Hz |
| Torque Noise | 0 | ±0.5 Nm |
| Position Resolution | Float64 | 0.001 rad |
| Sensor Latency | 0 ms | 1-10 ms |

---

## Learning Path

```
Week 1-4     Week 5-8      Week 9-12     Week 13-16
┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐
│   ROS 2 │→ │Physics  │→ │ Isaac   │→ │  VLA    │
│  Nodes  │  │Sim      │  │  Sim    │  │ Models  │
└─────────┘  └─────────┘  └─────────┘  └─────────┘
     ↓            ↓            ↓            ↓
  500Hz       1000Hz       60 FPS      100-500ms
  Control     Physics     Rendering    E2E Latency
```

---

## Weekly Breakdown

### Module 1: ROS 2 Fundamentals

| Week | Topic | Key Concepts | Code Focus |
|------|-------|--------------|------------|
| 1 | ROS 2 Architecture | Nodes, topics, services, DDS | Lifecycle nodes |
| 2 | Robot Modeling | URDF, Xacro, robot description | FK/IK solvers |
| 3 | Actions & Behaviors | Action servers, behavior trees | BT execution |
| 4 | Gazebo Simulation | World creation, plugin system | Physics plugins |

### Module 2: Physics & Simulation

| Week | Topic | Key Concepts | Code Focus |
|------|-------|--------------|------------|
| 5 | Rigid Body Dynamics | ZMP, COM, walking stability | Dynamics engine |
| 6 | Sensor Simulation | Camera, LiDAR, IMU noise | Sensor models |
| 7 | Sim-to-Real Transfer | Domain randomization, adaptation | Transfer learning |
| 8 | Digital Twin Architecture | Isaac-Gazebo sync, state est. | Real-time sync |

### Module 3: NVIDIA Isaac Sim

| Week | Topic | Key Concepts | Code Focus |
|------|-------|--------------|------------|
| 9 | Isaac Sim Basics | USD stages, OmniGraph, ROS 2 | Scene creation |
| 10 | Synthetic Data | Replicator, domain randomization | Data generation |
| 11 | VSLAM | Visual-inertial odometry, loop closure | VI-SLAM |
| 12 | Perception Pipeline | Object detection, depth estimation | CV pipeline |

### Module 4: VLA & Autonomy

| Week | Topic | Key Concepts | Code Focus |
|------|-------|--------------|------------|
| 13 | LLM Task Planning | Hierarchical planning, recovery | LLM integration |
| 14 | Speech Recognition | Whisper STT, voice commands | ASR pipeline |
| 15 | VLA Models | Vision-language-action models | E2E models |
| 16 | Capstone | End-to-end autonomous system | Full stack |

---

## Hardware Reality Notes

Throughout this course, you'll encounter **Hardware Reality** boxes highlighting real-world constraints that simulation often ignores:

### Motor Constraints
- **Torque Limits**: Peak torque vs. continuous torque
- **Thermal Management**: Derating curves and cooling
- **Velocity Limits**: Maximum joint velocities
- **Backlash**: Gear train imperfections

### Perception Constraints
- **Sensor Noise**: IMU bias drift, camera motion blur
- **Latency**: End-to-end pipeline delays
- **Occlusion**: Limited field of view
- **Lighting Variation**: Day/night, indoor/outdoor

### Computational Constraints
- **GPU Memory**: VRAM limitations on edge devices
- **Network Latency**: Cloud API delays
- **Power Consumption**: 300W+ sustained load
- **Heat Dissipation**: Thermal throttling

---

## The Chat Assistant

This textbook includes an AI-powered chat assistant powered by Groq's fast LLM inference:

### Features

1. **Global RAG**: Answer questions about the entire textbook content
2. **Selection RAG**: Help with specific highlighted text passages
3. **Code Examples**: Explain and debug ROS 2 code
4. **Concept Clarification**: Break down complex topics
5. **Exercise Help**: Guide through implementation challenges

### How to Use

- Click the chat button in the bottom right corner
- Ask questions about any topic in the course
- Highlight text for contextual help
- Request code explanations

---

## Getting Started

### Installation

```bash
# Clone the course repository
git clone https://github.com/your-username/ai-humanoid-book.git
cd ai-humanoid-book

# Install Python dependencies
pip install -r backend/requirements.txt

# Install Node dependencies (for docs)
npm install

# Start the development server
npm start
```

### Verify Installation

```bash
# Check ROS 2 installation
ros2 --version

# Verify Isaac Sim (if installed)
which isaac_sim

# Test backend is running
curl http://localhost:8000/api/v1/health
```

### Expected Output
```json
{"status": "healthy", "version": "1.0.0"}
```

---

## Contributing

This textbook is open source. Contributions welcome:

- Fix typos and errors
- Add new exercises
- Improve explanations
- Add real robot case studies
- Translate to other languages

---

*Start your journey into Physical AI by proceeding to Module 1, Week 1.*
