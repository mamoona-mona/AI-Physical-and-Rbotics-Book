---
sidebar_position: 0
---

# Module 1: ROS 2 Fundamentals

Master the Robot Operating System 2 (ROS 2) for humanoid robot control.

## Module Overview

ROS 2 represents a fundamental shift from ROS 1, designed specifically for production-quality robotic systems. This module establishes the software architecture foundation upon which all subsequent modules build.

### Why ROS 2 for Humanoids?

Humanoid robots present unique software challenges:

1. **Real-time Requirements**: Joint control at 500Hz+ requires deterministic communication
2. **Safety Critical**: Failures can cause physical damage or injury
3. **Complex Integration**: Multiple sensors, actuators, and algorithms must work together
4. **Long Lifetime**: Software must be maintainable for years of operation

ROS 2 addresses these with:

- **DDS Middleware**: Industry-standard, real-time communication
- **Lifecycle Management**: Proper state machines for robust operation
- **Security Framework**: Built-in encryption and authentication
- **Quality of Service**: Fine-grained control over data delivery

### What You'll Learn

| Week | Topic | Core Concepts | Hands-On |
|------|-------|---------------|----------|
| 1 | ROS 2 Architecture | Nodes, topics, services, DDS | Build lifecycle nodes |
| 2 | Robot Modeling | URDF, Xacro, kinematics | Create humanoid URDF |
| 3 | Actions & Behaviors | Action servers, behavior trees | Implement BT executor |
| 4 | Gazebo Simulation | World creation, plugins | Simulate walking |

---

## 1.1 The ROS 2 Stack

```
┌─────────────────────────────────────────────────────────────┐
│                    Humanoid Application                       │
│  (Motion Planning, Perception, Navigation, etc.)             │
├─────────────────────────────────────────────────────────────┤
│              ROS 2 Client Library (rclpy/rclcpp)              │
│  (Python/C++ API for node creation, topics, services)        │
├─────────────────────────────────────────────────────────────┤
│                 ROS Middleware Interface (RMW)                │
│  (Abstracts DDS implementation)                              │
├─────────────────────────────────────────────────────────────┤
│              DDS Implementation (e.g., Fast DDS)              │
│  (Actual network communication, discovery, QoS)              │
├─────────────────────────────────────────────────────────────┤
│                  Network Layer (UDP/IP)                       │
└─────────────────────────────────────────────────────────────┘
```

### Key Components

#### Nodes
Single-purpose executables that form the building blocks of robot software. Each node typically handles one aspect of robot functionality (e.g., joint control, sensor processing).

#### Topics
Asynchronous publish-subscribe communication for continuous data streams. Used for sensor data, joint states, and high-frequency control commands.

#### Services
Synchronous request-response communication for one-time operations like getting robot status or triggering actions.

#### Actions
Long-running task handling with feedback, ideal for movements like "walk to location" that take seconds to complete.

#### Parameters
Configuration values that can be changed at runtime without restarting nodes.

---

## 1.2 DDS Architecture Deep Dive

### Data Distribution Service (DDS)

DDS is an Object Management Group (OMG) standard for real-time, publish-subscribe middleware. Key concepts:

#### Domain
A logical partition of the network. All DDS participants on the same domain can discover and communicate with each other.

#### Participants
Endpoints in the DDS communication that can publish, subscribe, or both.

#### Topics
Named typed channels for data publication and subscription.

#### DataReaders and DataWriters
The interfaces for receiving and sending data on topics.

#### Quality of Service (QoS)

QoS policies control how data is transmitted:

| Policy | Purpose | Humanoid Use Case |
|--------|---------|-------------------|
| Reliability | Guarantee delivery | State logging (RELIABLE), control (BEST_EFFORT) |
| Durability | Persist data | Configuration (TRANSIENT_LOCAL), sensor data (VOLATILE) |
| Deadline | Enforce timing | Control loops must match frequency |
| LatencyBudget | Max acceptable delay | Emergency stop: &lt;100ms |

---

## 1.3 Lifecycle Management

### Why Lifecycle Nodes?

Lifecycle nodes provide a state machine for managing node behavior:

1. **Proper Initialization**: Ensure all resources are ready before operation
2. **Graceful Shutdown**: Clean up resources properly
3. **Error Recovery**: Handle failures systematically
4. **Deployment Control**: Enable/disable functionality remotely

### State Machine

```
        ┌────────────┐
        │ Unconfigured│
        └──────┬─────┘
               │ configure()
        ┌──────▼─────┐
        │  Inactive  │←────────────────┐
        └──────┬─────┘                 │
               │ activate()            │ error()
        ┌──────▼─────┐         ┌───────▼───────┐
        │   Active   │────────▶│    Error      │
        └──────┬─────┘         └───────┬───────┘
               │ deactivate()          │
        ┌──────▼─────┐         ┌───────▼───────┐
        │ Inactive   │────────▶│  Cleanup      │
        └──────┬─────┘         └───────┬───────┘
               │ cleanup()             │
        ┌──────▼─────┐         ┌───────▼───────┐
        │Unconfigured│         │  Shutdown     │
        └────────────┘         └───────────────┘
```

### Humanoid Robot States

| State | Description | Actions |
|-------|-------------|---------|
| Unconfigured | Resources not allocated | Configure: setup publishers/subscribers |
| Inactive | Ready but not active | Activate: start control loops |
| Active | Operating normally | Deactivate: stop control, maintain state |
| Errored | Failure detected | Cleanup: release resources |

---

## 1.4 Hardware Reality Considerations

### Computational Resources

| Resource | Typical Usage | Notes |
|----------|---------------|-------|
| CPU | 10-50% per node | Depends on processing complexity |
| Memory | 50-200MB per node | Includes DDS buffers |
| Network | 1-100 Mbps | Depends on sensor data rates |

### Timing Requirements

| Subsystem | Frequency | Latency Budget |
|-----------|-----------|----------------|
| Joint Control | 500-1000 Hz | < 2ms |
| IMU Processing | 200-500 Hz | < 5ms |
| State Estimation | 100-200 Hz | < 10ms |
| Motion Planning | 10-50 Hz | < 100ms |

---

## 1.5 Prerequisites

Before starting this module:

- [ ] Ubuntu 22.04 installed (native or VM)
- [ ] ROS 2 Humble installed and sourced
- [ ] Python 3.10+ with basic knowledge
- [ ] Understanding of basic robotics concepts
- [ ] Familiarity with Linux command line

### Installation Verification

```bash
# Verify ROS 2 installation
ros2 --version

# Check available packages
ros2 pkg list | head -20

# Test environment
echo $ROS_DISTRO
```

---

## 1.6 Module Structure

Each chapter follows this structure:

1. **Learning Objectives**: What you'll achieve
2. **Theory**: Conceptual explanations
3. **Code Examples**: Working implementations
4. **Hardware Reality**: Real-world constraints
5. **Summary**: Key takeaways
6. **Exercises**: Progressive challenges

---

## 1.7 Chapter Lab Setup

Create your workspace:

```bash
# Create workspace
mkdir -p ~/humanoid_ws/src
cd ~/humanoid_ws/src

# Create package
ros2 pkg create --build-type ament_python humanoid_control
cd ~/humanoid_ws

# Build
colcon build --packages-select humanoid_control

# Source
source install/setup.bash
```

---

## 1.8 Assessment

After completing this module, you should be able to:

- [ ] Create ROS 2 nodes with proper lifecycle management
- [ ] Implement all communication patterns (topics, services, actions)
- [ ] Configure DDS QoS for different data types
- [ ] Build humanoid robot models in URDF/Xacro
- [ ] Create and simulate worlds in Gazebo
- [ ] Implement behavior trees for robot control

---

## Next Steps

Proceed to [Chapter 1: ROS 2 Architecture](chapter-01-ros2-architecture) to begin your journey into ROS 2 for humanoid robotics.
