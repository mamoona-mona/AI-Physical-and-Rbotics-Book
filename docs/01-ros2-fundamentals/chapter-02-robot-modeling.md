---
sidebar_position: 2
---

# Chapter 2: Robot Modeling

## Learning Objectives

By the end of this chapter, you will:
- Create URDF/Xacro models for humanoid robots
- Implement forward and inverse kinematics
- Generate meshes and collision geometries
- Configure joint transmission for hardware interface

## 2.1 URDF Fundamentals

```python
#!/usr/bin/env python3
"""
Humanoid Robot URDF Generator

Generates URDF/Xacro files for humanoid robots including:
- Link geometries (mesh, box, cylinder)
- Joint definitions (revolute, continuous, fixed)
- Transmission configurations
- Gazebo plugin definitions

Hardware Reality Note:
    - Link mass: Total humanoid ~50-70kg
    - Center of mass: ~0.5m from ground at hip
    - Inertia tensor critical for stable walking
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
import xml.etree.ElementTree as ET


@dataclass
class InertialData:
    """Inertial properties for a link."""
    mass: float
    com: Tuple[float, float, float]  # Center of mass
    inertia: Tuple[float, float, float, float, float, float]  # Ixx, Iyy, Izz, Ixy, Ixz, Iyz


@dataclass
class Link:
    """Robot link specification."""
    name: str
    inertial: InertialData
    visual_geometry: Optional[str] = None  # mesh file or primitive type
    visual_origin: Tuple[float, float, float, float, float, float] = (0, 0, 0, 0, 0, 0)
    collision_geometry: Optional[str] = None
    collision_origin: Tuple[float, float, float, float, float, float] = (0, 0, 0, 0, 0, 0)


@dataclass
class Joint:
    """Robot joint specification."""
    name: str
    type: str  # revolute, continuous, prismatic, fixed
    parent: str
    child: str
    origin: Tuple[float, float, float, float, float, float]  # xyz, rpy
    axis: Tuple[float, float, float] = (1, 0, 0)
    limit_lower: float = 0.0
    limit_upper: float = 0.0
    limit_velocity: float = 0.0
    limit_effort: float = 0.0
    dynamics_damping: float = 0.0
    dynamics_friction: float = 0.0


class HumanoidURDFGenerator:
    """Generate complete URDF for humanoid robot."""

    # Standard humanoid dimensions (in meters)
    HIP_HEIGHT = 0.45
    TORSO_LENGTH = 0.35
    NECK_LENGTH = 0.12
    UPPER_LEG_LENGTH = 0.35
    LOWER_LEG_LENGTH = 0.35
    FOOT_LENGTH = 0.22
    SHOULDER_WIDTH = 0.35
    UPPER_ARM_LENGTH = 0.28
    LOWER_ARM_LENGTH = 0.25

    def __init__(self, robot_name: str = "humanoid"):
        self.robot_name = robot_name
        self.links: Dict[str, Link] = {}
        self.joints: Dict[str, Joint] = {}
        self.materials: List[Dict] = []

        # Default link inertials based on biomechanics
        self._init_standard_inertials()

    def _init_standard_inertials(self):
        """Initialize standard inertial parameters."""
        # Torso: ~15kg, COM at center
        self.register_link(Link(
            name="torso",
            inertial=InertialData(
                mass=15.0,
                com=(0, 0, 0),
                inertia=(0.1, 0.3, 0.3, 0, 0, 0)
            ),
            visual_geometry="box",
            visual_origin=(0, 0, 0.35/2, 0, 0, 0)
        ))

        # Hip: ~3kg each
        for side in ['left', 'right']:
            self.register_link(Link(
                name=f"{side}_hip",
                inertial=InertialData(
                    mass=3.0,
                    com=(0, 0, 0),
                    inertia=(0.01, 0.02, 0.02, 0, 0, 0)
                ),
                visual_geometry="cylinder",
                visual_origin=(0, 0, 0, 0, 0, 0)
            ))

        # Upper leg: ~5kg each
        for side in ['left', 'right']:
            self.register_link(Link(
                name=f"{side}_upper_leg",
                inertial=InertialData(
                    mass=5.0,
                    com=(0, 0, -self.UPPER_LEG_LENGTH/2),
                    inertia=(0.05, 0.05, 0.01, 0, 0, 0)
                ),
                visual_geometry="cylinder",
                visual_origin=(0, 0, -self.UPPER_LEG_LENGTH/2, 0, 0, 0)
            ))

        # Lower leg: ~3kg each (includes foot)
        for side in ['left', 'right']:
            self.register_link(Link(
                name=f"{side}_lower_leg",
                inertial=InertialData(
                    mass=3.0,
                    com=(0, 0, -self.LOWER_LEG_LENGTH/2),
                    inertia=(0.03, 0.03, 0.01, 0, 0, 0)
                ),
                visual_geometry="cylinder",
                visual_origin=(0, 0, -self.LOWER_LEG_LENGTH/2, 0, 0, 0)
            ))

    def register_link(self, link: Link):
        """Register a link in the robot model."""
        self.links[link.name] = link

    def register_joint(self, joint: Joint):
        """Register a joint in the robot model."""
        self.joints[joint.name] = joint

    def add_material(self, name: str, color: Tuple[float, float, float, float]):
        """Add material definition."""
        self.materials.append({
            'name': name,
            'color': color
        })

    def generate_urdf(self, xacro: bool = False) -> str:
        """Generate URDF or Xacro file."""
        if xacro:
            return self._generate_xacro()
        return self._generate_urdf()

    def _generate_urdf(self) -> str:
        """Generate standard URDF."""
        root = ET.Element('robot', name=self.robot_name)

        # Add materials
        for mat in self.materials:
            mat_elem = ET.SubElement(root, 'material', name=mat['name'])
            color = ET.SubElement(mat_elem, 'color', rgba=str(mat['color']))

        # Add links
        for name, link in self.links.items():
            link_elem = ET.SubElement(root, 'link', name=name)

            # Inertial
            inertial = ET.SubElement(link_elem, 'inertial')
            mass = ET.SubElement(inertial, 'mass', value=str(link.inertial.mass))
            origin = ET.SubElement(inertial, 'origin',
                xyz=f"{link.inertial.com[0]} {link.inertial.com[1]} {link.inertial.com[2]}",
                rpy="0 0 0")
            inertia = ET.SubElement(inertial, 'inertia',
                ixx=str(link.inertial.inertia[0]),
                ixy=str(link.inertial.inertia[3]),
                ixz=str(link.inertial.inertia[4]),
                iyy=str(link.inertial.inertia[1]),
                iyz=str(link.inertial.inertia[5]),
                izz=str(link.inertial.inertia[2]))

            # Visual
            if link.visual_geometry:
                visual = ET.SubElement(link_elem, 'visual')
                origin_v = ET.SubElement(visual, 'origin',
                    xyz=f"{link.visual_origin[0]} {link.visual_origin[1]} {link.visual_origin[2]}",
                    rpy=f"{link.visual_origin[3]} {link.visual_origin[4]} {link.visual_origin[5]}")
                visual_geo = ET.SubElement(visual, 'geometry')

                if link.visual_geometry == 'box':
                    ET.SubElement(visual_geo, 'box', size="0.1 0.1 0.1")
                elif link.visual_geometry == 'cylinder':
                    ET.SubElement(visual_geo, 'cylinder', radius="0.05", length="0.1")
                elif link.visual_geometry.endswith('.stl') or link.visual_geometry.endswith('.dae'):
                    ET.SubElement(visual_geo, 'mesh', filename=link.visual_geometry)

        # Add joints
        for name, joint in self.joints.items():
            joint_elem = ET.SubElement(root, 'joint', name=name, type=joint.type)
            ET.SubElement(joint_elem, 'origin',
                xyz=f"{joint.origin[0]} {joint.origin[1]} {joint.origin[2]}",
                rpy=f"{joint.origin[3]} {joint.origin[4]} {joint.origin[5]}")
            ET.SubElement(joint_elem, 'parent', link=joint.parent)
            ET.SubElement(joint_elem, 'child', link=joint.child)
            ET.SubElement(joint_elem, 'axis', xyz=f"{joint.axis[0]} {joint.axis[1]} {joint.axis[2]}")

            if joint.type == 'revolute':
                limit = ET.SubElement(joint_elem, 'limit',
                    lower=str(joint.limit_lower),
                    upper=str(joint.limit_upper),
                    effort=str(joint.limit_effort),
                    velocity=str(joint.limit_velocity))

        return ET.tostring(root, encoding='unicode')

    def build_bipedal_chain(self):
        """Build standard bipedal leg chain."""
        # Left leg joints
        self.register_joint(Joint(
            name="left_hip_yaw",
            type="revolute",
            parent="torso",
            child="left_hip",
            origin=(0, 0.1, -self.HIP_HEIGHT, 0, 0, 0),
            axis=(0, 0, 1),
            limit_lower=-0.5, limit_upper=0.5,
            limit_effort=150, limit_velocity=5
        ))

        self.register_joint(Joint(
            name="left_hip_roll",
            type="revolute",
            parent="left_hip",
            child="left_upper_leg",
            origin=(0, 0, 0, 0, 0, 0),
            axis=(1, 0, 0),
            limit_lower=-0.3, limit_upper=1.5,
            limit_effort=150, limit_velocity=5
        ))

        self.register_joint(Joint(
            name="left_hip_pitch",
            type="revolute",
            parent="left_upper_leg",
            child="left_lower_leg",
            origin=(0, 0, -self.UPPER_LEG_LENGTH, 0, 0, 0),
            axis=(1, 0, 0),
            limit_lower=-2.0, limit_upper=0.5,
            limit_effort=150, limit_velocity=5
        ))

        self.register_joint(Joint(
            name="left_knee",
            type="revolute",
            parent="left_lower_leg",
            child="left_foot",
            origin=(0, 0, -self.LOWER_LEG_LENGTH, 0, 0, 0),
            axis=(1, 0, 0),
            limit_lower=0, limit_upper=2.5,
            limit_effort=150, limit_velocity=8
        ))

        self.register_joint(Joint(
            name="left_ankle_pitch",
            type="revolute",
            parent="left_foot",
            child="left_foot_link",
            origin=(0, 0, 0, 0, 0, 0),
            axis=(1, 0, 0),
            limit_lower=-0.5, limit_upper=0.5,
            limit_effort=50, limit_velocity=5
        ))

        # Right leg (mirror of left)
        self.register_joint(Joint(
            name="right_hip_yaw",
            type="revolute",
            parent="torso",
            child="right_hip",
            origin=(0, -0.1, -self.HIP_HEIGHT, 0, 0, 0),
            axis=(0, 0, 1),
            limit_lower=-0.5, limit_upper=0.5,
            limit_effort=150, limit_velocity=5
        ))

        self.register_joint(Joint(
            name="right_hip_roll",
            type="revolute",
            parent="right_hip",
            child="right_upper_leg",
            origin=(0, 0, 0, 0, 0, 0),
            axis=(1, 0, 0),
            limit_lower=-0.3, limit_upper=1.5,
            limit_effort=150, limit_velocity=5
        ))

        self.register_joint(Joint(
            name="right_hip_pitch",
            type="revolute",
            parent="right_upper_leg",
            child="right_lower_leg",
            origin=(0, 0, -self.UPPER_LEG_LENGTH, 0, 0, 0),
            axis=(1, 0, 0),
            limit_lower=-2.0, limit_upper=0.5,
            limit_effort=150, limit_velocity=5
        ))

        self.register_joint(Joint(
            name="right_knee",
            type="revolute",
            parent="right_lower_leg",
            child="right_foot",
            origin=(0, 0, -self.LOWER_LEG_LENGTH, 0, 0, 0),
            axis=(1, 0, 0),
            limit_lower=0, limit_upper=2.5,
            limit_effort=150, limit_velocity=8
        ))

        self.register_joint(Joint(
            name="right_ankle_pitch",
            type="revolute",
            parent="right_foot",
            child="right_foot_link",
            origin=(0, 0, 0, 0, 0, 0),
            axis=(1, 0, 0),
            limit_lower=-0.5, limit_upper=0.5,
            limit_effort=50, limit_velocity=5
        ))


def main():
    """Generate and save URDF."""
    generator = HumanoidURDFGenerator("bipedal_robot")
    generator.build_bipedal_chain()

    urdf_content = generator.generate_urdf()

    with open("humanoid.urdf", "w") as f:
        f.write(urdf_content)

    print("Generated humanoid.urdf")
    print(f"Links: {len(generator.links)}")
    print(f"Joints: {len(generator.joints)}")


if __name__ == '__main__':
    main()
```

## 2.2 Forward Kinematics

```python
"""
Forward Kinematics for Humanoid Robot

Computes end-effector positions from joint angles using
homogeneous transformation matrices.

Hardware Reality Note:
    - DH parameters introduce numerical drift
    - Recompute periodically from sensor data
    - Joint encoder resolution: 0.001 radians
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple
import numpy as np


@dataclass
class DHParams:
    """Denavit-Hartenberg parameters."""
    a: float  # Link length
    d: float  # Link offset
    alpha: float  # Link twist
    theta: float  # Joint angle


def homogeneous_transform(dh: DHParams) -> np.ndarray:
    """
    Compute homogeneous transformation matrix from DH params.

    T = [cos(θ)  -sin(θ)cos(α)   sin(θ)sin(α)   a*cos(θ)]
        [sin(θ)   cos(θ)cos(α)  -cos(θ)sin(α)   a*sin(θ)]
        [0        sin(α)         cos(α)          d        ]
        [0        0              0               1        ]
    """
    c = np.cos(dh.theta)
    s = np.sin(dh.theta)
    ca = np.cos(dh.alpha)
    sa = np.sin(dh.alpha)

    T = np.array([
        [c, -s * ca, s * sa, dh.a * c],
        [s, c * ca, -c * sa, dh.a * s],
        [0, sa, ca, dh.d],
        [0, 0, 0, 1]
    ])

    return T


class ForwardKinematics:
    """Forward kinematics solver for humanoid."""

    def __init__(self):
        # 7-DOF arm DH parameters (meters, radians)
        self.arm_dh = [
            DHParams(a=0, d=0.35, alpha=-np.pi/2, theta=0),      # shoulder
            DHParams(a=0, d=0, alpha=np.pi/2, theta=0),         # shoulder pitch
            DHParams(a=0.28, d=0, alpha=0, theta=0),            # upper arm
            DHParams(a=0, d=0, alpha=-np.pi/2, theta=0),        # elbow
            DHParams(a=0.25, d=0, alpha=0, theta=0),            # forearm
            DHParams(a=0, d=0, alpha=-np.pi/2, theta=0),        # wrist pitch
            DHParams(a=0, d=0.1, alpha=0, theta=0),             # wrist roll
        ]

        # Base to shoulder transformation
        self.base_transform = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0.1],
            [0, 0, 1, 0.6],
            [0, 0, 0, 1]
        ])

    def compute_link_transform(self, joint_angles: List[float]) -> List[np.ndarray]:
        """Compute transformation for each link."""
        transforms = []
        T = self.base_transform

        for i, dh in enumerate(self.arm_dh):
            # Update theta with joint angle
            dh = DHParams(
                a=dh.a,
                d=dh.d,
                alpha=dh.alpha,
                theta=joint_angles[i] if i < len(joint_angles) else 0
            )

            T = T @ homogeneous_transform(dh)
            transforms.append(T.copy())

        return transforms

    def compute_end_effector(self, joint_angles: List[float]) -> np.ndarray:
        """
        Compute end-effector position and orientation.

        Returns:
            T: 4x4 homogeneous transformation matrix
        """
        transforms = self.compute_link_transform(joint_angles)
        return transforms[-1]

    def get_position(self, joint_angles: List[float]) -> np.ndarray:
        """Get end-effector position (x, y, z)."""
        T = self.compute_end_effector(joint_angles)
        return T[:3, 3]

    def get_orientation(self, joint_angles: List[float]) -> np.ndarray:
        """Get end-effector orientation as rotation matrix."""
        T = self.compute_end_effector(joint_angles)
        return T[:3, :3]

    def get_jacobian(self, joint_angles: List[float], num_links: int = 7) -> np.ndarray:
        """
        Compute geometric Jacobian.

        Hardware Reality Note:
            - Jacobian singularities at arm extension
            - Joint velocity limits: ~5 rad/s
            - Computed at 500Hz for control
        """
        # Get all link positions
        transforms = self.compute_link_transform(joint_angles)

        # End-effector position
        p_ee = self.get_position(joint_angles)

        J = np.zeros((6, num_links))

        for i in range(num_links):
            # Link position
            T_i = transforms[i]
            p_i = T_i[:3, 3]

            # Link orientation (z-axis)
            z_i = T_i[:3, 2]

            # Linear velocity contribution
            J[:3, i] = np.cross(z_i, p_ee - p_i)

            # Angular velocity contribution
            J[3:, i] = z_i

        return J


def main():
    """Demo forward kinematics."""
    fk = ForwardKinematics()

    # Neutral pose
    joint_angles = [0] * 7

    # Compute position
    pos = fk.get_position(joint_angles)
    print(f"Neutral pose end-effector: {pos}")

    # Compute with joint angles
    joint_angles = [0, 0, np.pi/4, -np.pi/2, 0, 0, 0]
    pos = fk.get_position(joint_angles)
    print(f"Bent arm end-effector: {pos}")

    # Compute Jacobian
    J = fk.get_jacobian(joint_angles)
    print(f"Jacobian shape: {J.shape}")


if __name__ == '__main__':
    main()
```

## 2.3 Chapter Summary

| Concept | Key Points |
|---------|------------|
| URDF/Xacro | Robot description format |
| DH Parameters | Forward kinematics |
| Collision Geometry | Safety and planning |
| Inertia Tensor | Dynamics simulation |

## 2.4 Exercises

1. **Basic**: Generate URDF for 2-link arm
2. **Intermediate**: Implement inverse kinematics for arm
3. **Advanced**: Add 3D mesh visualization

---

*Next: Chapter 3 - Actions & Behaviors*
