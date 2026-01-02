---
sidebar_position: 3
---

# Chapter 3: Actions and Behavior Trees

## Learning Objectives

By the end of this chapter, you will:
- Implement ROS 2 action servers and clients
- Build behavior trees for complex robot behaviors
- Handle preemption and recovery
- Coordinate multi-joint movements

## 3.1 ROS 2 Actions

```python
#!/usr/bin/env python3
"""
ROS 2 Action Server for Walking Control

Implements:
- Action server for trajectory execution
- Preemption handling for emergency stops
- Feedback for progress monitoring

Hardware Reality Note:
    - Walking trajectory: 500-1000ms execution time
    - Preemption must complete within 100ms
    - Force feedback triggers immediate stop
"""

import rclpy
from rclpy.action import ActionServer, CancelGoalService
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Path
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from typing import Optional, List
import numpy as np
import time


class WalkAction(Node):
    """Action server for walking trajectories."""

    def __init__(self):
        super().__init__('walk_action')

        # Action server
        self._action_server = ActionServer(
            self,
            WalkTrajectory,
            '/humanoid/walk',
            self.execute_callback,
            cancel_callback=self.cancel_callback
        )

        # Subscribers
        self._imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )

        # Publishers
        self._cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        # State
        self._current_goal: Optional[ServerGoalHandle] = None
        self._imu_data: Optional[Imu] = None
        self._emergency_stop = False

        self.get_logger().info('Walk action server started')

    def imu_callback(self, msg: Imu):
        """Monitor IMU for balance detection."""
        self._imu_data = msg

        # Hardware Reality: Fall detection thresholds
        # Pitch > 45 degrees or roll > 45 degrees
        # Angular velocity > 5 rad/s
        linear_accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        angular_vel = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

        if np.linalg.norm(angular_vel) > 5.0:
            self._emergency_stop = True
            self.get_logger().warn('Emergency stop: high angular velocity')

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        """Handle goal cancellation."""
        self.get_logger().info('Goal cancelled')
        self._current_goal = None
        # Send zero velocity
        self._publish_stop()
        return CancelGoalService.Response().CANCEL

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        """
        Execute walking trajectory.

        Hardware Reality Note:
            - ZMP must stay within support polygon
            - COM velocity limited to 0.5 m/s
            - Step frequency: 1-2 Hz
        """
        self._current_goal = goal_handle
        self._emergency_stop = False

        goal = goal_handle.request
        trajectory = goal.trajectory
        speed = goal.speed

        self.get_logger().info(f'Executing walk: {len(trajectory.poses)} steps at {speed} m/s')

        # Execute trajectory
        for i, pose in enumerate(trajectory.poses):
            # Check for preemption
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._publish_stop()
                return WalkTrajectory.Result()

            # Check for emergency
            if self._emergency_stop:
                goal_handle.aborted(
                    WalkTrajectory.Result(),
                    'Emergency stop triggered'
                )
                self._publish_stop()
                return WalkTrajectory.Result()

            # Publish velocity command
            self._publish_velocity(pose, speed)

            # Send feedback
            feedback = WalkTrajectory.Feedback()
            feedback.current_step = i + 1
            feedback.total_steps = len(trajectory.poses)
            feedback.current_pose = pose
            goal_handle.publish_feedback(feedback)

            # Hardware Reality: Step timing
            # Each step takes ~500ms
            time.sleep(0.5)

        # Complete
        goal_handle.succeed()
        result = WalkTrajectory.Result()
        result.final_pose = trajectory.poses[-1]
        result.steps_completed = len(trajectory.poses)
        self._publish_stop()

        return result

    def _publish_velocity(self, pose: Pose, speed: float):
        """Publish velocity command."""
        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = 0.0
        self._cmd_vel_pub.publish(cmd)

    def _publish_stop(self):
        """Publish stop command."""
        cmd = Twist()
        self._cmd_vel_pub.publish(cmd)


# Action definitions (typically in separate file)
class WalkTrajectory:
    """Action definition for walking."""
    Goal = type('Goal', (), {
        'trajectory': Path,
        'speed': 0.3,
        'style': 'normal'
    })
    Feedback = type('Feedback', (), {
        'current_step': 0,
        'total_steps': 0,
        'current_pose': Pose()
    })
    Result = type('Result', (), {
        'final_pose': Pose(),
        'steps_completed': 0,
        'success': True
    })
```

## 3.2 Behavior Trees

```python
#!/usr/bin/env python3
"""
Behavior Tree Implementation for Humanoid Robot

Implements:
- Control nodes (Sequence, Selector, Parallel)
- Action nodes (Walk, Grasp, Place)
- Condition nodes (Battery check, Safety check)

Hardware Reality Note:
    - Tree tick rate: 10-50Hz
    - Safety conditions checked every tick
    - Recovery actions retry up to 3 times
"""

from enum import Enum
from typing import Optional, List, Callable
import time


class NodeStatus(Enum):
    """Behavior tree node status."""
    SUCCESS = 'success'
    FAILURE = 'failure'
    RUNNING = 'running'


class TreeNode:
    """Base class for behavior tree nodes."""

    def __init__(self, name: str):
        self.name = name
        self.status = NodeStatus.FAILURE
        self.parent: Optional[TreeNode] = None

    def tick(self) -> NodeStatus:
        """Execute one tick of this node."""
        raise NotImplementedError

    def reset(self):
        """Reset node state."""
        self.status = NodeStatus.FAILURE


class SequenceNode(TreeNode):
    """Sequence node: runs children in order until one fails."""

    def __init__(self, name: str, children: List[TreeNode] = None):
        super().__init__(name)
        self.children = children or []
        self.current_child = 0

    def add_child(self, child: TreeNode):
        """Add a child node."""
        child.parent = self
        self.children.append(child)

    def tick(self) -> NodeStatus:
        if not self.children:
            return NodeStatus.SUCCESS

        if self.status != NodeStatus.RUNNING:
            self.current_child = 0

        while self.current_child < len(self.children):
            child = self.children[self.current_child]
            child_status = child.tick()

            if child_status == NodeStatus.RUNNING:
                self.status = NodeStatus.RUNNING
                return NodeStatus.RUNNING

            if child_status == NodeStatus.FAILURE:
                self.status = NodeStatus.FAILURE
                self.current_child = 0
                return NodeStatus.FAILURE

            self.current_child += 1

        self.status = NodeStatus.SUCCESS
        self.current_child = 0
        return NodeStatus.SUCCESS

    def reset(self):
        super().reset()
        for child in self.children:
            child.reset()


class SelectorNode(TreeNode):
    """Selector node: tries children until one succeeds."""

    def __init__(self, name: str, children: List[TreeNode] = None):
        super().__init__(name)
        self.children = children or []
        self.current_child = 0

    def add_child(self, child: TreeNode):
        """Add a child node."""
        child.parent = self
        self.children.append(child)

    def tick(self) -> NodeStatus:
        if not self.children:
            return NodeStatus.FAILURE

        if self.status != NodeStatus.RUNNING:
            self.current_child = 0

        while self.current_child < len(self.children):
            child = self.children[self.current_child]
            child_status = child.tick()

            if child_status == NodeStatus.RUNNING:
                self.status = NodeStatus.RUNNING
                return NodeStatus.RUNNING

            if child_status == NodeStatus.SUCCESS:
                self.status = NodeStatus.SUCCESS
                self.current_child = 0
                return NodeStatus.SUCCESS

            self.current_child += 1

        self.status = NodeStatus.FAILURE
        self.current_child = 0
        return NodeStatus.FAILURE

    def reset(self):
        super().reset()
        for child in self.children:
            child.reset()


class ActionNode(TreeNode):
    """Action node for robot actions."""

    def __init__(
        self,
        name: str,
        action_fn: Callable[[], NodeStatus],
        recovery_attempts: int = 3
    ):
        super().__init__(name)
        self.action_fn = action_fn
        self.recovery_attempts = recovery_attempts
        self.attempt_count = 0

    def tick(self) -> NodeStatus:
        if self.status == NodeStatus.RUNNING:
            result = self.action_fn()
            if result != NodeStatus.RUNNING:
                if result == NodeStatus.SUCCESS:
                    self.attempt_count = 0
                else:
                    self.attempt_count += 1
                self.status = result
            return self.status

        self.attempt_count = 0
        self.status = NodeStatus.RUNNING
        result = self.action_fn()
        self.status = result
        return result

    def reset(self):
        super().reset()
        self.attempt_count = 0


class ConditionNode(TreeNode):
    """Condition node for checks (always succeeds or fails)."""

    def __init__(self, name: str, condition_fn: Callable[[], bool]):
        super().__init__(name)
        self.condition_fn = condition_fn

    def tick(self) -> NodeStatus:
        if self.condition_fn():
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE


class InverterNode(TreeNode):
    """Inverter node: inverts child status."""

    def __init__(self, name: str, child: TreeNode):
        super().__init__(name)
        self.child = child
        child.parent = self

    def tick(self) -> NodeStatus:
        result = self.child.tick()
        if result == NodeStatus.SUCCESS:
            return NodeStatus.FAILURE
        if result == NodeStatus.FAILURE:
            return NodeStatus.SUCCESS
        return result

    def reset(self):
        super().reset()
        self.child.reset()


class HumanoidBehaviorTree:
    """Complete behavior tree for humanoid robot."""

    def __init__(self):
        self.root = self._build_tree()
        self.status = NodeStatus.FAILURE
        self.last_tick = time.time()

    def _build_tree(self) -> TreeNode:
        """Build the complete behavior tree."""
        # Main selector - try to accomplish task
        main = SelectorNode('main')

        # Safety override - always check
        safety = SelectorNode('safety_check')
        safety.add_child(ConditionNode('battery_ok', self._check_battery))
        safety.add_child(ConditionNode('not_falling', self._check_stability))
        safety.add_child(ActionNode('emergency_stop', self._emergency_stop))

        # Main sequence
        task = SequenceNode('task_sequence')

        # Approach task
        approach = SequenceNode('approach')
        approach.add_child(ConditionNode('task_available', self._has_task))
        approach.add_child(ActionNode('navigate_to', self._navigate))
        approach.add_child(ActionNode('approach_target', self._approach))

        # Manipulation task
        manipulate = SequenceNode('manipulate')
        manipulate.add_child(ActionNode('grasp', self._grasp))
        manipulate.add_child(ActionNode('lift', self._lift))
        manipulate.add_child(ActionNode('place', self._place))

        # Add to main
        main.add_child(approach)
        main.add_child(manipulate)
        main.add_child(safety)

        return main

    def tick(self) -> NodeStatus:
        """Tick the behavior tree."""
        self.status = self.root.tick()
        self.last_tick = time.time()
        return self.status

    # Condition functions
    def _check_battery(self) -> bool:
        """Check battery level (>20%)."""
        return True  # Simulated

    def _check_stability(self) -> bool:
        """Check robot stability."""
        return True  # Simulated

    def _has_task(self) -> bool:
        """Check if task is available."""
        return True

    # Action functions
    def _navigate(self) -> NodeStatus:
        """Navigate to target location."""
        time.sleep(0.1)
        return NodeStatus.SUCCESS

    def _approach(self) -> NodeStatus:
        """Approach target for manipulation."""
        time.sleep(0.1)
        return NodeStatus.SUCCESS

    def _grasp(self) -> NodeStatus:
        """Grasp object."""
        time.sleep(0.2)
        return NodeStatus.SUCCESS

    def _lift(self) -> NodeStatus:
        """Lift object."""
        time.sleep(0.1)
        return NodeStatus.SUCCESS

    def _place(self) -> NodeStatus:
        """Place object."""
        time.sleep(0.1)
        return NodeStatus.SUCCESS

    def _emergency_stop(self) -> NodeStatus:
        """Execute emergency stop."""
        return NodeStatus.SUCCESS


def main():
    """Demo behavior tree."""
    bt = HumanoidBehaviorTree()

    # Run behavior tree
    while True:
        status = bt.tick()
        print(f'Tree status: {status.value}')
        if status == NodeStatus.SUCCESS:
            break
        time.sleep(0.1)


if __name__ == '__main__':
    main()
```

## 3.3 Chapter Summary

| Concept | Key Points |
|---------|------------|
| Action Server | Asynchronous task execution |
| Behavior Tree | Hierarchical task decomposition |
| Preemption | Emergency stop handling |
| Recovery | Retry with backoff |

## 3.4 Exercises

1. **Basic**: Implement simple action server
2. **Intermediate**: Build sequence/selector tree
3. **Advanced**: Add recovery actions

---

*Next: Chapter 4 - Gazebo Simulation*
