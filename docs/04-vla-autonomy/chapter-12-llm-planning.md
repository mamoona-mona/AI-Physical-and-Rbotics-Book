---
sidebar_position: 12
---

# Chapter 12: LLM Task Planning

## Learning Objectives

By the end of this chapter, you will:
- Implement LLM-based task decomposition
- Handle hierarchical planning
- Recover from failures
- Integrate with behavior trees

## 12.1 LLM Planner

```python
#!/usr/bin/env python3
"""
LLM Task Planning for Humanoid Robots

Implements:
- Hierarchical task decomposition
- OpenAI API integration
- Recovery from failures
- Constraint satisfaction

Hardware Reality Note:
    - LLM latency: 500ms-2s per request
    - Token limits: 4096-32K context
    - Must handle hallucinations
"""

from typing import List, Dict, Optional
from dataclasses import dataclass
from enum import Enum
import openai


@dataclass
class TaskStep:
    """Individual task step."""
    action: str
    parameters: Dict
    expected_outcome: str
    recovery: str


@dataclass
class TaskPlan:
    """Complete task plan."""
    description: str
    steps: List[TaskStep]
    constraints: List[str]


class LLMPlanner:
    """LLM-based task planner."""

    def __init__(self, api_key: str):
        self.client = openai.OpenAI(api_key=api_key)
        self.task_library = self._load_task_library()

    def _load_task_library(self) -> Dict:
        """Load predefined task templates."""
        return {
            'pick_object': [
                'Navigate to object location',
                'Approach object within grasp range',
                'Open gripper',
                'Grasp object',
                'Verify grasp (check force-torque)',
                'Lift object',
                'Hold object securely'
            ],
            'place_object': [
                'Navigate to destination',
                'Approach placement location',
                'Position object over target',
                'Lower object',
                'Release gripper',
                'Verify placement',
                'Retract arm to safe position'
            ],
            'walk_to': [
                'Check path is clear',
                'Plan walking trajectory',
                'Verify balance is stable',
                'Execute walk',
                'Verify arrival at target'
            ]
        }

    async def plan_from_instruction(self, instruction: str) -> TaskPlan:
        """Generate task plan from natural language."""
        response = await self.client.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": system, "content": self._get_system_prompt()},
                {"role": "user", "content": f"Plan this task: {instruction}"}
            ],
            temperature=0.1
        )

        # Parse response into TaskPlan
        return self._parse_response(response.choices[0].message.content)

    def _get_system_prompt(self) -> str:
        """Get planning system prompt."""
        return """
        You are a task planner for a humanoid robot.
        Decompose high-level tasks into step-by-step plans.
        Each step should be an atomic action the robot can execute.

        Output format:
        - Step 1: [action] with parameters
        - Step 2: [action] with parameters
        ...

        Available actions:
        - navigate_to(target)
        - approach(target, distance)
        - grasp(object, force)
        - place(object, location)
        - release(object)
        - lift(object, height)
        - lower(object, height)
        - check_sensors()
        - recover_from_failure()

        Consider:
        - Robot safety constraints
        - Stability during manipulation
        - Sensor feedback for verification
        """

    def _parse_response(self, response: str) -> TaskPlan:
        """Parse LLM response into TaskPlan."""
        steps = []
        lines = response.split('\n')

        for line in lines:
            if line.startswith('Step'):
                # Parse step
                pass

        return TaskPlan(
            description=response,
            steps=steps,
            constraints=["maintain balance", "avoid collisions"]
        )
```

## Chapter Summary

| Concept | Key Points |
|---------|------------|
| LLM Planning | Natural language → actions |
| Hierarchical | Decompose complex tasks |
| Recovery | Handle failures gracefully |

---

*Next: Chapter 13 - Speech Recognition*
