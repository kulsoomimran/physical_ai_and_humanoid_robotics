#!/usr/bin/env python3
# Copyright 2025 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Action Planning and Execution Demo for Chapter 22: Action Planning and Execution.

This example demonstrates action planning and execution concepts including:
- Task decomposition and planning
- Path planning and navigation
- Manipulation planning
- Execution monitoring and control
- Plan execution and replanning
"""

import numpy as np
import time
from dataclasses import dataclass
from typing import List, Dict, Any, Optional, Tuple, Callable
import heapq
import math


@dataclass
class RobotState:
    """Represents the state of a robot."""
    position: Tuple[float, float, float]  # x, y, z
    orientation: Tuple[float, float, float, float]  # quaternion
    joints: List[float]  # joint angles
    gripper_state: str  # open/closed
    timestamp: float


@dataclass
class Task:
    """Represents a high-level task."""
    id: str
    description: str
    goal_conditions: List[Dict[str, Any]]
    priority: int = 1


@dataclass
class Action:
    """Represents a low-level action."""
    action_type: str  # 'navigation', 'manipulation', 'interaction', etc.
    parameters: Dict[str, Any]
    duration: float  # estimated duration in seconds
    preconditions: List[Dict[str, Any]]
    effects: List[Dict[str, Any]]


@dataclass
class Plan:
    """Represents a sequence of actions."""
    id: str
    task_id: str
    actions: List[Action]
    cost: float
    timestamp: float


class PathPlanner:
    """Simple path planner using A* algorithm."""

    def __init__(self, world_map: np.ndarray, resolution: float = 1.0):
        self.world_map = world_map  # 2D occupancy grid (0 = free, 1 = occupied)
        self.resolution = resolution

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Calculate heuristic distance between two points."""
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighbors for a position."""
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue  # Skip current position
                new_x, new_y = pos[0] + dx, pos[1] + dy
                if (0 <= new_x < self.world_map.shape[0] and
                    0 <= new_y < self.world_map.shape[1] and
                    self.world_map[new_x, new_y] == 0):  # Check if free space
                    neighbors.append((new_x, new_y))
        return neighbors

    def plan_path(self, start: Tuple[float, float], goal: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        """Plan a path from start to goal using A*."""
        # Convert to grid coordinates
        start_grid = (int(start[0] / self.resolution), int(start[1] / self.resolution))
        goal_grid = (int(goal[0] / self.resolution), int(goal[1] / self.resolution))

        # Check if start or goal are in occupied space
        if (self.world_map[start_grid[0], start_grid[1]] == 1 or
            self.world_map[goal_grid[0], goal_grid[1]] == 1):
            return None

        # A* algorithm
        frontier = [(0, start_grid)]
        came_from = {start_grid: None}
        cost_so_far = {start_grid: 0}

        while frontier:
            current_cost, current = heapq.heappop(frontier)

            if current == goal_grid:
                break

            for neighbor in self.get_neighbors(current):
                new_cost = cost_so_far[current] + self.heuristic(current, neighbor)
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self.heuristic(neighbor, goal_grid)
                    heapq.heappush(frontier, (priority, neighbor))
                    came_from[neighbor] = current

        # Reconstruct path
        if goal_grid not in came_from:
            return None  # No path found

        path = []
        current = goal_grid
        while current != start_grid:
            path.append((current[0] * self.resolution, current[1] * self.resolution))
            current = came_from[current]
        path.append((start_grid[0] * self.resolution, start_grid[1] * self.resolution))
        path.reverse()

        return path


class ManipulationPlanner:
    """Plan manipulation actions for the robot."""

    def __init__(self):
        self.reach_distance = 1.0  # Maximum reach distance for manipulation

    def plan_grasp(self, object_pos: Tuple[float, float, float],
                   robot_pos: Tuple[float, float, float]) -> Optional[Action]:
        """Plan a grasp action for the given object."""
        # Check if object is within reach
        distance = math.sqrt(sum((a - b)**2 for a, b in zip(object_pos, robot_pos)))

        if distance > self.reach_distance:
            # Need to navigate closer first
            return None

        # Plan the grasp action
        grasp_action = Action(
            action_type='grasp',
            parameters={
                'target_object': 'object',
                'position': object_pos,
                'approach_direction': (0, 0, 1)  # Approach from above
            },
            duration=2.0,
            preconditions=[
                {'type': 'at_location', 'location': robot_pos},
                {'type': 'gripper_state', 'state': 'open'},
                {'type': 'object_reachable', 'object': 'object'}
            ],
            effects=[
                {'type': 'object_grasped', 'object': 'object'},
                {'type': 'gripper_state', 'state': 'closed'}
            ]
        )

        return grasp_action

    def plan_place(self, target_pos: Tuple[float, float, float],
                   robot_pos: Tuple[float, float, float]) -> Optional[Action]:
        """Plan a place action at the target position."""
        # Check if target is within reach
        distance = math.sqrt(sum((a - b)**2 for a, b in zip(target_pos, robot_pos)))

        if distance > self.reach_distance:
            return None

        # Plan the place action
        place_action = Action(
            action_type='place',
            parameters={
                'target_position': target_pos,
                'release_direction': (0, 0, -1)  # Release downward
            },
            duration=1.5,
            preconditions=[
                {'type': 'at_location', 'location': robot_pos},
                {'type': 'object_grasped', 'object': 'object'}
            ],
            effects=[
                {'type': 'object_placed', 'object': 'object', 'at': target_pos},
                {'type': 'gripper_state', 'state': 'open'},
                {'type': 'object_grasped', 'object': 'object', 'value': False}
            ]
        )

        return place_action


class TaskPlanner:
    """High-level task planner that decomposes tasks into action sequences."""

    def __init__(self):
        self.manipulation_planner = ManipulationPlanner()

    def plan_task(self, task: Task, current_state: RobotState,
                  world_objects: List[Dict[str, Any]]) -> Optional[Plan]:
        """Plan a sequence of actions to achieve the given task."""
        actions = []

        if task.id == 'pick_and_place':
            # Find source and destination objects
            source_obj = next((obj for obj in world_objects if obj['type'] == 'source'), None)
            dest_obj = next((obj for obj in world_objects if obj['type'] == 'destination'), None)

            if not source_obj or not dest_obj:
                return None

            # Plan navigation to source object
            nav_to_source = Action(
                action_type='navigate',
                parameters={
                    'target_position': source_obj['position'],
                    'approach_distance': 0.5
                },
                duration=5.0,
                preconditions=[
                    {'type': 'robot_operational', 'value': True}
                ],
                effects=[
                    {'type': 'at_location', 'location': source_obj['position']}
                ]
            )
            actions.append(nav_to_source)

            # Plan grasp action
            grasp_action = self.manipulation_planner.plan_grasp(
                source_obj['position'], current_state.position
            )
            if grasp_action:
                actions.append(grasp_action)

            # Plan navigation to destination
            nav_to_dest = Action(
                action_type='navigate',
                parameters={
                    'target_position': dest_obj['position'],
                    'approach_distance': 0.5
                },
                duration=5.0,
                preconditions=[
                    {'type': 'object_grasped', 'object': 'object'}
                ],
                effects=[
                    {'type': 'at_location', 'location': dest_obj['position']}
                ]
            )
            actions.append(nav_to_dest)

            # Plan place action
            place_action = self.manipulation_planner.plan_place(
                dest_obj['position'], current_state.position
            )
            if place_action:
                actions.append(place_action)

        elif task.id == 'explore_area':
            # Plan a sequence of navigation actions to explore
            exploration_points = [
                (2.0, 0.0, 0.0),
                (2.0, 2.0, 0.0),
                (0.0, 2.0, 0.0),
                (0.0, 0.0, 0.0)
            ]

            for point in exploration_points:
                nav_action = Action(
                    action_type='navigate',
                    parameters={'target_position': point},
                    duration=3.0,
                    preconditions=[{'type': 'robot_operational', 'value': True}],
                    effects=[{'type': 'at_location', 'location': point}]
                )
                actions.append(nav_action)

        # Calculate total cost (simple estimation based on action count and duration)
        total_cost = sum(action.duration for action in actions) + len(actions) * 0.1

        return Plan(
            id=f"plan_{task.id}_{int(time.time())}",
            task_id=task.id,
            actions=actions,
            cost=total_cost,
            timestamp=time.time()
        )


class PlanExecutor:
    """Executes a plan and monitors execution."""

    def __init__(self):
        self.execution_log = []

    def execute_plan(self, plan: Plan, initial_state: RobotState) -> Dict[str, Any]:
        """Execute the plan and return execution results."""
        current_state = initial_state
        execution_log = []
        success = True
        error = None

        for i, action in enumerate(plan.actions):
            try:
                # Log the action execution
                action_log = {
                    'action_index': i,
                    'action': action,
                    'start_time': time.time(),
                    'status': 'executing'
                }
                execution_log.append(action_log)

                # Simulate action execution
                result = self.execute_action(action, current_state)

                # Update current state based on action effects
                current_state = self.update_state_from_action(current_state, action)

                # Update log with result
                action_log['status'] = 'completed'
                action_log['end_time'] = time.time()
                action_log['duration'] = action_log['end_time'] - action_log['start_time']
                action_log['result'] = result

                # Check if action succeeded
                if not result.get('success', True):
                    success = False
                    error = result.get('error', 'Unknown error')
                    break

            except Exception as e:
                success = False
                error = str(e)
                action_log['status'] = 'failed'
                action_log['error'] = str(e)
                action_log['end_time'] = time.time()
                break

        # Log execution result
        execution_result = {
            'plan_id': plan.id,
            'task_id': plan.task_id,
            'success': success,
            'error': error,
            'execution_log': execution_log,
            'final_state': current_state,
            'timestamp': time.time()
        }

        self.execution_log.append(execution_result)
        return execution_result

    def execute_action(self, action: Action, current_state: RobotState) -> Dict[str, Any]:
        """Simulate execution of an action."""
        # Simulate action execution with some randomness for realism
        success_probability = 0.95  # 95% success rate for simulation
        actual_duration = action.duration * np.random.uniform(0.8, 1.2)  # ±20% duration variation

        # Simulate execution time
        time.sleep(actual_duration * 0.01)  # Scale down for faster simulation

        # Determine success based on probability
        success = np.random.random() < success_probability

        return {
            'success': success,
            'actual_duration': actual_duration,
            'details': f"Executed {action.action_type} action with parameters {action.parameters}"
        }

    def update_state_from_action(self, current_state: RobotState, action: Action) -> RobotState:
        """Update robot state based on action effects."""
        new_state = RobotState(
            position=current_state.position,
            orientation=current_state.orientation,
            joints=current_state.joints,
            gripper_state=current_state.gripper_state,
            timestamp=time.time()
        )

        # Update state based on action type
        if action.action_type == 'navigate':
            target_pos = action.parameters.get('target_position', current_state.position)
            new_state.position = target_pos
        elif action.action_type == 'grasp':
            new_state.gripper_state = 'closed'
        elif action.action_type == 'place':
            new_state.gripper_state = 'open'

        return new_state


class ActionPlanningSystem:
    """Main system for action planning and execution."""

    def __init__(self):
        # Create a simple 10x10 occupancy grid (0 = free, 1 = occupied)
        world_map = np.zeros((10, 10))
        # Add some obstacles
        world_map[4:6, 4:6] = 1  # Central obstacle
        world_map[7:9, 1:3] = 1  # Corner obstacle

        self.path_planner = PathPlanner(world_map, resolution=0.5)
        self.task_planner = TaskPlanner()
        self.plan_executor = PlanExecutor()

    def plan_and_execute(self, task: Task, initial_state: RobotState,
                        world_objects: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Plan and execute a task."""
        # Plan the task
        plan = self.task_planner.plan_task(task, initial_state, world_objects)

        if not plan or not plan.actions:
            return {
                'success': False,
                'error': 'Could not generate a valid plan',
                'plan': None,
                'execution_result': None
            }

        # Execute the plan
        execution_result = self.plan_executor.execute_plan(plan, initial_state)

        return {
            'success': True,
            'plan': plan,
            'execution_result': execution_result
        }


def create_sample_world() -> Tuple[RobotState, List[Dict[str, Any]], List[Task]]:
    """Create a sample world for demonstration."""
    # Initial robot state
    robot_state = RobotState(
        position=(0.0, 0.0, 0.0),
        orientation=(0.0, 0.0, 0.0, 1.0),
        joints=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        gripper_state='open',
        timestamp=time.time()
    )

    # World objects
    world_objects = [
        {
            'name': 'red_box',
            'type': 'source',
            'position': (3.0, 1.0, 0.0),
            'color': 'red'
        },
        {
            'name': 'blue_container',
            'type': 'destination',
            'position': (1.0, 3.0, 0.0),
            'color': 'blue'
        }
    ]

    # Sample tasks
    tasks = [
        Task(
            id='pick_and_place',
            description='Pick up the red box and place it in the blue container',
            goal_conditions=[
                {'type': 'object_at_location', 'object': 'red_box', 'location': (1.0, 3.0, 0.0)}
            ],
            priority=1
        ),
        Task(
            id='explore_area',
            description='Explore the environment by visiting key locations',
            goal_conditions=[
                {'type': 'visited_locations', 'count': 4}
            ],
            priority=2
        )
    ]

    return robot_state, world_objects, tasks


def main():
    print("Action Planning and Execution Demo - Chapter 22")
    print("=" * 50)
    print("This demo illustrates action planning concepts:")
    print("- Task decomposition and planning")
    print("- Path planning and navigation")
    print("- Manipulation planning")
    print("- Execution monitoring and control")
    print("- Plan execution and replanning")
    print()

    # Initialize the action planning system
    planning_system = ActionPlanningSystem()

    # Create a sample world
    robot_state, world_objects, tasks = create_sample_world()

    print(f"Initial robot position: {robot_state.position}")
    print(f"World objects: {[obj['name'] for obj in world_objects]}")
    print()

    # Process each task
    for i, task in enumerate(tasks):
        print(f"Processing Task {i+1}: {task.description}")

        # Plan and execute the task
        result = planning_system.plan_and_execute(task, robot_state, world_objects)

        if result['success']:
            print(f"  Plan generated with {len(result['plan'].actions)} actions")
            print(f"  Estimated cost: {result['plan'].cost:.2f}")

            execution = result['execution_result']
            print(f"  Execution success: {execution['success']}")

            if execution['success']:
                print(f"  Actions completed: {len([log for log in execution['execution_log'] if log['status'] == 'completed'])}")
                print(f"  Final robot position: {execution['final_state'].position}")
            else:
                print(f"  Execution failed: {execution.get('error', 'Unknown error')}")
        else:
            print(f"  Failed to plan task: {result.get('error', 'Unknown error')}")

        print()

    # Demonstrate path planning specifically
    print("Path Planning Demonstration:")
    print("-" * 30)
    start_pos = (0.0, 0.0)
    goal_pos = (4.0, 4.0)

    path = planning_system.path_planner.plan_path(start_pos, goal_pos)
    if path:
        print(f"Path from {start_pos} to {goal_pos}:")
        print(f"  Waypoints: {len(path)}")
        print(f"  First few waypoints: {path[:3]}{'...' if len(path) > 3 else ''}")
    else:
        print(f"No path found from {start_pos} to {goal_pos}")
    print()

    # Show system components
    print("Action Planning System Components:")
    print("- Path Planner: Finds optimal paths in the environment")
    print("- Manipulation Planner: Plans specific manipulation actions")
    print("- Task Planner: Decomposes high-level tasks into action sequences")
    print("- Plan Executor: Executes plans and monitors progress")
    print("- State Monitor: Tracks robot state during execution")
    print()

    print("Key Action Planning Concepts Demonstrated:")
    print("1. Task Decomposition: Breaking complex tasks into simple actions")
    print("2. Path Planning: Finding collision-free routes to goals")
    print("3. Manipulation Planning: Planning precise manipulation movements")
    print("4. Execution Monitoring: Tracking plan execution and detecting failures")
    print("5. Plan Adaptation: Adjusting plans based on execution results")

    print(f"\nDemo completed. Processed {len(tasks)} tasks.")


if __name__ == "__main__":
    main()