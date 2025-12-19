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
Gazebo Robot Example for Chapter 8: Gazebo Simulation Environment.

This example demonstrates Gazebo simulation concepts including:
- Robot model integration in Gazebo
- Physics simulation and sensor modeling
- Control interfaces for simulated robots
"""

import time
import math
from dataclasses import dataclass
from typing import List, Tuple
import random


@dataclass
class RobotState:
    """Represents the state of a robot in the simulation."""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0  # orientation in radians
    linear_velocity: float = 0.0
    angular_velocity: float = 0.0
    timestamp: float = 0.0


@dataclass
class LaserScan:
    """Represents laser scan data from a simulated sensor."""
    ranges: List[float]
    angle_min: float
    angle_max: float
    angle_increment: float
    time_increment: float
    scan_time: float
    range_min: float
    range_max: float
    timestamp: float


class GazeboWorld:
    """Simulates a Gazebo world with physics and environment features."""

    def __init__(self, width: float = 10.0, height: float = 10.0):
        self.width = width
        self.height = height
        self.obstacles = [
            (2.0, 2.0, 0.5),    # x, y, radius
            (7.0, 3.0, 0.7),
            (5.0, 7.0, 0.4),
            (8.5, 8.5, 0.6)
        ]
        self.gravity = -9.81  # m/s^2

    def check_collision(self, x: float, y: float, radius: float = 0.3) -> bool:
        """Check if a robot at (x,y) collides with any obstacles."""
        for obs_x, obs_y, obs_radius in self.obstacles:
            distance = math.sqrt((x - obs_x)**2 + (y - obs_y)**2)
            if distance < radius + obs_radius:
                return True
        # Check boundary collision
        if x < radius or x > self.width - radius or y < radius or y > self.height - radius:
            return True
        return False

    def get_laser_scan(self, x: float, y: float, theta: float) -> LaserScan:
        """Simulate a laser scan from the robot's position."""
        angle_min = -math.pi / 2  # -90 degrees
        angle_max = math.pi / 2   # 90 degrees
        angle_increment = math.pi / 180  # 1 degree increments
        num_readings = int((angle_max - angle_min) / angle_increment) + 1

        ranges = []
        for i in range(num_readings):
            angle = theta + angle_min + i * angle_increment
            max_range = 5.0  # Maximum sensor range
            measured_range = max_range

            # Check for obstacle collisions along this ray
            for distance in [i * 0.1 for i in range(1, int(max_range / 0.1) + 1)]:
                check_x = x + distance * math.cos(angle)
                check_y = y + distance * math.sin(angle)

                for obs_x, obs_y, obs_radius in self.obstacles:
                    obs_distance = math.sqrt((check_x - obs_x)**2 + (check_y - obs_y)**2)
                    if obs_distance < obs_radius:
                        measured_range = distance
                        break

                # Check boundary
                if (check_x < 0 or check_x > self.width or
                    check_y < 0 or check_y > self.height):
                    measured_range = distance
                    break

                if distance >= measured_range:
                    break

            ranges.append(min(measured_range, max_range))

        return LaserScan(
            ranges=ranges,
            angle_min=angle_min,
            angle_max=angle_max,
            angle_increment=angle_increment,
            time_increment=0.0,
            scan_time=0.0,
            range_min=0.1,
            range_max=5.0,
            timestamp=time.time()
        )


class GazeboRobot:
    """Simulates a robot in the Gazebo environment."""

    def __init__(self, world: GazeboWorld, initial_x: float = 1.0, initial_y: float = 1.0):
        self.world = world
        self.state = RobotState(
            x=initial_x,
            y=initial_y,
            theta=0.0,
            linear_velocity=0.0,
            angular_velocity=0.0,
            timestamp=time.time()
        )
        self.max_linear_vel = 1.0  # m/s
        self.max_angular_vel = 1.0  # rad/s
        self.wheel_radius = 0.1  # meters
        self.wheel_base = 0.3  # meters (distance between wheels)

    def set_velocity(self, linear: float, angular: float):
        """Set the robot's linear and angular velocities."""
        self.state.linear_velocity = max(-self.max_linear_vel, min(linear, self.max_linear_vel))
        self.state.angular_velocity = max(-self.max_angular_vel, min(angular, self.max_angular_vel))

    def update(self, dt: float):
        """Update the robot's state based on physics simulation."""
        # Store old position for collision detection
        old_x, old_y = self.state.x, self.state.y

        # Update position based on velocities
        self.state.x += self.state.linear_velocity * math.cos(self.state.theta) * dt
        self.state.y += self.state.linear_velocity * math.sin(self.state.theta) * dt
        self.state.theta += self.state.angular_velocity * dt

        # Normalize angle to [-pi, pi]
        self.state.theta = ((self.state.theta + math.pi) % (2 * math.pi)) - math.pi

        # Check for collisions and revert if needed
        if self.world.check_collision(self.state.x, self.state.y):
            self.state.x = old_x
            self.state.y = old_y
            # Stop the robot when collision occurs
            self.state.linear_velocity = 0.0
            self.state.angular_velocity = 0.0

        self.state.timestamp = time.time()

    def get_laser_scan(self) -> LaserScan:
        """Get simulated laser scan data from the robot's position."""
        return self.world.get_laser_scan(self.state.x, self.state.y, self.state.theta)

    def get_odometry(self) -> RobotState:
        """Get the current odometry (position and velocities)."""
        return self.state


class GazeboController:
    """A simple controller that navigates the robot in the Gazebo world."""

    def __init__(self, robot: GazeboRobot):
        self.robot = robot
        self.target_x = 8.0
        self.target_y = 8.0
        self.arrived = False

    def compute_velocity(self) -> Tuple[float, float]:
        """Compute linear and angular velocities to reach the target."""
        if self.arrived:
            return 0.0, 0.0

        # Get current robot state
        current_x = self.robot.state.x
        current_y = self.robot.state.y
        current_theta = self.robot.state.theta

        # Calculate distance and angle to target
        dx = self.target_x - current_x
        dy = self.target_y - current_y
        distance_to_target = math.sqrt(dx**2 + dy**2)

        # Check if we've arrived
        if distance_to_target < 0.3:
            self.arrived = True
            return 0.0, 0.0

        # Calculate desired heading
        desired_theta = math.atan2(dy, dx)

        # Calculate angle difference
        angle_diff = desired_theta - current_theta
        # Normalize angle difference to [-pi, pi]
        angle_diff = ((angle_diff + math.pi) % (2 * math.pi)) - math.pi

        # Simple proportional controller
        linear_vel = min(0.5, distance_to_target)  # Slower as we get closer
        angular_vel = 2.0 * angle_diff  # Proportional control for orientation

        # Limit angular velocity
        angular_vel = max(-0.5, min(angular_vel, 0.5))

        return linear_vel, angular_vel


def main():
    print("Gazebo Simulation Example - Chapter 8")
    print("=" * 50)

    # Create a Gazebo world
    world = GazeboWorld(width=10.0, height=10.0)
    print(f"Created Gazebo world: {world.width}m x {world.height}m")
    print(f"Obstacles in world: {len(world.obstacles)}")

    # Create a robot in the world
    robot = GazeboRobot(world, initial_x=1.0, initial_y=1.0)
    print(f"Created robot at initial position: ({robot.state.x:.1f}, {robot.state.y:.1f})")

    # Create a controller for the robot
    controller = GazeboController(robot)
    print(f"Created controller with target: ({controller.target_x:.1f}, {controller.target_y:.1f})")

    print("\nStarting Gazebo simulation...")
    print("Robot will navigate to the target while avoiding obstacles.")
    print("Laser scan data will be displayed periodically.\n")

    # Simulation parameters
    dt = 0.1  # Time step (100ms)
    max_steps = 500  # Maximum simulation steps (50 seconds)

    try:
        for step in range(max_steps):
            # Get control commands
            linear_vel, angular_vel = controller.compute_velocity()

            # Set robot velocity
            robot.set_velocity(linear_vel, angular_vel)

            # Update robot state in simulation
            robot.update(dt)

            # Print status every 20 steps (2 seconds)
            if step % 20 == 0:
                state = robot.get_odometry()
                distance_to_target = math.sqrt(
                    (state.x - controller.target_x)**2 +
                    (state.y - controller.target_y)**2
                )
                print(f"Step {step:3d}: Pos=({state.x:5.2f}, {state.y:5.2f}), "
                      f"Theta={state.theta:6.3f}, DistToTarget={distance_to_target:5.2f}, "
                      f"Vel=({state.linear_velocity:5.2f}, {state.angular_velocity:5.2f})")

            # Print laser scan data every 50 steps (5 seconds)
            if step % 50 == 0:
                scan = robot.get_laser_scan()
                min_range = min(scan.ranges) if scan.ranges else float('inf')
                print(f"         Laser scan: Min range = {min_range:.2f}m, "
                      f"Readings = {len(scan.ranges)}")

            # Check if arrived at target
            if controller.arrived:
                print(f"\n✅ Robot reached the target at step {step}!")
                break

            time.sleep(dt)  # Simulate real-time execution

    except KeyboardInterrupt:
        print("\nSimulation stopped by user.")

    # Final status
    final_state = robot.get_odometry()
    final_distance = math.sqrt(
        (final_state.x - controller.target_x)**2 +
        (final_state.y - controller.target_y)**2
    )
    print(f"\nFinal position: ({final_state.x:.2f}, {final_state.y:.2f})")
    print(f"Final distance to target: {final_distance:.2f}m")
    print(f"Robot arrived at target: {controller.arrived}")


if __name__ == "__main__":
    main()