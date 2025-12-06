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
Unity Robotics Example for Chapter 9: Unity for Robotics Simulation.

This example demonstrates Unity robotics simulation concepts including:
- Robot simulation in Unity environment
- Physics-based interactions
- Sensor simulation (camera, lidar, IMU)
- ROS integration patterns
"""

import time
import math
import json
from dataclasses import dataclass, asdict
from typing import List, Dict, Any, Optional
import random


@dataclass
class UnityTransform:
    """Represents position and rotation in Unity coordinate system."""
    position_x: float = 0.0
    position_y: float = 0.0
    position_z: float = 0.0
    rotation_x: float = 0.0
    rotation_y: float = 0.0
    rotation_z: float = 0.0
    rotation_w: float = 1.0  # Quaternion w component


@dataclass
class UnityRobotState:
    """Represents the state of a robot in Unity simulation."""
    transform: UnityTransform
    linear_velocity: float = 0.0
    angular_velocity: float = 0.0
    timestamp: float = 0.0


@dataclass
class UnityCameraData:
    """Represents camera sensor data from Unity."""
    width: int
    height: int
    fov: float  # Field of view in degrees
    image_data: List[int]  # RGB values
    timestamp: float


@dataclass
class UnityLidarData:
    """Represents LIDAR sensor data from Unity."""
    ranges: List[float]
    angle_min: float
    angle_max: float
    angle_increment: float
    timestamp: float


class UnityEnvironment:
    """Simulates a Unity environment with physics and objects."""

    def __init__(self, scene_name: str = "RoboticsLab"):
        self.scene_name = scene_name
        self.objects = {
            "ground_plane": {"type": "plane", "position": [0, 0, 0], "size": [20, 20]},
            "wall_1": {"type": "box", "position": [10, 1, 0], "size": [0.5, 2, 10]},
            "wall_2": {"type": "box", "position": [-10, 1, 0], "size": [0.5, 2, 10]},
            "wall_3": {"type": "box", "position": [0, 1, 10], "size": [20, 2, 0.5]},
            "wall_4": {"type": "box", "position": [0, 1, -10], "size": [20, 2, 0.5]},
            "obstacle_1": {"type": "cylinder", "position": [3, 0.5, 2], "size": [0.8, 1]},
            "obstacle_2": {"type": "sphere", "position": [-4, 0.5, -3], "size": [0.7]},
        }
        self.gravity = -9.81  # m/s^2
        self.physics_scale = 1.0  # Unity to real-world scale factor

    def get_object_positions(self) -> Dict[str, List[float]]:
        """Get positions of all objects in the scene."""
        positions = {}
        for name, obj in self.objects.items():
            positions[name] = obj["position"]
        return positions

    def check_collision(self, position: List[float], radius: float = 0.5) -> bool:
        """Check for collisions with environment objects."""
        px, py, pz = position

        # Check collision with walls
        if abs(px) > 9.5 - radius or abs(pz) > 9.5 - radius:
            return True

        # Check collision with obstacles
        for name, obj in self.objects.items():
            if name.startswith("obstacle_"):
                ox, oy, oz = obj["position"]
                distance = math.sqrt((px - ox)**2 + (pz - oz)**2)
                if distance < radius + obj["size"][0]/2:
                    return True

        return False


class UnityRobot:
    """Represents a robot in the Unity simulation environment."""

    def __init__(self, env: UnityEnvironment, initial_position: List[float] = [0, 0, 0]):
        self.env = env
        self.state = UnityRobotState(
            transform=UnityTransform(
                position_x=initial_position[0],
                position_y=initial_position[1],
                position_z=initial_position[2]
            ),
            linear_velocity=0.0,
            angular_velocity=0.0,
            timestamp=time.time()
        )
        self.max_linear_vel = 2.0  # m/s
        self.max_angular_vel = 1.5  # rad/s
        self.camera_enabled = True
        self.lidar_enabled = True
        self.imu_enabled = True

    def set_velocity(self, linear: float, angular: float):
        """Set the robot's linear and angular velocities."""
        self.state.linear_velocity = max(-self.max_linear_vel, min(linear, self.max_linear_vel))
        self.state.angular_velocity = max(-self.max_angular_vel, min(angular, self.max_angular_vel))

    def update(self, dt: float):
        """Update the robot's state based on physics simulation."""
        old_pos = [self.state.transform.position_x, self.state.transform.position_y, self.state.transform.position_z]

        # Update position based on velocities (simplified differential drive model)
        current_angle = self.state.transform.rotation_y
        dx = self.state.linear_velocity * math.cos(current_angle) * dt
        dz = self.state.linear_velocity * math.sin(current_angle) * dt

        new_x = self.state.transform.position_x + dx
        new_z = self.state.transform.position_z + dz
        new_angle = self.state.transform.rotation_y + self.state.angular_velocity * dt

        # Normalize angle to [-pi, pi]
        new_angle = ((new_angle + math.pi) % (2 * math.pi)) - math.pi

        # Check for collisions before updating
        if not self.env.check_collision([new_x, self.state.transform.position_y, new_z]):
            self.state.transform.position_x = new_x
            self.state.transform.position_z = new_z
            self.state.transform.rotation_y = new_angle
        else:
            # Stop the robot if collision detected
            self.state.linear_velocity = 0.0
            self.state.angular_velocity = 0.0

        self.state.timestamp = time.time()

    def get_camera_data(self) -> Optional[UnityCameraData]:
        """Simulate and return camera sensor data."""
        if not self.camera_enabled:
            return None

        # Simulate a 640x480 RGB camera
        width, height = 640, 480
        fov = 60  # degrees

        # Generate simulated image data (simplified - just noise for demonstration)
        image_data = []
        for _ in range(width * height):
            # Random RGB values to simulate image data
            r = random.randint(0, 255)
            g = random.randint(0, 255)
            b = random.randint(0, 255)
            image_data.extend([r, g, b])

        return UnityCameraData(
            width=width,
            height=height,
            fov=fov,
            image_data=image_data,
            timestamp=time.time()
        )

    def get_lidar_data(self) -> Optional[UnityLidarData]:
        """Simulate and return LIDAR sensor data."""
        if not self.lidar_enabled:
            return None

        # Simulate 360-degree LIDAR with 1-degree increments
        angle_min = -math.pi
        angle_max = math.pi
        angle_increment = math.pi / 180  # 1 degree in radians
        num_readings = int((angle_max - angle_min) / angle_increment)

        ranges = []
        robot_x = self.state.transform.position_x
        robot_z = self.state.transform.position_z  # In Unity, Z is forward, Y is up

        for i in range(num_readings):
            angle = self.state.transform.rotation_y + angle_min + i * angle_increment
            max_range = 10.0  # Maximum sensor range in meters

            # Calculate the direction vector
            dir_x = math.cos(angle)
            dir_z = math.sin(angle)

            # Check for obstacles along this ray
            measured_range = max_range
            for dist in [i * 0.1 for i in range(1, int(max_range / 0.1) + 1)]:
                check_x = robot_x + dist * dir_x
                check_z = robot_z + dist * dir_z

                # Check collision with environment
                if self.env.check_collision([check_x, 0, check_z], radius=0.1):
                    measured_range = dist
                    break

            ranges.append(min(measured_range, max_range))

        return UnityLidarData(
            ranges=ranges,
            angle_min=angle_min,
            angle_max=angle_max,
            angle_increment=angle_increment,
            timestamp=time.time()
        )

    def get_imu_data(self) -> Dict[str, float]:
        """Simulate and return IMU sensor data."""
        if not self.imu_enabled:
            return {}

        # Simulate IMU data with some noise
        return {
            "acceleration_x": random.gauss(0, 0.1),
            "acceleration_y": random.gauss(0, 0.1) - 9.81,  # Gravity component
            "acceleration_z": random.gauss(0, 0.1),
            "angular_velocity_x": random.gauss(0, 0.01),
            "angular_velocity_y": self.state.angular_velocity + random.gauss(0, 0.01),
            "angular_velocity_z": random.gauss(0, 0.01),
            "orientation_x": self.state.transform.rotation_x,
            "orientation_y": self.state.transform.rotation_y,
            "orientation_z": self.state.transform.rotation_z,
            "orientation_w": self.state.transform.rotation_w,
            "timestamp": time.time()
        }


class UnityRobotController:
    """A controller that manages robot behavior in Unity simulation."""

    def __init__(self, robot: UnityRobot):
        self.robot = robot
        self.targets = [
            {"x": 5.0, "z": 5.0},
            {"x": -5.0, "z": 5.0},
            {"x": -5.0, "z": -5.0},
            {"x": 5.0, "z": -5.0}
        ]
        self.current_target_index = 0
        self.arrived_at_target = False
        self.navigation_active = True

    def compute_velocity(self) -> tuple[float, float]:
        """Compute linear and angular velocities to navigate to target."""
        if not self.navigation_active:
            return 0.0, 0.0

        # Get current robot position
        current_x = self.robot.state.transform.position_x
        current_z = self.robot.state.transform.position_z  # Z is forward in Unity

        # Get current target
        target = self.targets[self.current_target_index]
        target_x = target["x"]
        target_z = target["z"]

        # Calculate distance to target
        distance = math.sqrt((current_x - target_x)**2 + (current_z - target_z)**2)

        # Check if we've reached the target
        if distance < 0.5:  # 0.5m tolerance
            self.arrived_at_target = True
            self.current_target_index = (self.current_target_index + 1) % len(self.targets)
            self.arrived_at_target = False
            print(f"✅ Reached target {self.current_target_index}, heading to next...")

        # Calculate desired heading
        desired_angle = math.atan2(target_z - current_z, target_x - current_x)

        # Get current orientation
        current_angle = self.robot.state.transform.rotation_y

        # Calculate angle difference
        angle_diff = desired_angle - current_angle
        # Normalize angle difference to [-pi, pi]
        angle_diff = ((angle_diff + math.pi) % (2 * math.pi)) - math.pi

        # Simple proportional controller
        linear_vel = min(1.0, distance)  # Slower as we get closer
        angular_vel = 2.0 * angle_diff  # Proportional control for orientation

        # Limit angular velocity
        angular_vel = max(-1.0, min(angular_vel, 1.0))

        return linear_vel, angular_vel

    def get_sensor_data(self) -> Dict[str, Any]:
        """Get all sensor data from the robot."""
        camera_data = self.robot.get_camera_data()
        lidar_data = self.robot.get_lidar_data()
        imu_data = self.robot.get_imu_data()

        sensor_data = {
            "timestamp": time.time(),
            "camera": {
                "enabled": camera_data is not None,
                "width": camera_data.width if camera_data else 0,
                "height": camera_data.height if camera_data else 0,
                "fov": camera_data.fov if camera_data else 0
            } if camera_data else None,
            "lidar": {
                "enabled": lidar_data is not None,
                "num_readings": len(lidar_data.ranges) if lidar_data else 0,
                "min_range": min(lidar_data.ranges) if lidar_data else 0,
                "max_range": max(lidar_data.ranges) if lidar_data else 0
            } if lidar_data else None,
            "imu": imu_data if imu_data else {}
        }

        return sensor_data


def main():
    print("Unity Robotics Simulation Example - Chapter 9")
    print("=" * 55)

    # Create a Unity environment
    env = UnityEnvironment(scene_name="RoboticsLab")
    print(f"Created Unity environment: {env.scene_name}")
    print(f"Environment objects: {len(env.objects)}")

    # Create a robot in the environment
    robot = UnityRobot(env, initial_position=[0, 0, 0])
    print(f"Created robot at initial position: ({robot.state.transform.position_x}, {robot.state.transform.position_z})")

    # Create a controller for the robot
    controller = UnityRobotController(robot)
    print(f"Created controller with {len(controller.targets)} navigation targets")

    print("\nStarting Unity simulation...")
    print("Robot will navigate between targets while collecting sensor data.")
    print("Sensor data will be displayed periodically.\n")

    # Simulation parameters
    dt = 0.1  # Time step (100ms)
    max_steps = 1000  # Maximum simulation steps (100 seconds)

    try:
        for step in range(max_steps):
            # Get control commands
            linear_vel, angular_vel = controller.compute_velocity()

            # Set robot velocity
            robot.set_velocity(linear_vel, angular_vel)

            # Update robot state in simulation
            robot.update(dt)

            # Print status every 50 steps (5 seconds)
            if step % 50 == 0:
                state = robot.state
                target = controller.targets[controller.current_target_index]
                distance_to_target = math.sqrt(
                    (state.transform.position_x - target["x"])**2 +
                    (state.transform.position_z - target["z"])**2
                )
                print(f"Step {step:4d}: Pos=({state.transform.position_x:6.2f}, {state.transform.position_z:6.2f}), "
                      f"Angle={state.transform.rotation_y:6.3f}, DistToTarget={distance_to_target:6.2f}, "
                      f"Vel=({state.linear_velocity:5.2f}, {state.angular_velocity:5.2f})")

            # Print sensor data every 100 steps (10 seconds)
            if step % 100 == 0:
                sensor_data = controller.get_sensor_data()
                if sensor_data["lidar"]:
                    print(f"         LIDAR: {sensor_data['lidar']['num_readings']} readings, "
                          f"Min={sensor_data['lidar']['min_range']:.2f}m, "
                          f"Max={sensor_data['lidar']['max_range']:.2f}m")
                if sensor_data["camera"]:
                    print(f"         Camera: {sensor_data['camera']['width']}x{sensor_data['camera']['height']}, "
                          f"FOV={sensor_data['camera']['fov']}°")

            time.sleep(dt)  # Simulate real-time execution

    except KeyboardInterrupt:
        print("\nSimulation stopped by user.")

    # Final status
    final_state = robot.state
    print(f"\nFinal position: ({final_state.transform.position_x:.2f}, {final_state.transform.position_z:.2f})")
    print(f"Simulation completed after {max_steps} steps.")


if __name__ == "__main__":
    main()