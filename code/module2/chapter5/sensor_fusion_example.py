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
Sensor Simulation and Data Fusion Example for Chapter 11: Sensor Simulation and Data Fusion.

This example demonstrates sensor simulation and data fusion concepts including:
- Multiple sensor types (IMU, GPS, LIDAR, Camera)
- Sensor noise modeling
- Kalman filtering for data fusion
- State estimation from multiple sensor sources
"""

import time
import math
import random
from dataclasses import dataclass
from typing import List, Tuple, Optional
import numpy as np


@dataclass
class RobotState:
    """True state of the robot for simulation."""
    x: float = 0.0
    y: float = 0.0
    vx: float = 0.0  # velocity in x
    vy: float = 0.0  # velocity in y
    heading: float = 0.0  # orientation in radians
    timestamp: float = 0.0


@dataclass
class IMUData:
    """IMU sensor data with acceleration and angular velocity."""
    linear_acceleration_x: float
    linear_acceleration_y: float
    linear_acceleration_z: float
    angular_velocity_x: float
    angular_velocity_y: float
    angular_velocity_z: float
    timestamp: float


@dataclass
class GPSData:
    """GPS sensor data with position and velocity."""
    latitude: float
    longitude: float
    altitude: float
    velocity_x: float
    velocity_y: float
    timestamp: float


@dataclass
class LIDARData:
    """LIDAR sensor data with distance measurements."""
    ranges: List[float]
    angle_min: float
    angle_max: float
    angle_increment: float
    timestamp: float


@dataclass
class CameraData:
    """Camera sensor data with image features."""
    features_x: List[float]  # X coordinates of detected features
    features_y: List[float]  # Y coordinates of detected features
    timestamp: float


class SensorSimulator:
    """Simulates various sensors with realistic noise models."""

    def __init__(self, initial_state: RobotState):
        self.true_state = initial_state
        self.time = initial_state.timestamp

        # Sensor noise parameters
        self.imu_acc_noise = 0.01  # m/s^2
        self.imu_gyro_noise = 0.001  # rad/s
        self.gps_pos_noise = 0.5  # meters
        self.gps_vel_noise = 0.1  # m/s
        self.lidar_noise = 0.02  # meters

    def update_true_state(self, dt: float, control_input: Tuple[float, float, float]):
        """Update the true robot state based on control input."""
        ax, ay, angular_vel = control_input

        # Update heading
        self.true_state.heading += angular_vel * dt
        self.true_state.heading = ((self.true_state.heading + math.pi) % (2 * math.pi)) - math.pi

        # Update velocities (with some damping)
        self.true_state.vx += ax * dt
        self.true_state.vy += ay * dt
        self.true_state.vx *= 0.999  # Small damping
        self.true_state.vy *= 0.999  # Small damping

        # Update positions
        self.true_state.x += self.true_state.vx * dt
        self.true_state.y += self.true_state.vy * dt

        self.time += dt
        self.true_state.timestamp = self.time

    def get_imu_data(self) -> IMUData:
        """Simulate IMU readings with noise."""
        # Calculate true angular velocity based on heading change
        # For this simulation, we'll use a simple model
        angular_vel_z = 0.1  # Simplified - in real system this would come from control input

        return IMUData(
            linear_acceleration_x=self.true_state.vx * 0.1 + random.gauss(0, self.imu_acc_noise),
            linear_acceleration_y=self.true_state.vy * 0.1 + random.gauss(0, self.imu_acc_noise),
            linear_acceleration_z=-9.81 + random.gauss(0, self.imu_acc_noise),  # Gravity
            angular_velocity_x=random.gauss(0, self.imu_gyro_noise),
            angular_velocity_y=random.gauss(0, self.imu_gyro_noise),
            angular_velocity_z=angular_vel_z + random.gauss(0, self.imu_gyro_noise),
            timestamp=self.time
        )

    def get_gps_data(self) -> GPSData:
        """Simulate GPS readings with noise."""
        # Convert x,y to latitude,longitude (simplified)
        # In a real system, this would involve proper coordinate transformations
        lat = 40.0 + self.true_state.x * 0.00001  # Rough conversion
        lon = -74.0 + self.true_state.y * 0.00001  # Rough conversion

        return GPSData(
            latitude=lat + random.gauss(0, self.gps_pos_noise * 0.00001),
            longitude=lon + random.gauss(0, self.gps_pos_noise * 0.00001),
            altitude=100.0 + random.gauss(0, self.gps_pos_noise),  # Fixed altitude with noise
            velocity_x=self.true_state.vx + random.gauss(0, self.gps_vel_noise),
            velocity_y=self.true_state.vy + random.gauss(0, self.gps_vel_noise),
            timestamp=self.time
        )

    def get_lidar_data(self) -> LIDARData:
        """Simulate LIDAR readings with noise."""
        # Simulate 360-degree LIDAR with 1-degree increments
        angle_min = -math.pi
        angle_max = math.pi
        angle_increment = math.pi / 180  # 1 degree
        num_readings = int((angle_max - angle_min) / angle_increment)

        ranges = []
        robot_x, robot_y = self.true_state.x, self.true_state.y

        # In a real simulation, this would check for obstacles in the environment
        # For this example, we'll simulate some simple readings
        for i in range(num_readings):
            # Create some "obstacles" at fixed positions for realistic readings
            angle = self.true_state.heading + angle_min + i * angle_increment
            min_range = 10.0  # Max range if no obstacle

            # Simulate a few obstacles
            obstacles = [
                (5.0, 0.0),   # x, y position of obstacle
                (0.0, 5.0),
                (-3.0, -3.0)
            ]

            for obs_x, obs_y in obstacles:
                dx = obs_x - robot_x
                dy = obs_y - robot_y
                distance_to_obs = math.sqrt(dx*dx + dy*dy)

                # Calculate angle to obstacle
                angle_to_obs = math.atan2(dy, dx)
                angle_diff = abs(angle - angle_to_obs)
                angle_diff = min(angle_diff, 2*math.pi - angle_diff)  # Normalize angle difference

                # If this ray is close to the obstacle, update range
                if angle_diff < 0.1:  # About 5.7 degrees
                    min_range = min(min_range, distance_to_obs)

            # Add noise to the range
            noisy_range = min_range + random.gauss(0, self.lidar_noise)
            ranges.append(max(0.1, noisy_range))  # Ensure positive range

        return LIDARData(
            ranges=ranges,
            angle_min=angle_min,
            angle_max=angle_max,
            angle_increment=angle_increment,
            timestamp=self.time
        )

    def get_camera_data(self) -> CameraData:
        """Simulate camera feature detection."""
        # Simulate detection of visual features in the environment
        # For this example, we'll create features based on "landmarks" in the environment
        landmarks = [
            (5.0, 0.0),   # x, y position of landmark
            (0.0, 5.0),
            (-3.0, -3.0),
            (4.0, 4.0)
        ]

        features_x = []
        features_y = []

        for lx, ly in landmarks:
            # Calculate relative position of landmark to robot
            rel_x = lx - self.true_state.x
            rel_y = ly - self.true_state.y

            # Rotate based on robot's heading to get camera-relative coordinates
            cos_h = math.cos(-self.true_state.heading)
            sin_h = math.sin(-self.true_state.heading)
            cam_x = rel_x * cos_h - rel_y * sin_h
            cam_y = rel_x * sin_h + rel_y * cos_h

            # Only add features that are in front of the robot and within range
            if cam_x > 0 and math.sqrt(cam_x*cam_x + cam_y*cam_y) < 10.0:
                # Add some noise to simulate detection uncertainty
                features_x.append(cam_x + random.gauss(0, 0.1))
                features_y.append(cam_y + random.gauss(0, 0.1))

        return CameraData(
            features_x=features_x,
            features_y=features_y,
            timestamp=self.time
        )


class KalmanFilter:
    """Simple Kalman Filter for fusing position estimates."""

    def __init__(self):
        # State vector: [x, y, vx, vy]
        self.state = np.array([0.0, 0.0, 0.0, 0.0])

        # Covariance matrix (uncertainty in state estimate)
        self.covariance = np.eye(4) * 1000.0  # Start with high uncertainty

        # Process noise (how much we expect the model to be wrong)
        self.process_noise = np.eye(4)
        self.process_noise[0, 0] = 0.1  # x position process noise
        self.process_noise[1, 1] = 0.1  # y position process noise
        self.process_noise[2, 2] = 0.5  # x velocity process noise
        self.process_noise[3, 3] = 0.5  # y velocity process noise

        # Measurement noise for different sensors
        self.gps_noise = 0.5  # meters
        self.camera_noise = 0.2  # meters

    def predict(self, dt: float):
        """Predict the next state based on motion model."""
        # Simple motion model: x = x + vx*dt, y = y + vy*dt
        F = np.array([
            [1, 0, dt, 0],   # x = x + vx*dt
            [0, 1, 0, dt],   # y = y + vy*dt
            [0, 0, 1, 0],    # vx = vx (constant velocity model)
            [0, 0, 0, 1]     # vy = vy (constant velocity model)
        ])

        # Predict state
        self.state = F @ self.state

        # Predict covariance
        self.covariance = F @ self.covariance @ F.T + self.process_noise * dt

    def update_gps(self, gps_x: float, gps_y: float):
        """Update filter with GPS measurement."""
        # Measurement matrix (we can measure x and y directly)
        H = np.array([
            [1, 0, 0, 0],  # Measure x
            [0, 1, 0, 0]   # Measure y
        ])

        # Measurement
        z = np.array([gps_x, gps_y])

        # Expected measurement
        h = H @ self.state

        # Innovation (difference between measurement and prediction)
        y = z - h

        # Innovation covariance
        S = H @ self.covariance @ H.T + np.eye(2) * (self.gps_noise**2)

        # Kalman gain
        K = self.covariance @ H.T @ np.linalg.inv(S)

        # Update state
        self.state = self.state + K @ y

        # Update covariance
        self.covariance = (np.eye(4) - K @ H) @ self.covariance

    def update_camera(self, camera_x: float, camera_y: float):
        """Update filter with camera-based position measurement."""
        # Measurement matrix (we can measure x and y directly)
        H = np.array([
            [1, 0, 0, 0],  # Measure x
            [0, 1, 0, 0]   # Measure y
        ])

        # Measurement (relative to robot's position)
        z = np.array([camera_x, camera_y])

        # Expected measurement
        h = H @ self.state

        # Innovation (difference between measurement and prediction)
        y = z - h

        # Innovation covariance
        S = H @ self.covariance @ H.T + np.eye(2) * (self.camera_noise**2)

        # Kalman gain
        K = self.covariance @ H.T @ np.linalg.inv(S)

        # Update state
        self.state = self.state + K @ y

        # Update covariance
        self.covariance = (np.eye(4) - K @ H) @ self.covariance

    def get_state(self) -> Tuple[float, float, float, float]:
        """Get the current estimated state [x, y, vx, vy]."""
        return tuple(self.state)


class SensorFusionSystem:
    """Main system that manages sensor simulation and data fusion."""

    def __init__(self):
        self.simulator = SensorSimulator(RobotState(x=0, y=0, vx=0, vy=0, heading=0, timestamp=0))
        self.kalman_filter = KalmanFilter()
        self.estimated_positions = []
        self.true_positions = []
        self.gps_positions = []
        self.time_stamps = []

    def step(self, dt: float, control_input: Tuple[float, float, float]):
        """Run one step of the simulation and fusion."""
        # Update true state
        self.simulator.update_true_state(dt, control_input)
        true_state = self.simulator.true_state

        # Get sensor readings
        imu_data = self.simulator.get_imu_data()
        gps_data = self.simulator.get_gps_data()
        lidar_data = self.simulator.get_lidar_data()
        camera_data = self.simulator.get_camera_data()

        # Predict state using IMU data (simplified - in reality, IMU integration is more complex)
        self.kalman_filter.predict(dt)

        # Update with GPS data
        # Convert lat/lon to x,y (simplified)
        gps_x = (gps_data.latitude - 40.0) * 100000  # Rough conversion
        gps_y = (gps_data.longitude - (-74.0)) * 100000  # Rough conversion
        self.kalman_filter.update_gps(gps_x, gps_y)

        # If camera has features, update with camera data (simplified)
        if len(camera_data.features_x) > 0:
            # For this example, we'll use the first feature as a position reference
            # In reality, camera data would need to be processed to get position
            camera_x = camera_data.features_x[0] if len(camera_data.features_x) > 0 else 0
            camera_y = camera_data.features_y[0] if len(camera_data.features_y) > 0 else 0
            self.kalman_filter.update_camera(camera_x, camera_y)

        # Store data for visualization
        est_x, est_y, est_vx, est_vy = self.kalman_filter.get_state()

        self.true_positions.append((true_state.x, true_state.y))
        self.estimated_positions.append((est_x, est_y))
        self.gps_positions.append((gps_x, gps_y))
        self.time_stamps.append(true_state.timestamp)

        return {
            'true_state': true_state,
            'imu_data': imu_data,
            'gps_data': gps_data,
            'lidar_data': lidar_data,
            'camera_data': camera_data,
            'estimated_state': (est_x, est_y, est_vx, est_vy)
        }


def main():
    print("Sensor Simulation and Data Fusion Example - Chapter 11")
    print("=" * 60)

    # Create sensor fusion system
    fusion_system = SensorFusionSystem()
    print("Created sensor fusion system with Kalman filter")

    print("\nStarting sensor simulation and fusion...")
    print("System will simulate multiple sensors and fuse their data.")
    print("True position, GPS readings, and estimated position will be shown.\n")

    # Simulation parameters
    dt = 0.1  # 100ms time step
    max_steps = 500  # 50 seconds of simulation
    control_input = (0.5, 0.2, 0.1)  # ax, ay, angular_vel

    try:
        for step in range(max_steps):
            # Run one step of simulation and fusion
            data = fusion_system.step(dt, control_input)

            # Print status every 50 steps (5 seconds)
            if step % 50 == 0:
                true_state = data['true_state']
                est_x, est_y, est_vx, est_vy = data['estimated_state']

                print(f"Step {step:3d}: True=({true_state.x:6.2f}, {true_state.y:6.2f}), "
                      f"Est=({est_x:6.2f}, {est_y:6.2f}), "
                      f"Error={math.sqrt((true_state.x-est_x)**2 + (true_state.y-est_y)**2):6.2f}m")

            time.sleep(dt * 0.1)  # Slow down simulation for observation

    except KeyboardInterrupt:
        print("\nSimulation stopped by user.")

    # Final statistics
    final_true = fusion_system.true_positions[-1] if fusion_system.true_positions else (0, 0)
    final_est = fusion_system.estimated_positions[-1] if fusion_system.estimated_positions else (0, 0)
    final_error = math.sqrt((final_true[0] - final_est[0])**2 + (final_true[1] - final_est[1])**2)

    print(f"\nFinal positions:")
    print(f"  True:     ({final_true[0]:6.2f}, {final_true[1]:6.2f})")
    print(f"  Estimated: ({final_est[0]:6.2f}, {final_est[1]:6.2f})")
    print(f"  Final error: {final_error:6.2f}m")
    print(f"Total steps: {len(fusion_system.time_stamps)}")
    print("Sensor fusion simulation completed.")


if __name__ == "__main__":
    main()