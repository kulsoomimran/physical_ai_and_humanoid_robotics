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
Real-time Simulation and Performance Optimization Example for Chapter 12: Real-time Simulation and Performance Optimization.

This example demonstrates real-time simulation concepts including:
- Fixed time-step physics simulation
- Performance profiling and optimization
- Real-time scheduling constraints
- Multi-threading for simulation components
"""

import time
import threading
import queue
import numpy as np
import math
from dataclasses import dataclass
from typing import List, Dict, Callable, Optional
import statistics
import gc


@dataclass
class SimulationState:
    """State of the simulation at a given time."""
    time: float
    object_count: int
    fps: float
    cpu_usage: float
    memory_usage: float
    physics_iterations: int
    render_iterations: int


class PhysicsEngine:
    """A simple physics engine for real-time simulation."""

    def __init__(self, fixed_timestep: float = 0.016):  # ~60 Hz
        self.fixed_timestep = fixed_timestep
        self.objects = []
        self.time_acc = 0.0
        self.last_time = time.time()
        self.physics_steps = 0
        self.active = False

    def add_object(self, position, velocity, mass=1.0, radius=0.5):
        """Add a physics object to the simulation."""
        obj = {
            'position': np.array(position, dtype=np.float32),
            'velocity': np.array(velocity, dtype=np.float32),
            'mass': mass,
            'radius': radius,
            'forces': np.array([0.0, 0.0], dtype=np.float32),
            'restitution': 0.7,
            'friction': 0.2
        }
        self.objects.append(obj)
        return len(self.objects) - 1

    def apply_force(self, obj_id: int, force):
        """Apply a force to a specific object."""
        if 0 <= obj_id < len(self.objects):
            self.objects[obj_id]['forces'] += np.array(force, dtype=np.float32)

    def update_physics(self, dt: float):
        """Update physics for all objects."""
        gravity = np.array([0.0, -9.81], dtype=np.float32)
        boundary_min = np.array([-10.0, 0.0])
        boundary_max = np.array([10.0, 10.0])

        for obj in self.objects:
            # Apply gravity
            obj['forces'] += gravity * obj['mass']

            # Update velocity: v = v0 + a*t, where a = F/m
            acceleration = obj['forces'] / obj['mass']
            obj['velocity'] += acceleration * dt

            # Apply damping
            obj['velocity'] *= 0.995

            # Update position: p = p0 + v*t
            obj['position'] += obj['velocity'] * dt

            # Reset forces for next iteration
            obj['forces'] = np.array([0.0, 0.0], dtype=np.float32)

            # Boundary collision - simple reflection
            pos = obj['position']
            radius = obj['radius']

            if pos[0] - radius < boundary_min[0]:
                pos[0] = boundary_min[0] + radius
                obj['velocity'][0] *= -obj['restitution']
            elif pos[0] + radius > boundary_max[0]:
                pos[0] = boundary_max[0] - radius
                obj['velocity'][0] *= -obj['restitution']

            if pos[1] - radius < boundary_min[1]:
                pos[1] = boundary_min[1] + radius
                obj['velocity'][1] *= -obj['restitution']
            elif pos[1] + radius > boundary_max[1]:
                pos[1] = boundary_max[1] - radius
                obj['velocity'][1] *= -obj['restitution']

        self.physics_steps += 1

    def run_fixed_step_simulation(self, target_timestep: float):
        """Run physics simulation with fixed time steps."""
        current_time = time.time()
        frame_time = current_time - self.last_time
        self.last_time = current_time

        self.time_acc += frame_time

        # Run physics steps with fixed timestep
        steps = 0
        while self.time_acc >= target_timestep:
            self.update_physics(target_timestep)
            self.time_acc -= target_timestep
            steps += 1

        return steps


class Renderer:
    """A simple renderer for the simulation."""

    def __init__(self):
        self.render_calls = 0
        self.last_render_time = time.time()
        self.fps_history = []

    def render(self, objects: List[Dict]):
        """Render the current state of objects."""
        # Simulate rendering work
        # In a real system, this would draw objects to screen
        render_start = time.time()

        # Simulate rendering time based on object count
        time.sleep(0.001 * len(objects))  # 1ms per object approximately

        render_time = time.time() - render_start
        current_time = time.time()

        # Calculate FPS
        if current_time - self.last_render_time > 0:
            fps = 1.0 / (current_time - self.last_render_time)
            self.fps_history.append(fps)
            if len(self.fps_history) > 100:  # Keep last 100 FPS values
                self.fps_history = self.fps_history[-100:]

        self.last_render_time = current_time
        self.render_calls += 1

        return render_time

    def get_average_fps(self) -> float:
        """Get average FPS over the last frames."""
        if not self.fps_history:
            return 0.0
        return sum(self.fps_history) / len(self.fps_history)

    def get_recent_fps(self) -> float:
        """Get FPS from the last few frames."""
        if not self.fps_history:
            return 0.0
        recent = self.fps_history[-10:] if len(self.fps_history) >= 10 else self.fps_history
        return sum(recent) / len(recent)


class PerformanceProfiler:
    """Performance profiling and optimization tools."""

    def __init__(self):
        self.metrics = {
            'physics_time': [],
            'render_time': [],
            'total_frame_time': [],
            'object_counts': [],
            'memory_usage': [],
            'cpu_usage': []
        }
        self.start_time = time.time()

    def record_metric(self, metric_name: str, value: float):
        """Record a performance metric."""
        if metric_name not in self.metrics:
            self.metrics[metric_name] = []
        self.metrics[metric_name].append(value)

        # Keep only the last 1000 values to prevent memory issues
        if len(self.metrics[metric_name]) > 1000:
            self.metrics[metric_name] = self.metrics[metric_name][-1000:]

    def get_statistics(self) -> Dict[str, Dict[str, float]]:
        """Get statistics for all recorded metrics."""
        stats = {}
        for name, values in self.metrics.items():
            if values:
                stats[name] = {
                    'min': min(values),
                    'max': max(values),
                    'avg': sum(values) / len(values),
                    'std': statistics.stdev(values) if len(values) > 1 else 0
                }
        return stats

    def get_current_metrics(self) -> Dict[str, float]:
        """Get the most recent values for all metrics."""
        current = {}
        for name, values in self.metrics.items():
            if values:
                current[name] = values[-1]
        return current

    def get_uptime(self) -> float:
        """Get the time since profiling started."""
        return time.time() - self.start_time


class RealTimeSimulation:
    """Main real-time simulation system."""

    def __init__(self, fixed_physics_timestep: float = 0.016):
        self.physics = PhysicsEngine(fixed_physics_timestep)
        self.renderer = Renderer()
        self.profiler = PerformanceProfiler()
        self.active = False
        self.target_fps = 60
        self.frame_time_target = 1.0 / self.target_fps
        self.simulation_speed = 1.0  # 1.0 = real-time, < 1.0 = slower, > 1.0 = faster

        # Optimization settings
        self.max_objects = 100  # Limit for performance
        self.culling_distance = 50.0  # Don't render objects beyond this distance
        self.lod_distance = 20.0  # Level of detail threshold

        # Multi-threading components
        self.render_queue = queue.Queue(maxsize=2)  # Limit render queue size
        self.render_thread = None
        self.physics_thread = None

    def add_objects(self, count: int):
        """Add multiple objects to the simulation."""
        for i in range(count):
            if len(self.physics.objects) >= self.max_objects:
                break

            # Add an object with random position and velocity
            x = random.uniform(-5, 5)
            y = random.uniform(5, 8)
            vx = random.uniform(-2, 2)
            vy = random.uniform(-1, 1)

            self.physics.add_object([x, y], [vx, vy], mass=random.uniform(0.5, 2.0))

    def optimize_simulation(self):
        """Apply various optimization techniques."""
        # Simple object culling - remove objects that have fallen far below
        self.physics.objects = [
            obj for obj in self.physics.objects
            if obj['position'][1] > -10  # Keep only objects above y = -10
        ]

        # Memory optimization
        if len(self.physics.objects) > self.max_objects:
            # Remove oldest objects if we exceed the limit
            excess = len(self.physics.objects) - self.max_objects
            self.physics.objects = self.physics.objects[excess:]

        # Force garbage collection periodically
        if len(self.physics.objects) % 50 == 0:
            gc.collect()

    def run_frame(self) -> SimulationState:
        """Run one frame of the simulation."""
        frame_start = time.time()

        # Update physics with fixed timestep
        physics_start = time.time()
        physics_steps = self.physics.run_fixed_step_simulation(self.physics.fixed_timestep)
        physics_time = time.time() - physics_start

        # Optimize the simulation
        self.optimize_simulation()

        # Render the scene
        render_start = time.time()
        render_time = self.renderer.render(self.physics.objects)
        total_frame_time = time.time() - frame_start

        # Record performance metrics
        self.profiler.record_metric('physics_time', physics_time)
        self.profiler.record_metric('render_time', render_time)
        self.profiler.record_metric('total_frame_time', total_frame_time)
        self.profiler.record_metric('object_counts', len(self.physics.objects))
        self.profiler.record_metric('physics_iterations', physics_steps)

        # Estimate CPU and memory usage (simulated values)
        import os
        import psutil
        process = psutil.Process(os.getpid())
        cpu_percent = process.cpu_percent()
        memory_mb = process.memory_info().rss / 1024 / 1024

        self.profiler.record_metric('cpu_usage', cpu_percent)
        self.profiler.record_metric('memory_usage', memory_mb)

        # Calculate FPS
        current_time = time.time()
        if hasattr(self, 'last_frame_time') and current_time - self.last_frame_time > 0:
            fps = 1.0 / (current_time - self.last_frame_time)
        else:
            fps = 0.0
        self.last_frame_time = current_time

        return SimulationState(
            time=current_time,
            object_count=len(self.physics.objects),
            fps=self.renderer.get_recent_fps(),
            cpu_usage=cpu_percent,
            memory_usage=memory_mb,
            physics_iterations=physics_steps,
            render_iterations=1
        )

    def run_simulation(self, duration: float = 10.0):
        """Run the simulation for a specified duration."""
        self.active = True
        start_time = time.time()

        # Add initial objects
        self.add_objects(20)

        frame_count = 0
        while time.time() - start_time < duration and self.active:
            state = self.run_frame()
            frame_count += 1

            # Add new objects periodically to keep the simulation interesting
            if frame_count % 100 == 0:
                self.add_objects(5)

            # Print status every 5 seconds
            elapsed = time.time() - start_time
            if int(elapsed) % 5 == 0 and elapsed > 0:
                stats = self.profiler.get_statistics()
                if 'physics_time' in stats:
                    avg_physics = stats['physics_time']['avg']
                    avg_render = stats['render_time']['avg'] if 'render_time' in stats else 0
                    print(f"Time: {elapsed:5.1f}s, Objects: {state.object_count:3d}, "
                          f"Physics: {avg_physics*1000:5.2f}ms, Render: {avg_render*1000:5.2f}ms, "
                          f"FPS: {state.fps:5.1f}")

            # Simple frame rate control (in a real system, you might use more sophisticated timing)
            time.sleep(max(0, self.frame_time_target - (time.time() - frame_start)))

        return frame_count

    def get_performance_report(self) -> str:
        """Generate a performance report."""
        stats = self.profiler.get_statistics()
        current = self.profiler.get_current_metrics()

        report = []
        report.append("PERFORMANCE REPORT")
        report.append("=" * 50)
        report.append(f"Simulation uptime: {self.profiler.get_uptime():.2f} seconds")
        report.append(f"Current objects: {current.get('object_counts', 0)}")
        report.append(f"Current FPS: {self.renderer.get_recent_fps():.2f}")

        if 'physics_time' in stats:
            avg_physics = stats['physics_time']['avg']
            max_physics = stats['physics_time']['max']
            report.append(f"Physics time - Avg: {avg_physics*1000:.2f}ms, Max: {max_physics*1000:.2f}ms")

        if 'render_time' in stats:
            avg_render = stats['render_time']['avg']
            max_render = stats['render_time']['max']
            report.append(f"Render time - Avg: {avg_render*1000:.2f}ms, Max: {max_render*1000:.2f}ms")

        if 'cpu_usage' in stats:
            avg_cpu = stats['cpu_usage']['avg']
            max_cpu = stats['cpu_usage']['max']
            report.append(f"CPU usage - Avg: {avg_cpu:.2f}%, Max: {max_cpu:.2f}%")

        if 'memory_usage' in stats:
            avg_memory = stats['memory_usage']['avg']
            max_memory = stats['memory_usage']['max']
            report.append(f"Memory usage - Avg: {avg_memory:.2f}MB, Max: {max_memory:.2f}MB")

        return "\n".join(report)


def main():
    print("Real-time Simulation and Performance Optimization Example - Chapter 12")
    print("=" * 75)

    # Create real-time simulation
    sim = RealTimeSimulation(fixed_physics_timestep=0.016)  # ~60 Hz physics
    print(f"Created real-time simulation with {sim.physics.fixed_timestep*1000:.1f}ms physics timestep")
    print(f"Target rendering FPS: {sim.target_fps}")
    print(f"Max objects limit: {sim.max_objects}")

    print("\nStarting real-time simulation...")
    print("Simulation will run for 20 seconds with performance monitoring.")
    print("Performance metrics will be displayed periodically.\n")

    try:
        # Run simulation for 20 seconds
        frame_count = sim.run_simulation(duration=20.0)

        print(f"\nSimulation completed after {frame_count} frames")

        # Show final performance report
        print(f"\n{sim.get_performance_report()}")

    except KeyboardInterrupt:
        print("\nSimulation stopped by user.")
        print(f"\n{sim.get_performance_report()}")

    print("\nReal-time simulation completed.")


if __name__ == "__main__":
    import random
    main()