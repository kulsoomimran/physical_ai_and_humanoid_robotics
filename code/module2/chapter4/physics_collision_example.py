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
Physics Simulation and Collision Detection Example for Chapter 10: Physics Simulation and Collision Detection.

This example demonstrates physics simulation concepts including:
- Rigid body dynamics
- Collision detection algorithms
- Contact response and friction
- Physics engines and simulation parameters
"""

import time
import math
from dataclasses import dataclass
from typing import List, Tuple, Optional
import random


@dataclass
class Vector3:
    """3D vector for physics calculations."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def magnitude(self) -> float:
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    def normalize(self):
        mag = self.magnitude()
        if mag > 0:
            self.x /= mag
            self.y /= mag
            self.z /= mag

    def __add__(self, other):
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar):
        return Vector3(self.x * scalar, self.y * scalar, self.z * scalar)

    def dot(self, other) -> float:
        return self.x * other.x + self.y * other.y + self.z * other.z


@dataclass
class PhysicsBody:
    """Represents a physics body with mass, position, velocity, etc."""
    position: Vector3
    velocity: Vector3
    acceleration: Vector3
    mass: float
    radius: float  # For spherical objects
    restitution: float = 0.5  # Bounciness (0 = no bounce, 1 = perfect bounce)
    friction: float = 0.2  # Friction coefficient
    is_static: bool = False  # Whether the body is fixed in place
    force: Vector3 = None

    def __post_init__(self):
        if self.force is None:
            self.force = Vector3(0, 0, 0)

    def apply_force(self, force: Vector3):
        """Apply a force to the body."""
        self.force.x += force.x
        self.force.y += force.force
        self.force.z += force.z

    def update(self, dt: float):
        """Update the body's state based on physics."""
        if self.is_static:
            return

        # F = ma, so a = F/m
        self.acceleration.x = self.force.x / self.mass
        self.acceleration.y = self.force.y / self.mass
        self.acceleration.z = self.force.z / self.mass

        # Update velocity: v = v0 + a*t
        self.velocity.x += self.acceleration.x * dt
        self.velocity.y += self.acceleration.y * dt
        self.velocity.z += self.acceleration.z * dt

        # Update position: p = p0 + v*t
        self.position.x += self.velocity.x * dt
        self.position.y += self.velocity.y * dt
        self.position.z += self.velocity.z * dt

        # Reset forces for next iteration
        self.force = Vector3(0, 0, 0)


class PhysicsWorld:
    """A physics simulation world with gravity and collision handling."""

    def __init__(self, gravity: Vector3 = None, bounds: Tuple[float, float, float, float] = (-10, 10, -10, 10)):
        self.gravity = gravity or Vector3(0, -9.81, 0)  # Default Earth gravity
        self.bounds = bounds  # (min_x, max_x, min_z, max_z) - Y is up
        self.bodies: List[PhysicsBody] = []
        self.ground_level = 0.0
        self.linear_damping = 0.99
        self.angular_damping = 0.99

    def add_body(self, body: PhysicsBody):
        """Add a physics body to the world."""
        self.bodies.append(body)

    def update(self, dt: float):
        """Update the physics simulation."""
        # Apply gravity to all non-static bodies
        for body in self.bodies:
            if not body.is_static:
                gravity_force = Vector3(
                    self.gravity.x * body.mass,
                    self.gravity.y * body.mass,
                    self.gravity.z * body.mass
                )
                body.apply_force(gravity_force)

                # Apply damping to simulate air resistance
                body.velocity.x *= self.linear_damping
                body.velocity.y *= self.linear_damping
                body.velocity.z *= self.linear_damping

        # Update all bodies
        for body in self.bodies:
            body.update(dt)

        # Handle collisions
        self.handle_collisions()

        # Handle ground collisions
        self.handle_ground_collisions()

        # Handle boundary collisions
        self.handle_boundary_collisions()

    def handle_collisions(self):
        """Detect and resolve collisions between bodies."""
        for i in range(len(self.bodies)):
            for j in range(i + 1, len(self.bodies)):
                body1 = self.bodies[i]
                body2 = self.bodies[j]

                if body1.is_static and body2.is_static:
                    continue

                # Calculate distance between centers
                distance_vector = Vector3(
                    body2.position.x - body1.position.x,
                    body2.position.y - body1.position.y,
                    body2.position.z - body1.position.z
                )
                distance = distance_vector.magnitude()

                # Check if bodies are colliding
                if distance < body1.radius + body2.radius:
                    # Calculate collision normal (from body1 to body2)
                    if distance > 0:
                        normal = Vector3(
                            distance_vector.x / distance,
                            distance_vector.y / distance,
                            distance_vector.z / distance
                        )
                    else:
                        # Bodies are at the same position, use random normal
                        normal = Vector3(1, 0, 0)

                    # Calculate relative velocity
                    rel_velocity = Vector3(
                        body2.velocity.x - body1.velocity.x,
                        body2.velocity.y - body1.velocity.y,
                        body2.velocity.z - body1.velocity.z
                    )

                    # Calculate velocity along normal
                    vel_along_normal = rel_velocity.dot(normal)

                    # Do not resolve if velocities are separating
                    if vel_along_normal > 0:
                        continue

                    # Calculate restitution (bounciness)
                    restitution = min(body1.restitution, body2.restitution)

                    # Calculate impulse scalar
                    impulse_scalar = -(1 + restitution) * vel_along_normal
                    impulse_scalar /= (1/body1.mass + 1/body2.mass)

                    # Apply impulse
                    impulse = Vector3(
                        impulse_scalar * normal.x,
                        impulse_scalar * normal.y,
                        impulse_scalar * normal.z
                    )

                    # Apply friction
                    # Calculate tangent vector (perpendicular to normal)
                    rel_velocity_normal = Vector3(
                        normal.x * rel_velocity.dot(normal),
                        normal.y * rel_velocity.dot(normal),
                        normal.z * rel_velocity.dot(normal)
                    )
                    rel_velocity_tangent = Vector3(
                        rel_velocity.x - rel_velocity_normal.x,
                        rel_velocity.y - rel_velocity_normal.y,
                        rel_velocity.z - rel_velocity_normal.z
                    )

                    # Apply friction impulse
                    friction_impulse = Vector3(
                        rel_velocity_tangent.x * -1,
                        rel_velocity_tangent.y * -1,
                        rel_velocity_tangent.z * -1
                    )
                    friction_impulse.normalize()
                    friction_impulse = friction_impulse * (impulse.magnitude() * min(body1.friction, body2.friction))

                    # Apply impulses to bodies
                    body1.velocity.x -= impulse.x / body1.mass
                    body1.velocity.y -= impulse.y / body1.mass
                    body1.velocity.z -= impulse.z / body1.mass
                    body1.velocity.x -= friction_impulse.x / body1.mass
                    body1.velocity.y -= friction_impulse.y / body1.mass
                    body1.velocity.z -= friction_impulse.z / body1.mass

                    body2.velocity.x += impulse.x / body2.mass
                    body2.velocity.y += impulse.y / body2.mass
                    body2.velocity.z += impulse.z / body2.mass
                    body2.velocity.x += friction_impulse.x / body2.mass
                    body2.velocity.y += friction_impulse.y / body2.mass
                    body2.velocity.z += friction_impulse.z / body2.mass

                    # Positional correction to prevent sinking
                    percent = 0.2  # Penetration percentage to correct
                    slop = 0.01  # Small separation allowance
                    correction = Vector3(
                        (max(distance - (body1.radius + body2.radius), -slop) / (1/body1.mass + 1/body2.mass)) * percent * normal.x,
                        (max(distance - (body1.radius + body2.radius), -slop) / (1/body1.mass + 1/body2.mass)) * percent * normal.y,
                        (max(distance - (body1.radius + body2.radius), -slop) / (1/body1.mass + 1/body2.mass)) * percent * normal.z
                    )

                    body1.position.x -= correction.x / body1.mass
                    body1.position.y -= correction.y / body1.mass
                    body1.position.z -= correction.z / body1.mass

                    body2.position.x += correction.x / body2.mass
                    body2.position.y += correction.y / body2.mass
                    body2.position.z += correction.z / body2.mass

    def handle_ground_collisions(self):
        """Handle collisions with the ground plane."""
        for body in self.bodies:
            if body.is_static:
                continue

            # Check if body is below ground level
            if body.position.y - body.radius < self.ground_level:
                # Calculate penetration depth
                penetration = self.ground_level - (body.position.y - body.radius)

                # Correct position
                body.position.y = self.ground_level + body.radius

                # Calculate velocity at contact point (for a sphere, it's just the center velocity)
                vel_dot_normal = body.velocity.y  # Velocity in the normal direction (up)

                # Do not resolve if object is moving away from ground
                if vel_dot_normal > 0:
                    continue

                # Calculate impulse for bounce
                impulse = -(1 + body.restitution) * body.mass * vel_dot_normal

                # Apply bounce impulse
                body.velocity.y += impulse / body.mass

                # Apply friction
                friction_impulse = body.velocity.x * body.friction
                body.velocity.x *= (1 - body.friction)

    def handle_boundary_collisions(self):
        """Handle collisions with world boundaries."""
        min_x, max_x, min_z, max_z = self.bounds

        for body in self.bodies:
            if body.is_static:
                continue

            # X boundary collisions
            if body.position.x - body.radius < min_x:
                body.position.x = min_x + body.radius
                body.velocity.x *= -body.restitution
            elif body.position.x + body.radius > max_x:
                body.position.x = max_x - body.radius
                body.velocity.x *= -body.restitution

            # Z boundary collisions (Y is up in this simulation)
            if body.position.z - body.radius < min_z:
                body.position.z = min_z + body.radius
                body.velocity.z *= -body.restitution
            elif body.position.z + body.radius > max_z:
                body.position.z = max_z - body.radius
                body.velocity.z *= -body.restitution


class CollisionDetector:
    """A specialized collision detection system."""

    def __init__(self):
        self.collision_pairs = []

    def broad_phase(self, bodies: List[PhysicsBody]) -> List[Tuple[int, int]]:
        """Broad phase collision detection using simple bounds checking."""
        pairs = []
        for i in range(len(bodies)):
            for j in range(i + 1, len(bodies)):
                # Simple distance check for broad phase
                dx = bodies[i].position.x - bodies[j].position.x
                dy = bodies[i].position.y - bodies[j].position.y
                dz = bodies[i].position.z - bodies[j].position.z
                distance_squared = dx*dx + dy*dy + dz*dz
                max_distance = bodies[i].radius + bodies[j].radius

                if distance_squared < max_distance * max_distance:
                    pairs.append((i, j))
        return pairs

    def narrow_phase(self, body1: PhysicsBody, body2: PhysicsBody) -> bool:
        """Narrow phase collision detection."""
        dx = body2.position.x - body1.position.x
        dy = body2.position.y - body1.position.y
        dz = body2.position.z - body1.position.z
        distance_squared = dx*dx + dy*dy + dz*dz
        radius_sum = body1.radius + body2.radius

        return distance_squared < radius_sum * radius_sum


def main():
    print("Physics Simulation and Collision Detection Example - Chapter 10")
    print("=" * 65)

    # Create a physics world
    world = PhysicsWorld(gravity=Vector3(0, -9.81, 0), bounds=(-5, 5, -5, 5))
    print(f"Created physics world with gravity: ({world.gravity.x}, {world.gravity.y}, {world.gravity.z})")
    print(f"World bounds: X({world.bounds[0]}, {world.bounds[1]}), Z({world.bounds[2]}, {world.bounds[3]})")

    # Create some physics bodies
    # Ball 1 - bouncy ball
    ball1 = PhysicsBody(
        position=Vector3(0, 8, 0),  # Start high
        velocity=Vector3(0, 0, 0),
        acceleration=Vector3(0, 0, 0),
        mass=1.0,
        radius=0.5,
        restitution=0.8,  # Very bouncy
        friction=0.1
    )
    world.add_body(ball1)
    print("Added bouncy ball at (0, 8, 0)")

    # Ball 2 - less bouncy ball
    ball2 = PhysicsBody(
        position=Vector3(2, 10, 1),
        velocity=Vector3(0, 0, 0),
        acceleration=Vector3(0, 0, 0),
        mass=1.5,
        radius=0.4,
        restitution=0.3,  # Less bouncy
        friction=0.2
    )
    world.add_body(ball2)
    print("Added less bouncy ball at (2, 10, 1)")

    # Static platform
    platform = PhysicsBody(
        position=Vector3(0, 2, 0),
        velocity=Vector3(0, 0, 0),
        acceleration=Vector3(0, 0, 0),
        mass=0,  # Infinite mass (static)
        radius=2.0,  # Large radius to make it wide
        restitution=0.1,
        friction=0.5,
        is_static=True
    )
    world.add_body(platform)
    print("Added static platform at (0, 2, 0)")

    # Create collision detector
    detector = CollisionDetector()
    print("\nStarting physics simulation...")
    print("Balls will fall and bounce with collision detection and response.\n")

    # Simulation parameters
    dt = 0.01  # Small time step for accuracy
    max_steps = 2000  # Run for 20 seconds (2000 * 0.01s)
    collision_count = 0

    try:
        for step in range(max_steps):
            # Update physics world
            world.update(dt)

            # Perform collision detection
            potential_collisions = detector.broad_phase(world.bodies)
            actual_collisions = 0
            for i, j in potential_collisions:
                if detector.narrow_phase(world.bodies[i], world.bodies[j]):
                    actual_collisions += 1
                    collision_count += 1

            # Print status every 200 steps (2 seconds)
            if step % 200 == 0:
                ball1_pos = world.bodies[0].position
                ball2_pos = world.bodies[1].position
                print(f"Step {step:4d}: Ball1=({ball1_pos.x:5.2f}, {ball1_pos.y:5.2f}, {ball1_pos.z:5.2f}), "
                      f"Ball2=({ball2_pos.x:5.2f}, {ball2_pos.y:5.2f}, {ball2_pos.z:5.2f}), "
                      f"Collisions={collision_count}")

            time.sleep(dt * 10)  # Slow down simulation for observation

    except KeyboardInterrupt:
        print("\nSimulation stopped by user.")

    # Final status
    final_ball1 = world.bodies[0]
    final_ball2 = world.bodies[1]
    print(f"\nFinal positions:")
    print(f"  Ball 1: ({final_ball1.position.x:5.2f}, {final_ball1.position.y:5.2f}, {final_ball1.position.z:5.2f})")
    print(f"  Ball 2: ({final_ball2.position.x:5.2f}, {final_ball2.position.y:5.2f}, {final_ball2.position.z:5.2f})")
    print(f"Total collisions detected: {collision_count}")
    print("Physics simulation completed.")


if __name__ == "__main__":
    main()