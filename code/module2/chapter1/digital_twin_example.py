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
Digital Twin Example for Chapter 7: Introduction to Digital Twin Technology.

This example demonstrates fundamental digital twin concepts including:
- Virtual representation of a physical system
- Real-time data synchronization
- Data-driven insights and decision making
"""

import time
import json
import threading
from datetime import datetime
from dataclasses import dataclass, asdict
from typing import Dict, Any, Callable
import random


@dataclass
class PhysicalSystemState:
    """Represents the state of a physical system (e.g., robot, sensor, etc.)"""
    temperature: float
    pressure: float
    vibration: float
    operational: bool
    timestamp: float
    efficiency: float


class PhysicalSystem:
    """Simulates a physical system that can be represented by a digital twin."""

    def __init__(self, system_id: str):
        self.system_id = system_id
        self.state = PhysicalSystemState(
            temperature=20.0,
            pressure=1.0,
            vibration=0.1,
            operational=True,
            timestamp=time.time(),
            efficiency=1.0
        )
        self.running = False
        self._lock = threading.Lock()

    def start_simulation(self):
        """Start simulating the physical system."""
        self.running = True
        self._simulation_thread = threading.Thread(target=self._simulate)
        self._simulation_thread.start()

    def stop_simulation(self):
        """Stop simulating the physical system."""
        self.running = False
        if hasattr(self, '_simulation_thread'):
            self._simulation_thread.join()

    def _simulate(self):
        """Internal method to simulate physical system changes."""
        while self.running:
            with self._lock:
                # Simulate changes in system state
                self.state.temperature += random.uniform(-0.5, 0.5)
                self.state.pressure += random.uniform(-0.01, 0.01)
                self.state.vibration += random.uniform(-0.01, 0.02)
                self.state.timestamp = time.time()

                # Simulate efficiency changes based on conditions
                if self.state.temperature > 30:
                    self.state.efficiency = max(0.5, self.state.efficiency - 0.01)
                elif self.state.temperature < 15:
                    self.state.efficiency = min(1.0, self.state.efficiency + 0.005)
                else:
                    self.state.efficiency = min(1.0, self.state.efficiency + 0.001)

                # Simulate occasional operational issues
                if random.random() < 0.001:  # 0.1% chance per iteration
                    self.state.operational = False

                if not self.state.operational and random.random() < 0.05:  # 5% chance to recover
                    self.state.operational = True

            time.sleep(1)  # Update every second

    def get_state(self) -> PhysicalSystemState:
        """Get the current state of the physical system."""
        with self._lock:
            return self.state


class DigitalTwin:
    """Digital twin that mirrors the physical system and provides insights."""

    def __init__(self, physical_system: PhysicalSystem):
        self.physical_system = physical_system
        self.historical_data = []
        self.insights = []
        self.alerts = []
        self.analysis_callbacks = []

    def sync_with_physical(self):
        """Synchronize with the physical system to get latest state."""
        current_state = self.physical_system.get_state()
        self.historical_data.append(current_state)

        # Keep only the last 1000 data points to prevent memory issues
        if len(self.historical_data) > 1000:
            self.historical_data = self.historical_data[-1000:]

        # Generate insights based on the current state
        self._analyze_state(current_state)

        # Run any registered analysis callbacks
        for callback in self.analysis_callbacks:
            callback(current_state)

    def _analyze_state(self, state: PhysicalSystemState):
        """Analyze the current state and generate insights."""
        # Check for anomalies
        if state.temperature > 35:
            alert = f"WARNING: High temperature detected: {state.temperature:.2f}°C"
            self.alerts.append(alert)
            self.insights.append(f"Heat-related issue detected at {datetime.fromtimestamp(state.timestamp)}")

        if state.vibration > 0.5:
            alert = f"WARNING: High vibration detected: {state.vibration:.3f}"
            self.alerts.append(alert)
            self.insights.append(f"Mechanical issue detected at {datetime.fromtimestamp(state.timestamp)}")

        if not state.operational:
            alert = "CRITICAL: System is not operational"
            self.alerts.append(alert)

        # Calculate trends
        if len(self.historical_data) >= 10:
            recent_temps = [s.temperature for s in self.historical_data[-10:]]
            if all(recent_temps[i] > recent_temps[i-1] for i in range(1, len(recent_temps))):
                self.insights.append("Temperature increasing trend detected")

    def get_insights(self) -> list:
        """Get all generated insights."""
        return self.insights.copy()

    def get_alerts(self) -> list:
        """Get all generated alerts."""
        return self.alerts.copy()

    def get_efficiency_trend(self) -> float:
        """Calculate efficiency trend over time."""
        if len(self.historical_data) < 2:
            return 0.0

        recent_states = self.historical_data[-10:]  # Last 10 states
        efficiencies = [s.efficiency for s in recent_states]

        if len(efficiencies) < 2:
            return 0.0

        # Calculate simple trend (slope)
        start_eff = efficiencies[0]
        end_eff = efficiencies[-1]
        return end_eff - start_eff

    def register_analysis_callback(self, callback: Callable[[PhysicalSystemState], None]):
        """Register a callback function for custom analysis."""
        self.analysis_callbacks.append(callback)


def print_system_info(physical_system: PhysicalSystem):
    """Callback function to print system information."""
    state = physical_system.get_state()
    print(f"[{datetime.fromtimestamp(state.timestamp).strftime('%H:%M:%S')}] "
          f"Temp: {state.temperature:.2f}°C, "
          f"Pressure: {state.pressure:.3f}, "
          f"Vibration: {state.vibration:.3f}, "
          f"Efficiency: {state.efficiency:.3f}, "
          f"Operational: {state.operational}")


def main():
    print("Digital Twin Example - Introduction to Digital Twin Technology")
    print("=" * 60)

    # Create a physical system
    physical_system = PhysicalSystem("robot_arm_001")
    physical_system.start_simulation()

    # Create a digital twin for the physical system
    digital_twin = DigitalTwin(physical_system)

    # Register a callback for real-time monitoring
    digital_twin.register_analysis_callback(
        lambda state: print_system_info(physical_system)
    )

    print("Starting Digital Twin simulation...")
    print("Physical system is being simulated with random variations.")
    print("Digital twin will synchronize and analyze the data.\n")

    try:
        for i in range(30):  # Run for 30 iterations (30 seconds)
            # Synchronize the digital twin with the physical system
            digital_twin.sync_with_physical()

            # Periodically show insights
            if i % 10 == 0 and i > 0:
                print(f"\n--- INSIGHTS (Iteration {i}) ---")
                insights = digital_twin.get_insights()
                if insights:
                    for insight in insights[-5:]:  # Show last 5 insights
                        print(f"• {insight}")

                alerts = digital_twin.get_alerts()
                if alerts:
                    print(f"\n--- ALERTS ---")
                    for alert in alerts[-3:]:  # Show last 3 alerts
                        print(f"⚠️  {alert}")

                efficiency_trend = digital_twin.get_efficiency_trend()
                print(f"\nEfficiency Trend: {efficiency_trend:+.4f}")

            time.sleep(1)

    except KeyboardInterrupt:
        print("\nSimulation stopped by user.")

    finally:
        physical_system.stop_simulation()
        print("\nDigital Twin simulation completed.")
        print(f"Total data points collected: {len(digital_twin.historical_data)}")
        print(f"Total insights generated: {len(digital_twin.insights)}")
        print(f"Total alerts generated: {len(digital_twin.alerts)}")


if __name__ == "__main__":
    main()