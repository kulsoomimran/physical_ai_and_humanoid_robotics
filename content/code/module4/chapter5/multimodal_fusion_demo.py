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
Multimodal Fusion and Decision Making Demo for Chapter 23: Multimodal Fusion and Decision Making.

This example demonstrates multimodal fusion concepts including:
- Sensor data fusion from multiple modalities
- Decision making under uncertainty
- Bayesian inference for multimodal reasoning
- Attention mechanisms for modality weighting
- Uncertainty quantification and propagation
"""

import numpy as np
import time
from dataclasses import dataclass
from typing import List, Dict, Any, Optional, Tuple, Union
import math
import random
from scipy.stats import norm


@dataclass
class SensorObservation:
    """Represents an observation from a sensor modality."""
    modality: str  # 'vision', 'audio', 'tactile', 'lidar', etc.
    data: np.ndarray
    timestamp: float
    confidence: float
    uncertainty: float


@dataclass
class FusedBelief:
    """Represents a fused belief state from multiple modalities."""
    state_estimate: np.ndarray
    uncertainty: np.ndarray  # Covariance matrix
    confidence: float
    timestamp: float
    source_modalities: List[str]


@dataclass
class Decision:
    """Represents a decision made based on multimodal information."""
    action: str
    confidence: float
    expected_utility: float
    timestamp: float
    supporting_evidence: List[Dict[str, Any]]


class KalmanFilter:
    """Simple Kalman filter for fusing temporal observations."""

    def __init__(self, state_dim: int):
        self.state_dim = state_dim
        # Initialize state and covariance
        self.state = np.zeros(state_dim)
        self.covariance = np.eye(state_dim) * 1000.0  # High initial uncertainty

        # Process noise (how much we expect the state to change)
        self.process_noise = np.eye(state_dim) * 0.1

        # Measurement noise (sensor uncertainty)
        self.measurement_noise = np.eye(state_dim) * 1.0

    def predict(self):
        """Predict the next state (identity model - no motion model)."""
        # In a real system, this would incorporate motion models
        pass

    def update(self, measurement: np.ndarray, measurement_noise: Optional[np.ndarray] = None):
        """Update the state estimate with a new measurement."""
        if measurement_noise is None:
            measurement_noise = self.measurement_noise

        # Innovation (measurement prediction error)
        innovation = measurement - self.state
        innovation_covariance = self.covariance + measurement_noise

        # Kalman gain
        kalman_gain = self.covariance @ np.linalg.inv(innovation_covariance)

        # Update state and covariance
        self.state = self.state + kalman_gain @ innovation
        self.covariance = (np.eye(self.state_dim) - kalman_gain) @ self.covariance

        return self.state.copy(), self.covariance.copy()


class BayesianFusion:
    """Bayesian fusion of multiple sensor modalities."""

    def __init__(self):
        self.kalman_filters = {}  # Separate filter for each modality

    def fuse_observations(self, observations: List[SensorObservation]) -> FusedBelief:
        """Fuse multiple sensor observations using Bayesian principles."""
        if not observations:
            return FusedBelief(
                state_estimate=np.zeros(3),  # Default 3D position
                uncertainty=np.eye(3) * 1000.0,
                confidence=0.0,
                timestamp=time.time(),
                source_modalities=[]
            )

        # For this example, we'll use a weighted average based on confidence
        # In practice, more sophisticated fusion methods would be used

        # Separate observations by modality
        modality_observations = {}
        for obs in observations:
            if obs.modality not in modality_observations:
                modality_observations[obs.modality] = []
            modality_observations[obs.modality].append(obs)

        # Initialize Kalman filter for each modality if needed
        for modality in modality_observations:
            if modality not in self.kalman_filters:
                self.kalman_filters[modality] = KalmanFilter(3)  # 3D position

        # Update each modality's filter
        modality_estimates = {}
        for modality, obs_list in modality_observations.items():
            # Use the most recent observation for this modality
            latest_obs = obs_list[-1]
            # Convert uncertainty to covariance matrix
            uncertainty_diag = np.ones(3) * latest_obs.uncertainty
            measurement_noise = np.diag(uncertainty_diag)

            # Update the filter
            state, cov = self.kalman_filters[modality].update(
                latest_obs.data, measurement_noise
            )
            modality_estimates[modality] = {
                'state': state,
                'covariance': cov,
                'confidence': latest_obs.confidence
            }

        # Combine estimates using confidence-weighted average
        total_weight = sum(est['confidence'] for est in modality_estimates.values())
        if total_weight == 0:
            total_weight = 1e-6  # Avoid division by zero

        # Weighted average of states
        fused_state = np.zeros(3)
        for modality, est in modality_estimates.items():
            weight = est['confidence'] / total_weight
            fused_state += weight * est['state']

        # Combined uncertainty (simplified)
        fused_uncertainty = np.eye(3) * min(est['covariance'].max() for est in modality_estimates.values())

        # Overall confidence is the average of individual confidences
        avg_confidence = sum(est['confidence'] for est in modality_estimates.values()) / len(modality_estimates)

        return FusedBelief(
            state_estimate=fused_state,
            uncertainty=fused_uncertainty,
            confidence=avg_confidence,
            timestamp=time.time(),
            source_modalities=list(modality_observations.keys())
        )


class AttentionMechanism:
    """Attention mechanism for weighting different modalities."""

    def __init__(self, num_modalities: int):
        self.num_modalities = num_modalities
        # Initialize attention weights (uniform initially)
        self.attention_weights = np.ones(num_modalities) / num_modalities

    def compute_attention_weights(self, modality_features: List[np.ndarray],
                                context: Optional[np.ndarray] = None) -> np.ndarray:
        """Compute attention weights for different modalities."""
        # Simple attention based on feature similarity
        if context is not None:
            # Compute similarity between each modality and context
            similarities = []
            for features in modality_features:
                # Compute cosine similarity
                if np.linalg.norm(features) > 0 and np.linalg.norm(context) > 0:
                    similarity = np.dot(features.flatten(), context.flatten()) / (
                        np.linalg.norm(features) * np.linalg.norm(context)
                    )
                else:
                    similarity = 0.0
                similarities.append(similarity)

            # Apply softmax to get attention weights
            exp_similarities = np.exp(similarities)
            attention_weights = exp_similarities / np.sum(exp_similarities)
        else:
            # Without context, use learned attention weights
            attention_weights = self.attention_weights / np.sum(self.attention_weights)

        return attention_weights

    def update_weights(self, modality_importance: List[float]):
        """Update attention weights based on modality importance."""
        self.attention_weights = np.array(modality_importance)
        # Normalize
        if np.sum(self.attention_weights) > 0:
            self.attention_weights = self.attention_weights / np.sum(self.attention_weights)


class DecisionMaker:
    """Makes decisions based on fused multimodal information."""

    def __init__(self):
        self.utility_functions = {
            'approach': self._approach_utility,
            'avoid': self._avoid_utility,
            'grasp': self.__grasp_utility,
            'navigate': self.__navigate_utility,
            'observe': self.__observe_utility
        }

    def make_decision(self, fused_belief: FusedBelief, context: Dict[str, Any]) -> Decision:
        """Make a decision based on the fused belief and context."""
        # Determine possible actions based on the situation
        possible_actions = self._get_possible_actions(fused_belief, context)

        best_action = None
        best_expected_utility = float('-inf')
        best_confidence = 0.0
        best_evidence = []

        for action in possible_actions:
            utility, confidence, evidence = self._evaluate_action(
                action, fused_belief, context
            )

            if utility > best_expected_utility:
                best_expected_utility = utility
                best_action = action
                best_confidence = confidence
                best_evidence = evidence

        if best_action is None:
            best_action = 'wait'  # Default action if no good option found
            best_expected_utility = 0.0
            best_confidence = 0.1
            best_evidence = [{'type': 'default', 'reason': 'No suitable action found'}]

        return Decision(
            action=best_action,
            confidence=best_confidence,
            expected_utility=best_expected_utility,
            timestamp=time.time(),
            supporting_evidence=best_evidence
        )

    def _get_possible_actions(self, fused_belief: FusedBelief, context: Dict[str, Any]) -> List[str]:
        """Determine possible actions based on belief and context."""
        actions = []

        # If there's an object in range and gripper is open, consider grasping
        if context.get('gripper_state') == 'open' and fused_belief.confidence > 0.5:
            distance = np.linalg.norm(fused_belief.state_estimate)
            if distance < 1.0:  # Within grasp range
                actions.append('grasp')

        # If there's an obstacle, consider avoidance
        if context.get('obstacle_detected', False):
            actions.extend(['avoid', 'navigate'])

        # If no specific situation, consider exploration
        if not actions:
            actions.extend(['navigate', 'observe'])

        return actions

    def _evaluate_action(self, action: str, fused_belief: FusedBelief,
                        context: Dict[str, Any]) -> Tuple[float, float, List[Dict[str, Any]]]:
        """Evaluate an action based on utility function."""
        utility_func = self.utility_functions.get(action, self._default_utility)
        utility, confidence = utility_func(fused_belief, context)

        # Collect evidence supporting this action
        evidence = self._collect_evidence(action, fused_belief, context)

        return utility, confidence, evidence

    def _approach_utility(self, fused_belief: FusedBelief, context: Dict[str, Any]) -> Tuple[float, float]:
        """Utility function for approach action."""
        distance = np.linalg.norm(fused_belief.state_estimate)
        # Higher utility for closer objects (up to a point)
        utility = max(0, 10 - distance)  # Max utility at distance 0
        confidence = fused_belief.confidence
        return utility, confidence

    def _avoid_utility(self, fused_belief: FusedBelief, context: Dict[str, Any]) -> Tuple[float, float]:
        """Utility function for avoid action."""
        distance = np.linalg.norm(fused_belief.state_estimate)
        # Higher utility for avoiding close obstacles
        if distance < 0.5:
            utility = 10  # High utility to avoid collision
        else:
            utility = 0
        confidence = fused_belief.confidence
        return utility, confidence

    def _grasp_utility(self, fused_belief: FusedBelief, context: Dict[str, Any]) -> Tuple[float, float]:
        """Utility function for grasp action."""
        distance = np.linalg.norm(fused_belief.state_estimate)
        # High utility if object is within grasp range and confidence is high
        if distance < 0.3 and fused_belief.confidence > 0.7:
            utility = 15
        elif distance < 0.5 and fused_belief.confidence > 0.5:
            utility = 8
        else:
            utility = 0
        confidence = fused_belief.confidence
        return utility, confidence

    def _navigate_utility(self, fused_belief: FusedBelief, context: Dict[str, Any]) -> Tuple[float, float]:
        """Utility function for navigate action."""
        # Utility based on distance to goal and obstacle presence
        goal_distance = context.get('goal_distance', 10.0)
        obstacle_distance = np.linalg.norm(fused_belief.state_estimate) if fused_belief.confidence > 0.3 else float('inf')

        utility = 0
        if obstacle_distance < 0.5:  # Obstacle nearby
            utility = 12  # High utility to navigate around obstacle
        else:
            utility = max(0, 10 - goal_distance)  # Approach goal

        confidence = fused_belief.confidence
        return utility, confidence

    def _observe_utility(self, fused_belief: FusedBelief, context: Dict[str, Any]) -> Tuple[float, float]:
        """Utility function for observe action."""
        # Higher utility when confidence is low (need more information)
        utility = (1.0 - fused_belief.confidence) * 15
        confidence = fused_belief.confidence * 0.5  # Lower confidence for observation
        return utility, confidence

    def _default_utility(self, fused_belief: FusedBelief, context: Dict[str, Any]) -> Tuple[float, float]:
        """Default utility function."""
        utility = 0.0
        confidence = fused_belief.confidence
        return utility, confidence

    def _collect_evidence(self, action: str, fused_belief: FusedBelief,
                         context: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Collect evidence supporting the decision."""
        evidence = [
            {
                'type': 'fused_belief',
                'confidence': fused_belief.confidence,
                'state': fused_belief.state_estimate.tolist(),
                'uncertainty': fused_belief.uncertainty.diagonal().tolist()
            },
            {
                'type': 'context',
                'gripper_state': context.get('gripper_state', 'unknown'),
                'obstacle_detected': context.get('obstacle_detected', False),
                'goal_distance': context.get('goal_distance', 'unknown')
            }
        ]
        return evidence


class MultimodalFusionSystem:
    """Main system for multimodal fusion and decision making."""

    def __init__(self):
        self.bayesian_fusion = BayesianFusion()
        self.attention_mechanism = AttentionMechanism(num_modalities=4)  # vision, audio, tactile, lidar
        self.decision_maker = DecisionMaker()

    def process_observations(self, observations: List[SensorObservation],
                           context: Dict[str, Any]) -> Dict[str, Any]:
        """Process multimodal observations and make decisions."""
        # Fuse observations
        fused_belief = self.bayesian_fusion.fuse_observations(observations)

        # Apply attention mechanism to weight modalities
        modality_features = [obs.data for obs in observations if len(obs.data) > 0]
        if modality_features:
            attention_weights = self.attention_mechanism.compute_attention_weights(
                modality_features
            )
        else:
            attention_weights = np.array([])

        # Make decision based on fused belief
        decision = self.decision_maker.make_decision(fused_belief, context)

        return {
            'fused_belief': fused_belief,
            'attention_weights': attention_weights,
            'decision': decision,
            'timestamp': time.time()
        }


def create_sample_observations() -> List[SensorObservation]:
    """Create sample observations from different modalities."""
    timestamp = time.time()

    # Vision observation (detecting an object)
    vision_data = np.array([0.8, 0.6, 0.0])  # 3D position estimate
    vision_obs = SensorObservation(
        modality='vision',
        data=vision_data,
        timestamp=timestamp,
        confidence=0.8,
        uncertainty=0.2
    )

    # Audio observation (hearing a sound)
    audio_data = np.array([0.9, 0.5, 0.1])  # Direction estimate
    audio_obs = SensorObservation(
        modality='audio',
        data=audio_data,
        timestamp=timestamp,
        confidence=0.6,
        uncertainty=0.3
    )

    # Tactile observation (touch sensor)
    tactile_data = np.array([0.0, 0.0, 0.0])  # No contact
    tactile_obs = SensorObservation(
        modality='tactile',
        data=tactile_data,
        timestamp=timestamp,
        confidence=0.9,
        uncertainty=0.05
    )

    # LIDAR observation (distance measurement)
    lidar_data = np.array([0.75, 0.55, 0.05])  # Precise distance
    lidar_obs = SensorObservation(
        modality='lidar',
        data=lidar_data,
        timestamp=timestamp,
        confidence=0.95,
        uncertainty=0.1
    )

    return [vision_obs, audio_obs, tactile_obs, lidar_obs]


def main():
    print("Multimodal Fusion and Decision Making Demo - Chapter 23")
    print("=" * 60)
    print("This demo illustrates multimodal fusion concepts:")
    print("- Sensor data fusion from multiple modalities")
    print("- Decision making under uncertainty")
    print("- Bayesian inference for multimodal reasoning")
    print("- Attention mechanisms for modality weighting")
    print("- Uncertainty quantification and propagation")
    print()

    # Initialize the multimodal fusion system
    fusion_system = MultimodalFusionSystem()

    # Create sample observations
    observations = create_sample_observations()
    print(f"Created {len(observations)} sample observations:")
    for obs in observations:
        print(f"  - {obs.modality}: {obs.data} (confidence: {obs.confidence:.2f})")
    print()

    # Define context for decision making
    context = {
        'gripper_state': 'open',
        'obstacle_detected': False,
        'goal_distance': 2.0
    }

    # Process observations and make decisions
    print("Processing observations with multimodal fusion system...")
    result = fusion_system.process_observations(observations, context)

    # Display results
    fused_belief = result['fused_belief']
    decision = result['decision']

    print(f"Fused belief state: {fused_belief.state_estimate}")
    print(f"Fused belief confidence: {fused_belief.confidence:.2f}")
    print(f"Uncertainty: {np.diag(fused_belief.uncertainty)}")
    print(f"Source modalities: {fused_belief.source_modalities}")
    print()

    print(f"Decision made: {decision.action}")
    print(f"Decision confidence: {decision.confidence:.2f}")
    print(f"Expected utility: {decision.expected_utility:.2f}")
    print()

    if len(result['attention_weights']) > 0:
        print(f"Attention weights: {result['attention_weights']}")
    print()

    # Demonstrate with different contexts
    print("Testing with different contexts:")
    contexts = [
        {'gripper_state': 'open', 'obstacle_detected': False, 'goal_distance': 0.2},  # Close to goal
        {'gripper_state': 'closed', 'obstacle_detected': True, 'goal_distance': 2.0},  # Obstacle detected
        {'gripper_state': 'open', 'obstacle_detected': False, 'goal_distance': 5.0},  # Far from goal
    ]

    for i, ctx in enumerate(contexts):
        print(f"Context {i+1}: {ctx}")
        result = fusion_system.process_observations(observations, ctx)
        decision = result['decision']
        print(f"  Decision: {decision.action} (utility: {decision.expected_utility:.2f})")
    print()

    # Show system components
    print("Multimodal Fusion System Components:")
    print("- Bayesian Fusion: Combines information from multiple sensors")
    print("- Kalman Filters: Tracks estimates for each modality over time")
    print("- Attention Mechanism: Weights modalities based on relevance")
    print("- Decision Maker: Selects actions based on fused information")
    print("- Uncertainty Propagation: Tracks confidence in estimates")
    print()

    print("Key Multimodal Fusion Concepts Demonstrated:")
    print("1. Sensor Fusion: Combining data from different modalities")
    print("2. Bayesian Reasoning: Updating beliefs based on evidence")
    print("3. Attention Mechanisms: Focusing on relevant information")
    print("4. Uncertainty Quantification: Tracking confidence in estimates")
    print("5. Decision Making: Selecting actions under uncertainty")

    print(f"\nDemo completed. Processed observations and made decisions.")


if __name__ == "__main__":
    main()