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
Language Processing and Command Interpretation Demo for Chapter 21: Language Processing and Command Interpretation.

This example demonstrates language processing concepts including:
- Natural language understanding
- Command parsing and semantic analysis
- Intent recognition and entity extraction
- Language-to-action mapping
"""

import numpy as np
import time
import re
from dataclasses import dataclass
from typing import List, Dict, Any, Optional, Tuple
import json
import math


@dataclass
class LanguageCommand:
    """Represents a natural language command."""
    text: str
    timestamp: float
    confidence: float = 1.0


@dataclass
class ParsedCommand:
    """Represents a parsed language command with semantic structure."""
    original_text: str
    intent: str
    entities: Dict[str, Any]  # Named entities extracted from the command
    action_verb: str
    target_objects: List[str]
    attributes: Dict[str, Any]
    confidence: float


@dataclass
class SemanticFrame:
    """Represents the semantic structure of a command."""
    action: str
    agent: str  # Who performs the action (robot, user, etc.)
    patient: str  # What is affected by the action
    instrument: str  # What is used to perform the action
    location: str  # Where the action takes place
    time: str  # When the action occurs


class Vocabulary:
    """Manages vocabulary for language processing."""

    def __init__(self):
        # Action verbs
        self.action_verbs = {
            'move', 'go', 'navigate', 'walk', 'drive', 'roll', 'step',
            'pick', 'grasp', 'grab', 'take', 'lift', 'get', 'catch',
            'place', 'put', 'set', 'drop', 'release', 'position',
            'push', 'pull', 'press', 'touch', 'press',
            'look', 'see', 'observe', 'watch', 'examine', 'inspect',
            'find', 'locate', 'search', 'seek', 'detect',
            'follow', 'chase', 'pursue', 'accompany',
            'stop', 'wait', 'pause', 'hold', 'freeze',
            'turn', 'rotate', 'spin', 'pivot', 'face',
            'open', 'close', 'lift', 'lower', 'raise'
        }

        # Spatial relations
        self.spatial_relations = {
            'near', 'close', 'far', 'left', 'right', 'front', 'back',
            'behind', 'in front of', 'beside', 'next to', 'between',
            'above', 'below', 'under', 'over', 'on', 'at', 'to'
        }

        # Object types
        self.object_types = {
            'box', 'cube', 'sphere', 'cylinder', 'object', 'item',
            'person', 'human', 'robot', 'agent', 'table', 'chair',
            'shelf', 'cabinet', 'door', 'window', 'wall', 'floor',
            'book', 'bottle', 'cup', 'plate', 'fork', 'knife',
            'apple', 'banana', 'orange', 'food', 'snack'
        }

        # Colors
        self.colors = {
            'red', 'blue', 'green', 'yellow', 'orange', 'purple',
            'pink', 'brown', 'black', 'white', 'gray', 'grey'
        }

        # Sizes
        self.sizes = {
            'small', 'large', 'big', 'tiny', 'huge', 'massive',
            'mini', 'compact', 'giant', 'enormous', 'little'
        }

        # Shapes
        self.shapes = {
            'round', 'square', 'rectangular', 'circular', 'triangular',
            'cylindrical', 'spherical', 'conical', 'pyramidal'
        }


class Tokenizer:
    """Simple tokenizer for language commands."""

    def __init__(self):
        self.vocabulary = Vocabulary()

    def tokenize(self, text: str) -> List[str]:
        """Tokenize text into meaningful units."""
        # Convert to lowercase and split by whitespace and punctuation
        text = text.lower()
        # Split on whitespace and common punctuation
        tokens = re.split(r'[,\.\!\?\;\:\s]+', text)
        # Remove empty tokens
        tokens = [token.strip() for token in tokens if token.strip()]
        return tokens

    def identify_token_types(self, tokens: List[str]) -> List[Tuple[str, str]]:
        """Identify the type of each token."""
        token_types = []
        vocab = self.vocabulary

        for token in tokens:
            if token in vocab.action_verbs:
                token_types.append((token, 'ACTION'))
            elif token in vocab.spatial_relations:
                token_types.append((token, 'SPATIAL'))
            elif token in vocab.object_types:
                token_types.append((token, 'OBJECT'))
            elif token in vocab.colors:
                token_types.append((token, 'COLOR'))
            elif token in vocab.sizes:
                token_types.append((token, 'SIZE'))
            elif token in vocab.shapes:
                token_types.append((token, 'SHAPE'))
            else:
                # Check if it's a number
                try:
                    float(token)
                    token_types.append((token, 'NUMBER'))
                except ValueError:
                    token_types.append((token, 'OTHER'))

        return token_types


class SemanticParser:
    """Parses natural language commands into semantic structures."""

    def __init__(self):
        self.tokenizer = Tokenizer()
        self.vocabulary = Vocabulary()

    def parse_command(self, command: LanguageCommand) -> ParsedCommand:
        """Parse a language command into its semantic components."""
        tokens = self.tokenizer.tokenize(command.text)
        token_types = self.tokenizer.identify_token_types(tokens)

        # Extract action verb (first verb in the command)
        action_verb = ""
        for token, token_type in token_types:
            if token_type == 'ACTION':
                action_verb = token
                break

        # Extract entities
        entities = self.extract_entities(tokens, token_types)

        # Determine intent based on action verb
        intent = self.determine_intent(action_verb)

        # Extract target objects
        target_objects = self.extract_target_objects(tokens, token_types)

        # Extract attributes
        attributes = self.extract_attributes(tokens, token_types)

        return ParsedCommand(
            original_text=command.text,
            intent=intent,
            entities=entities,
            action_verb=action_verb,
            target_objects=target_objects,
            attributes=attributes,
            confidence=command.confidence
        )

    def extract_entities(self, tokens: List[str], token_types: List[Tuple[str, str]]) -> Dict[str, Any]:
        """Extract named entities from the command."""
        entities = {}

        # Extract colors
        colors = [token for token, token_type in token_types if token_type == 'COLOR']
        if colors:
            entities['colors'] = colors

        # Extract sizes
        sizes = [token for token, token_type in token_types if token_type == 'SIZE']
        if sizes:
            entities['sizes'] = sizes

        # Extract objects
        objects = [token for token, token_type in token_types if token_type == 'OBJECT']
        if objects:
            entities['objects'] = objects

        # Extract spatial relations
        spatial = [token for token, token_type in token_types if token_type == 'SPATIAL']
        if spatial:
            entities['spatial_relations'] = spatial

        return entities

    def determine_intent(self, action_verb: str) -> str:
        """Determine the intent of the command based on the action verb."""
        if action_verb in ['move', 'go', 'navigate', 'walk', 'drive', 'roll', 'step']:
            return 'NAVIGATION'
        elif action_verb in ['pick', 'grasp', 'grab', 'take', 'lift', 'get', 'catch']:
            return 'GRASPING'
        elif action_verb in ['place', 'put', 'set', 'drop', 'release', 'position']:
            return 'PLACING'
        elif action_verb in ['push', 'pull', 'press', 'touch']:
            return 'MANIPULATION'
        elif action_verb in ['look', 'see', 'observe', 'watch', 'examine', 'inspect']:
            return 'PERCEPTION'
        elif action_verb in ['find', 'locate', 'search', 'seek', 'detect']:
            return 'SEARCH'
        elif action_verb in ['follow', 'chase', 'pursue', 'accompany']:
            return 'FOLLOWING'
        elif action_verb in ['stop', 'wait', 'pause', 'hold', 'freeze']:
            return 'STOP'
        elif action_verb in ['turn', 'rotate', 'spin', 'pivot', 'face']:
            return 'ROTATION'
        elif action_verb in ['open', 'close', 'lift', 'lower', 'raise']:
            return 'DOOR_CONTROL'
        else:
            return 'UNKNOWN'

    def extract_target_objects(self, tokens: List[str], token_types: List[Tuple[str, str]]) -> List[str]:
        """Extract target objects from the command."""
        target_objects = []

        for i, (token, token_type) in enumerate(token_types):
            if token_type == 'OBJECT':
                target_objects.append(token)

        return target_objects

    def extract_attributes(self, tokens: List[str], token_types: List[Tuple[str, str]]) -> Dict[str, Any]:
        """Extract attributes from the command."""
        attributes = {}

        # Extract colors
        colors = [token for token, token_type in token_types if token_type == 'COLOR']
        if colors:
            attributes['colors'] = colors

        # Extract sizes
        sizes = [token for token, token_type in token_types if token_type == 'SIZE']
        if sizes:
            attributes['sizes'] = sizes

        # Extract shapes
        shapes = [token for token, token_type in token_types if token_type == 'SHAPE']
        if shapes:
            attributes['shapes'] = shapes

        return attributes

    def create_semantic_frame(self, parsed_command: ParsedCommand) -> SemanticFrame:
        """Create a semantic frame from the parsed command."""
        # Simple heuristics to extract semantic roles
        action = parsed_command.action_verb
        agent = "robot"  # Default agent is the robot
        patient = " ".join(parsed_command.target_objects[:1]) if parsed_command.target_objects else "object"
        instrument = ""  # Not typically specified in simple commands
        location = " ".join(parsed_command.entities.get('spatial_relations', [])[:1]) if 'spatial_relations' in parsed_command.entities else ""
        time = ""  # Not typically specified in simple commands

        return SemanticFrame(
            action=action,
            agent=agent,
            patient=patient,
            instrument=instrument,
            location=location,
            time=time
        )


class LanguageGrounding:
    """Grounds language commands in the physical world."""

    def __init__(self):
        self.semantic_parser = SemanticParser()

    def ground_command(self, command: LanguageCommand, world_state: Dict[str, Any]) -> Dict[str, Any]:
        """Ground a language command in the current world state."""
        parsed_command = self.semantic_parser.parse_command(command)
        semantic_frame = self.semantic_parser.create_semantic_frame(parsed_command)

        # Resolve ambiguous references based on world state
        resolved_objects = self.resolve_objects(parsed_command, world_state)

        # Create grounded command
        grounded_command = {
            'original_command': command,
            'parsed_command': parsed_command,
            'semantic_frame': semantic_frame,
            'resolved_objects': resolved_objects,
            'grounding_confidence': self.calculate_grounding_confidence(parsed_command, resolved_objects)
        }

        return grounded_command

    def resolve_objects(self, parsed_command: ParsedCommand, world_state: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Resolve object references in the command based on world state."""
        resolved_objects = []

        # Get objects from world state
        world_objects = world_state.get('objects', [])
        target_names = parsed_command.target_objects
        colors = parsed_command.entities.get('colors', [])
        sizes = parsed_command.entities.get('sizes', [])

        for obj in world_objects:
            obj_name = obj.get('name', '').lower()

            # Check if this object matches any target names
            name_match = any(target_name in obj_name or obj_name in target_name for target_name in target_names)

            # Check if this object matches any specified colors
            color_match = True  # If no colors specified, all match
            if colors:
                obj_color = obj.get('color', '').lower()
                color_match = obj_color in colors

            # Check if this object matches any specified sizes
            size_match = True  # If no sizes specified, all match
            if sizes:
                obj_size = obj.get('size', '').lower()
                size_match = obj_size in sizes

            # If all conditions match, add to resolved objects
            if name_match and color_match and size_match:
                resolved_objects.append(obj)

        return resolved_objects

    def calculate_grounding_confidence(self, parsed_command: ParsedCommand, resolved_objects: List[Dict[str, Any]]) -> float:
        """Calculate confidence in the grounding."""
        if not parsed_command.target_objects:
            # If no target objects specified, confidence is lower
            return 0.5

        if not resolved_objects:
            # If no objects resolved, confidence is low
            return 0.2

        # Calculate confidence based on specificity of the command
        specificity_score = 0.5  # Base score

        # Increase for each specified attribute
        if parsed_command.entities.get('colors'):
            specificity_score += 0.2
        if parsed_command.entities.get('sizes'):
            specificity_score += 0.2
        if parsed_command.entities.get('shapes'):
            specificity_score += 0.2

        # Cap at 1.0
        specificity_score = min(1.0, specificity_score)

        # Combine with number of resolved objects (more specific if only one matches)
        if len(resolved_objects) == 1:
            return min(1.0, specificity_score * 1.2)
        elif len(resolved_objects) > 1:
            # Lower confidence if multiple objects match
            return max(0.3, specificity_score * 0.7)
        else:
            return 0.2


class CommandExecutor:
    """Executes grounded language commands."""

    def __init__(self):
        self.grounding_system = LanguageGrounding()

    def execute_command(self, command: LanguageCommand, world_state: Dict[str, Any]) -> Dict[str, Any]:
        """Execute a language command in the world."""
        # Ground the command
        grounded_command = self.grounding_system.ground_command(command, world_state)

        # Generate action plan based on the grounded command
        action_plan = self.generate_action_plan(grounded_command)

        # Execute the action plan
        execution_result = self.execute_action_plan(action_plan, world_state)

        return {
            'command': command,
            'grounded_command': grounded_command,
            'action_plan': action_plan,
            'execution_result': execution_result,
            'timestamp': time.time()
        }

    def generate_action_plan(self, grounded_command: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Generate an action plan from the grounded command."""
        parsed_command = grounded_command['parsed_command']
        intent = parsed_command.intent
        resolved_objects = grounded_command['resolved_objects']

        action_plan = []

        if intent == 'NAVIGATION':
            if resolved_objects:
                for obj in resolved_objects[:1]:  # Navigate to first resolved object
                    action_plan.append({
                        'action_type': 'navigate',
                        'target_position': obj.get('position', [0, 0, 0]),
                        'description': f"Navigate to {obj.get('name', 'object')}"
                    })
            else:
                # If no specific object, move in the direction specified
                action_plan.append({
                    'action_type': 'navigate',
                    'target_position': [1, 0, 0],  # Move 1 meter in x direction
                    'description': "Move forward"
                })

        elif intent == 'GRASPING':
            if resolved_objects:
                for obj in resolved_objects[:1]:  # Grasp first resolved object
                    action_plan.append({
                        'action_type': 'grasp',
                        'target_object': obj.get('name', 'object'),
                        'target_position': obj.get('position', [0, 0, 0]),
                        'description': f"Grasp {obj.get('name', 'object')}"
                    })
            else:
                action_plan.append({
                    'action_type': 'grasp',
                    'description': "Grasp object at current position",
                    'target_position': [0, 0, 0]
                })

        elif intent == 'PLACING':
            if resolved_objects:
                for obj in resolved_objects[:1]:  # Place at first resolved object
                    action_plan.append({
                        'action_type': 'place',
                        'target_position': obj.get('position', [0, 0, 0]),
                        'description': f"Place object at {obj.get('name', 'object')}"
                    })
            else:
                action_plan.append({
                    'action_type': 'place',
                    'description': "Place object at current position",
                    'target_position': [0, 0, 0]
                })

        elif intent == 'PERCEPTION':
            action_plan.append({
                'action_type': 'observe',
                'description': "Observe the environment"
            })

        elif intent == 'SEARCH':
            if resolved_objects:
                for obj in resolved_objects[:1]:  # Search for first resolved object
                    action_plan.append({
                        'action_type': 'search',
                        'target_object': obj.get('name', 'object'),
                        'description': f"Search for {obj.get('name', 'object')}"
                    })
            else:
                action_plan.append({
                    'action_type': 'search',
                    'description': "Search for object"
                })

        else:
            action_plan.append({
                'action_type': 'unknown',
                'description': f"Unknown command intent: {intent}"
            })

        return action_plan

    def execute_action_plan(self, action_plan: List[Dict[str, Any]], world_state: Dict[str, Any]) -> Dict[str, Any]:
        """Execute the action plan."""
        results = []

        for action in action_plan:
            # Simulate action execution
            result = {
                'action': action,
                'status': 'completed',  # In simulation, all actions succeed
                'timestamp': time.time()
            }
            results.append(result)

        return {
            'actions_executed': len(results),
            'results': results,
            'world_state_after': world_state  # In simulation, world state doesn't change
        }


def create_sample_world_state() -> Dict[str, Any]:
    """Create a sample world state for demonstration."""
    return {
        'objects': [
            {
                'name': 'red box',
                'position': [1.0, 0.5, 0.0],
                'color': 'red',
                'size': 'medium',
                'type': 'box'
            },
            {
                'name': 'blue sphere',
                'position': [2.0, 1.0, 0.0],
                'color': 'blue',
                'size': 'small',
                'type': 'sphere'
            },
            {
                'name': 'large green cube',
                'position': [0.5, 1.5, 0.0],
                'color': 'green',
                'size': 'large',
                'type': 'cube'
            },
            {
                'name': 'small yellow cylinder',
                'position': [1.5, 0.0, 0.0],
                'color': 'yellow',
                'size': 'small',
                'type': 'cylinder'
            }
        ],
        'robot_position': [0.0, 0.0, 0.0],
        'robot_orientation': [0.0, 0.0, 0.0, 1.0]  # quaternion
    }


def main():
    print("Language Processing and Command Interpretation Demo - Chapter 21")
    print("=" * 65)
    print("This demo illustrates language processing concepts:")
    print("- Natural language understanding and parsing")
    print("- Command interpretation and semantic analysis")
    print("- Language-to-action mapping")
    print("- Grounding language in the physical world")
    print()

    # Initialize the language processing system
    command_executor = CommandExecutor()

    # Create a sample world state
    world_state = create_sample_world_state()
    print("Created sample world state with objects:")
    for obj in world_state['objects']:
        print(f"  - {obj['name']} at position {obj['position']}")
    print()

    # Define test commands
    test_commands = [
        "Go to the red box",
        "Pick up the blue sphere",
        "Find the large green cube",
        "Look at the small yellow cylinder",
        "Place object near the red box"
    ]

    print("Processing language commands...")
    print()

    for i, cmd_text in enumerate(test_commands):
        print(f"Command {i+1}: '{cmd_text}'")

        # Create language command
        lang_cmd = LanguageCommand(
            text=cmd_text,
            timestamp=time.time(),
            confidence=0.9
        )

        # Execute the command
        result = command_executor.execute_command(lang_cmd, world_state)

        # Display results
        parsed_cmd = result['grounded_command']['parsed_command']
        print(f"  Intent: {parsed_cmd.intent}")
        print(f"  Action verb: {parsed_cmd.action_verb}")
        print(f"  Target objects: {parsed_cmd.target_objects}")
        print(f"  Entities: {parsed_cmd.entities}")

        if result['grounded_command']['resolved_objects']:
            print(f"  Resolved objects: {[obj['name'] for obj in result['grounded_command']['resolved_objects']]}")
        else:
            print(f"  Resolved objects: None")

        print(f"  Grounding confidence: {result['grounded_command']['grounding_confidence']:.2f}")

        print(f"  Action plan:")
        for action in result['action_plan']:
            print(f"    - {action['description']}")

        print()

    # Demonstrate the language processing pipeline
    print("Language Processing Pipeline Components:")
    print("- Tokenizer: Breaks down text into meaningful units")
    print("- Semantic Parser: Extracts intent and entities from commands")
    print("- Language Grounding: Connects language to physical world")
    print("- Command Executor: Translates commands into actions")
    print()

    print("Key Language Processing Concepts Demonstrated:")
    print("1. Intent Recognition: Identifying the purpose of a command")
    print("2. Entity Extraction: Finding objects, attributes, and relationships")
    print("3. Semantic Parsing: Converting text to structured meaning")
    print("4. Language Grounding: Connecting words to physical objects")
    print("5. Action Planning: Generating executable plans from commands")

    print(f"\nDemo completed. Processed {len(test_commands)} language commands.")


if __name__ == "__main__":
    main()