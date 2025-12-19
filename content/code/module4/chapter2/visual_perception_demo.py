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
Visual Perception and Understanding Demo for Chapter 20: Visual Perception and Understanding.

This example demonstrates visual perception concepts including:
- Feature detection and extraction
- Object recognition and classification
- Scene understanding and segmentation
- Depth estimation and 3D reconstruction
"""

import numpy as np
import cv2
import time
from dataclasses import dataclass
from typing import List, Dict, Any, Optional, Tuple
import random
import math


@dataclass
class ImageObservation:
    """Represents an image observation from a camera."""
    rgb_image: np.ndarray  # RGB image data
    depth_map: Optional[np.ndarray]  # Depth information (optional)
    timestamp: float
    camera_intrinsics: Dict[str, float]  # Camera intrinsic parameters


@dataclass
class DetectedObject:
    """Represents a detected object in the scene."""
    name: str
    confidence: float
    bounding_box: Tuple[int, int, int, int]  # (x, y, width, height)
    center_2d: Tuple[int, int]  # 2D center coordinates
    center_3d: Optional[Tuple[float, float, float]]  # 3D center coordinates (if depth available)
    features: List[float]  # Feature vector for the object


@dataclass
class SceneUnderstanding:
    """Represents the understood scene."""
    objects: List[DetectedObject]
    segmentation_mask: Optional[np.ndarray]  # Semantic segmentation mask
    scene_description: str  # Natural language description of the scene
    timestamp: float


class FeatureDetector:
    """Detects and extracts features from images."""

    def __init__(self):
        # Initialize feature detection parameters
        self.feature_params = dict(
            maxCorners=100,
            qualityLevel=0.3,
            minDistance=7,
            blockSize=7
        )

    def detect_features(self, image: np.ndarray) -> List[Tuple[int, int]]:
        """Detect feature points in the image."""
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

        # Convert to float32 for corner detection
        gray = np.float32(gray)

        # Detect corners
        corners = cv2.goodFeaturesToTrack(gray, **self.feature_params)

        if corners is not None:
            corners = np.int0(corners)
            return [(corner[0][0], corner[0][1]) for corner in corners]
        else:
            return []

    def extract_sift_features(self, image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Extract SIFT-like features from the image."""
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

        # For simulation, we'll create a simple feature extraction
        # In real applications, we would use actual SIFT or other descriptors
        features = []
        keypoints = []

        # Create a grid of potential keypoints
        height, width = gray.shape
        step = 32  # Sample every 32 pixels

        for y in range(0, height, step):
            for x in range(0, width, step):
                if x < width and y < height:
                    # Extract a small patch around the keypoint
                    patch_size = 16
                    y_end = min(y + patch_size, height)
                    x_end = min(x + patch_size, width)

                    patch = gray[y:y_end, x:x_end]
                    # Simple feature descriptor based on patch statistics
                    feature = [
                        np.mean(patch),
                        np.std(patch),
                        np.var(patch),
                        np.min(patch),
                        np.max(patch)
                    ]

                    features.append(feature)
                    keypoints.append((x, y))

        return np.array(keypoints), np.array(features)


class ObjectDetector:
    """Detects and classifies objects in images."""

    def __init__(self):
        # Define a simple object vocabulary for simulation
        self.object_classes = [
            'person', 'car', 'bicycle', 'motorcycle', 'airplane', 'bus',
            'train', 'truck', 'boat', 'traffic light', 'fire hydrant',
            'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog',
            'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe',
            'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
            'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat',
            'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
            'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl',
            'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot',
            'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
            'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop',
            'mouse', 'remote', 'keyboard', 'cell phone', 'microwave',
            'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock',
            'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
        ]

        # Create simple class features for simulation
        self.class_features = {}
        for i, class_name in enumerate(self.object_classes):
            # Create a unique feature vector for each class
            self.class_features[class_name] = np.random.rand(128) * (i + 1) * 0.1

    def detect_objects(self, image: np.ndarray) -> List[DetectedObject]:
        """Detect objects in the image."""
        height, width = image.shape[:2]
        detected_objects = []

        # For simulation, we'll randomly place some objects in the image
        num_objects = random.randint(1, 5)

        for _ in range(num_objects):
            # Randomly select a class
            class_name = random.choice(self.object_classes)

            # Random bounding box
            box_width = random.randint(30, 200)
            box_height = random.randint(30, 200)
            x = random.randint(0, width - box_width)
            y = random.randint(0, height - box_height)

            # Random confidence
            confidence = random.uniform(0.5, 0.95)

            # Calculate center
            center_2d = (x + box_width // 2, y + box_height // 2)

            # Create feature vector based on class
            features = self.class_features[class_name].tolist()

            detected_obj = DetectedObject(
                name=class_name,
                confidence=confidence,
                bounding_box=(x, y, box_width, box_height),
                center_2d=center_2d,
                center_3d=None,  # Will be filled in if depth is available
                features=features
            )

            detected_objects.append(detected_obj)

        return detected_objects


class DepthEstimator:
    """Estimates depth from images."""

    def __init__(self):
        self.depth_range = (0.1, 10.0)  # meters

    def estimate_depth(self, image: np.ndarray) -> np.ndarray:
        """Estimate depth map from a single image."""
        height, width = image.shape[:2]

        # For simulation, create a synthetic depth map
        # In reality, this would use stereo vision, structured light, or learning-based methods
        depth_map = np.zeros((height, width), dtype=np.float32)

        # Create some depth variation based on image content
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY).astype(np.float32)

        # Add some noise and variation
        noise = np.random.normal(0, 0.1, (height, width)).astype(np.float32)
        depth_map = 5.0 + gray * 0.01 + noise

        # Clamp to valid range
        depth_map = np.clip(depth_map, self.depth_range[0], self.depth_range[1])

        return depth_map

    def estimate_3d_position(self, center_2d: Tuple[int, int], depth_value: float,
                           camera_intrinsics: Dict[str, float]) -> Tuple[float, float, float]:
        """Estimate 3D position from 2D coordinates and depth."""
        x_2d, y_2d = center_2d
        fx = camera_intrinsics['fx']
        fy = camera_intrinsics['fy']
        cx = camera_intrinsics['cx']
        cy = camera_intrinsics['cy']

        # Convert 2D pixel coordinates to 3D world coordinates
        x_3d = (x_2d - cx) * depth_value / fx
        y_3d = (y_2d - cy) * depth_value / fy
        z_3d = depth_value

        return (x_3d, y_3d, z_3d)


class SemanticSegmenter:
    """Performs semantic segmentation on images."""

    def __init__(self):
        # Define color mapping for different classes
        self.class_colors = {
            'background': [0, 0, 0],
            'person': [220, 20, 60],      # Red
            'car': [0, 0, 142],           # Dark blue
            'bicycle': [128, 64, 128],    # Purple
            'motorcycle': [0, 0, 230],    # Blue
            'airplane': [128, 128, 0],    # Olive
            'bus': [0, 60, 100],          # Dark green-blue
            'train': [0, 80, 100],        # Green-blue
            'truck': [0, 0, 70],          # Dark blue
            'traffic light': [250, 170, 30],  # Yellow
            'stop sign': [255, 0, 0],     # Bright red
            'parking meter': [102, 102, 156], # Gray-blue
            'bench': [190, 153, 153],     # Gray
            'bird': [153, 255, 153],      # Light green
            'cat': [128, 128, 128],       # Gray
            'dog': [255, 128, 0],         # Orange
            'horse': [255, 153, 153],     # Light red
            'sheep': [255, 153, 153],     # Light red
            'cow': [60, 60, 60],          # Dark gray
            'chair': [250, 170, 30],      # Yellow
            'couch': [255, 0, 0],         # Red
            'potted plant': [107, 142, 35], # Green
            'bed': [152, 251, 152],       # Light green
            'dining table': [70, 130, 180], # Steel blue
            'toilet': [220, 220, 0],      # Yellow-green
            'tv': [90, 20, 120],          # Purple
        }

    def segment_image(self, image: np.ndarray, detected_objects: List[DetectedObject]) -> np.ndarray:
        """Create a semantic segmentation mask for the image."""
        height, width = image.shape[:2]
        segmentation_mask = np.zeros((height, width), dtype=np.uint8)

        # For each detected object, create a mask
        for i, obj in enumerate(detected_objects):
            x, y, w, h = obj.bounding_box

            # Create a simple rectangular mask for the object
            # In reality, this would be more sophisticated
            mask_region = segmentation_mask[y:y+h, x:x+w]
            mask_region[:] = (i + 1) % 255  # Use different values for different objects

        return segmentation_mask


class SceneUnderstandingSystem:
    """Main system for visual perception and scene understanding."""

    def __init__(self):
        self.feature_detector = FeatureDetector()
        self.object_detector = ObjectDetector()
        self.depth_estimator = DepthEstimator()
        self.segmenter = SemanticSegmenter()

    def process_image(self, image_obs: ImageObservation) -> SceneUnderstanding:
        """Process an image observation to understand the scene."""
        # Detect features in the image
        features = self.feature_detector.detect_features(image_obs.rgb_image)

        # Detect objects in the image
        detected_objects = self.object_detector.detect_objects(image_obs.rgb_image)

        # If depth information is available, enhance object information
        if image_obs.depth_map is not None:
            for obj in detected_objects:
                # Get depth at object center
                center_x, center_y = obj.center_2d
                depth_value = image_obs.depth_map[center_y, center_x]

                # Estimate 3D position
                obj.center_3d = self.depth_estimator.estimate_3d_position(
                    obj.center_2d, depth_value, image_obs.camera_intrinsics
                )
        else:
            # Estimate depth if not provided
            estimated_depth = self.depth_estimator.estimate_depth(image_obs.rgb_image)
            for obj in detected_objects:
                center_x, center_y = obj.center_2d
                depth_value = estimated_depth[center_y, center_y]

                obj.center_3d = self.depth_estimator.estimate_3d_position(
                    obj.center_2d, depth_value, image_obs.camera_intrinsics
                )

        # Create semantic segmentation mask
        segmentation_mask = self.segmenter.segment_image(image_obs.rgb_image, detected_objects)

        # Generate a natural language description of the scene
        scene_description = self.generate_scene_description(detected_objects)

        return SceneUnderstanding(
            objects=detected_objects,
            segmentation_mask=segmentation_mask,
            scene_description=scene_description,
            timestamp=time.time()
        )

    def generate_scene_description(self, detected_objects: List[DetectedObject]) -> str:
        """Generate a natural language description of the scene."""
        if not detected_objects:
            return "The scene appears to be empty or no objects were detected."

        # Group objects by type
        object_counts = {}
        for obj in detected_objects:
            if obj.name in object_counts:
                object_counts[obj.name] += 1
            else:
                object_counts[obj.name] = 1

        # Create a description
        description_parts = []
        for obj_name, count in object_counts.items():
            if count == 1:
                description_parts.append(f"a {obj_name}")
            else:
                description_parts.append(f"{count} {obj_name}s")

        if len(description_parts) == 1:
            scene_desc = f"The scene contains {description_parts[0]}."
        elif len(description_parts) == 2:
            scene_desc = f"The scene contains {description_parts[0]} and {description_parts[1]}."
        else:
            scene_desc = f"The scene contains {', '.join(description_parts[:-1])}, and {description_parts[-1]}."

        return scene_desc


def simulate_camera_intrinsics() -> Dict[str, float]:
    """Simulate typical camera intrinsic parameters."""
    return {
        'fx': 525.0,  # Focal length in x
        'fy': 525.0,  # Focal length in y
        'cx': 319.5,  # Principal point x
        'cy': 239.5,  # Principal point y
        'width': 640, # Image width
        'height': 480 # Image height
    }


def main():
    print("Visual Perception and Understanding Demo - Chapter 20")
    print("=" * 55)
    print("This demo illustrates visual perception concepts:")
    print("- Feature detection and extraction")
    print("- Object recognition and classification")
    print("- Scene understanding and segmentation")
    print("- Depth estimation and 3D reconstruction")
    print()

    # Initialize the visual perception system
    perception_system = SceneUnderstandingSystem()

    # Simulate camera intrinsics
    camera_intrinsics = simulate_camera_intrinsics()

    # Create a simulated image (in a real system, this would come from a camera)
    print("Creating simulated image observation...")
    height, width = 480, 640
    simulated_image = np.random.randint(0, 255, (height, width, 3), dtype=np.uint8)

    # Add some recognizable patterns to the simulated image
    cv2.rectangle(simulated_image, (100, 100), (200, 200), (255, 0, 0), -1)  # Blue square
    cv2.circle(simulated_image, (300, 150), 50, (0, 255, 0), -1)  # Green circle
    cv2.rectangle(simulated_image, (400, 300), (500, 400), (0, 0, 255), -1)  # Red square

    # Create image observation
    image_obs = ImageObservation(
        rgb_image=simulated_image,
        depth_map=None,  # Will be estimated by the system
        timestamp=time.time(),
        camera_intrinsics=camera_intrinsics
    )

    print("Processing image with visual perception system...")
    print(f"Image size: {simulated_image.shape[1]}x{simulated_image.shape[0]}")
    print()

    # Process the image to understand the scene
    scene_understanding = perception_system.process_image(image_obs)

    print("Scene Understanding Results:")
    print("-" * 30)
    print(f"Detected {len(scene_understanding.objects)} objects:")

    for i, obj in enumerate(scene_understanding.objects):
        print(f"  {i+1}. {obj.name}")
        print(f"     Confidence: {obj.confidence:.2f}")
        print(f"     Bounding box: {obj.bounding_box}")
        print(f"     2D center: {obj.center_2d}")
        if obj.center_3d:
            print(f"     3D center: ({obj.center_3d[0]:.2f}, {obj.center_3d[1]:.2f}, {obj.center_3d[2]:.2f})")
        print()

    print(f"Scene Description: {scene_understanding.scene_description}")
    print()

    # Demonstrate feature detection
    print("Feature Detection Results:")
    print("-" * 25)
    features = perception_system.feature_detector.detect_features(simulated_image)
    print(f"Detected {len(features)} feature points")

    if features:
        print(f"Sample feature points: {features[:5]}{'...' if len(features) > 5 else ''}")
    print()

    # Demonstrate depth estimation
    print("Depth Estimation Results:")
    print("-" * 25)
    estimated_depth = perception_system.depth_estimator.estimate_depth(simulated_image)
    print(f"Estimated depth map shape: {estimated_depth.shape}")
    print(f"Depth range: {np.min(estimated_depth):.2f} - {np.max(estimated_depth):.2f} meters")
    print()

    # Show system components
    print("Visual Perception System Components:")
    print("- Feature Detector: Extracts distinctive points from images")
    print("- Object Detector: Recognizes and classifies objects in the scene")
    print("- Depth Estimator: Estimates depth information for 3D understanding")
    print("- Semantic Segmenter: Labels each pixel with its object class")
    print("- Scene Understanding: Combines all components for comprehensive scene analysis")
    print()

    print("Key Visual Perception Concepts Demonstrated:")
    print("1. Feature Detection: Identifying distinctive points in images")
    print("2. Object Recognition: Classifying objects in the scene")
    print("3. Scene Segmentation: Dividing the image into meaningful regions")
    print("4. Depth Estimation: Understanding 3D structure from 2D images")
    print("5. Multi-modal Integration: Combining different perception modalities")

    print(f"\nDemo completed. Processed scene with {len(scene_understanding.objects)} objects.")


if __name__ == "__main__":
    main()