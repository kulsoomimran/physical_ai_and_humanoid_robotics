# ROS 2 Nodes and Topics Examples

This directory contains example code for Chapter 3: ROS 2 Nodes and Topics.

## Examples

### Node and Topic Example
Demonstrates creating nodes and using topics for communication:
- `node_topic_example.py`: Python implementation showing publisher and subscriber nodes
- `node_topic_example.cpp`: C++ implementation showing publisher and subscriber nodes

## Running the Examples

### Python Version

1. Make sure you have ROS 2 installed and sourced.

2. To run the Python example:
```bash
python3 node_topic_example.py
```

This will create both a publisher and subscriber node in the same process.

### C++ Version

1. Make sure you have ROS 2 installed and sourced.

2. Create a workspace and package:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake node_topic_examples
cd node_topic_examples
```

3. Copy the `node_topic_example.cpp` file to the `src` directory of your package.

4. Update the CMakeLists.txt to build the executable:
```cmake
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(node_topic_example src/node_topic_example.cpp)
ament_target_dependencies(node_topic_example rclcpp std_msgs)

install(TARGETS
  node_topic_example
  DESTINATION lib/${PROJECT_NAME}
)
```

5. Build and run:
```bash
cd ~/ros2_ws
colcon build --packages-select node_topic_examples
source install/setup.bash
ros2 run node_topic_examples node_topic_example
```

The examples demonstrate:
- Creating publisher and subscriber nodes
- Topic-based communication
- QoS profile configuration
- Proper node lifecycle management