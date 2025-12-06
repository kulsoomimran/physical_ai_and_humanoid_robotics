# ROS 2 Tools and Debugging Examples

This directory contains example code for Chapter 6: ROS 2 Tools and Debugging.

## Examples

### Debugging Node
Demonstrates various debugging techniques in ROS 2:
- `debugging_node.py`: Python node with extensive logging and debugging features
- `debugging_node.cpp`: C++ node with extensive logging and debugging features

### Launch File for Debugging
Demonstrates using launch files for debugging configurations:
- `debug_launch.py`: Launch file with debugging parameters and configurations

## Running the Examples

### Python Debugging Node

1. Make sure you have ROS 2 installed and sourced.

2. To run the Python debugging node with default settings:
```bash
python3 debugging_node.py
```

3. To run with custom parameters:
```bash
ros2 run <package_name> debugging_node --ros-args -p debug_level:=3 -p message_interval:=0.5 -p error_simulation:=true
```

### C++ Debugging Node

1. Create a workspace and package:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake chapter6_examples
```

2. Copy the `debugging_node.cpp` file to the `src` directory of your package.

3. Update the CMakeLists.txt to build the executable:
```cmake
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(debugging_node_cpp src/debugging_node.cpp)
ament_target_dependencies(debugging_node_cpp rclcpp std_msgs example_interfaces)

install(TARGETS
  debugging_node_cpp
  DESTINATION lib/${PROJECT_NAME}
)
```

4. Build and run:
```bash
cd ~/ros2_ws
colcon build --packages-select chapter6_examples
source install/setup.bash
ros2 run chapter6_examples debugging_node_cpp
```

### Using Launch Files for Debugging

1. To launch the debugging configuration:
```bash
ros2 launch debug_launch.py
```

2. With custom parameters:
```bash
ros2 launch debug_launch.py debug_level:=3 error_simulation:=true namespace:=test
```

## Debugging Techniques Demonstrated

### Logging and Debugging
- Proper use of different log levels (DEBUG, INFO, WARN, ERROR, FATAL)
- Parameter-based debugging configurations
- Error simulation for testing error handling
- Exception handling with traceback information

### Command-Line Tools Usage
- Using `ros2 param` to adjust parameters at runtime
- Using `ros2 topic echo` to inspect data flow
- Using `ros2 service call` to test service functionality
- Using `ros2 run` with parameter arguments

### Monitoring and Visualization
- Publishing messages for debugging
- Subscribing to topics for data inspection
- Service call monitoring and response analysis

## Common Debugging Commands

### Node and Topic Inspection
```bash
# List all nodes
ros2 node list

# Get information about a specific node
ros2 node info /debugging_node

# List all topics
ros2 topic list

# Echo messages from a topic
ros2 topic echo /debug_topic

# Get information about a topic
ros2 topic info /debug_topic
```

### Service and Parameter Debugging
```bash
# List all services
ros2 service list

# Call a service
ros2 service call /debug_add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"

# List parameters of a node
ros2 param list /debugging_node

# Get a parameter value
ros2 param get /debugging_node debug_level

# Set a parameter value
ros2 param set /debugging_node debug_level 3
```

### System Diagnostics
```bash
# Run basic system diagnostics
ros2 doctor

# Generate detailed system report
ros2 doctor --report

# Check specific extensions
ros2 doctor --show-extensions
```

## Using rqt for Debugging

### Launch rqt with useful plugins:
```bash
rqt
```

Then add plugins like:
- **rqt_graph**: Visualize node connections
- **rqt_console**: Monitor logs in real-time
- **rqt_plot**: Plot numerical values
- **rqt_reconfigure**: Dynamically adjust parameters

### Using RViz2 for Visualization
```bash
rviz2
```

RViz2 can be used to visualize:
- Robot models and TF frames
- Sensor data (laser scans, point clouds)
- Custom markers and messages

## Best Practices for Debugging

1. **Use appropriate log levels**: Don't clutter output with excessive DEBUG messages in production
2. **Parameterize debug behavior**: Use parameters to control debug features
3. **Handle errors gracefully**: Don't let debugging code cause system failures
4. **Use descriptive names**: Name topics, services, and parameters clearly
5. **Document debugging features**: Include information about debug parameters in documentation

The examples demonstrate practical debugging techniques that can be applied to real ROS 2 applications.