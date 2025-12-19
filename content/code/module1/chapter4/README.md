# ROS 2 Services and Actions Examples

This directory contains example code for Chapter 4: ROS 2 Services and Actions.

## Examples

### Simple Service Example
Demonstrates creating a ROS 2 service server and client:
- `simple_service.py`: Python implementation showing service server and client
- `simple_service.cpp`: C++ implementation showing service server and client

### Simple Action Example
Demonstrates creating a ROS 2 action server and client:
- `simple_action.py`: Python implementation showing action server and client

## Running the Examples

### Python Version

1. Make sure you have ROS 2 installed and sourced.

2. To run the simple service example:
```bash
python3 simple_service.py
```

3. To run the simple action example:
```bash
python3 simple_action.py
```

### C++ Version

1. Make sure you have ROS 2 installed and sourced.

2. Create a workspace and package:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake simple_service_example
cd simple_service_example
```

3. Copy the `simple_service.cpp` file to the `src` directory of your package.

4. Update the CMakeLists.txt to build the executable:
```cmake
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(simple_service_cpp src/simple_service.cpp)
ament_target_dependencies(simple_service_cpp rclcpp example_interfaces)

install(TARGETS
  simple_service_cpp
  DESTINATION lib/${PROJECT_NAME}
)
```

5. Build and run:
```bash
cd ~/ros2_ws
colcon build --packages-select simple_service_example
source install/setup.bash
ros2 run simple_service_example simple_service_cpp
```

The examples demonstrate:
- Service-based request-response communication
- Action-based long-running operations with feedback
- Proper error handling and resource management
- Synchronous and asynchronous operation patterns