# ROS 2 Packages and Launch Files Examples

This directory contains example code for Chapter 5: ROS 2 Packages and Launch Files.

## Examples

### Simple Package Example
Demonstrates creating a ROS 2 package with:
- `package.xml`: Package metadata and dependencies
- `CMakeLists.txt`: Build configuration for C++ code
- `setup.py`: Setup configuration for Python code
- `simple_talker.cpp`: C++ node example
- `simple_listener.py`: Python node example

### Launch File Examples
Demonstrates different launch file formats:
- `simple_launch.py`: Python launch file
- `simple_launch.xml`: XML launch file
- `simple_launch.yaml`: YAML launch file

## Running the Examples

### Creating and Building the Package

1. Make sure you have ROS 2 installed and sourced.

2. Create a workspace and copy the simple_package:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# Copy the simple_package directory here
```

3. Update the CMakeLists.txt to build the executable:
```cmake
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(simple_talker src/simple_talker.cpp)
ament_target_dependencies(simple_talker rclcpp std_msgs)

install(TARGETS
  simple_talker
  DESTINATION lib/${PROJECT_NAME}
)
```

4. Build and run:
```bash
cd ~/ros2_ws
colcon build --packages-select simple_package
source install/setup.bash
```

### Running Launch Files

After building, you can run the launch files:

Python launch file:
```bash
ros2 launch simple_package simple_launch.py
```

XML launch file:
```bash
ros2 launch simple_package simple_launch.xml
```

YAML launch file:
```bash
ros2 launch simple_package simple_launch.yaml
```

You can also pass arguments to the launch files:
```bash
ros2 launch simple_package simple_launch.py namespace:=my_robot
```

## Package Structure
```
simple_package/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── setup.cfg
├── src/
│   └── simple_talker.cpp
├── simple_listener.py
└── launch/
    ├── simple_launch.py
    ├── simple_launch.xml
    └── simple_launch.yaml
```

The examples demonstrate:
- Package organization and metadata
- CMake build system configuration
- Python package setup
- Launch file formats (XML, YAML, Python)
- Node parameters and namespaces
- Topic remapping in launch files