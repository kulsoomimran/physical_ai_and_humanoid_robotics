# ROS 2 Architecture and Communication Examples

This directory contains example code for Chapter 2: ROS 2 Architecture and Communication.

## Examples

### QoS Examples
Demonstrates different Quality of Service (QoS) policies in ROS 2:
- `qos_examples.py`: Publisher with different QoS configurations
- `qos_subscriber_example.py`: Subscriber with matching QoS configurations

## Running the Examples

1. Make sure you have ROS 2 installed and sourced.

2. To run the publisher example:
```bash
python3 qos_examples.py
```

3. In a separate terminal, to run the subscriber example:
```bash
python3 qos_subscriber_example.py
```

The examples demonstrate:
- Reliable vs Best Effort communication
- Different durability policies (Volatile vs Transient Local)
- Different history policies (Keep Last vs Keep All)