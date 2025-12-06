# ROS 2 Introduction Examples

This package contains example code for Chapter 1: Introduction to ROS 2.

## Examples

### Minimal Publisher
A simple publisher that sends "Hello World" messages to a topic.

### Minimal Subscriber
A simple subscriber that listens to messages from the publisher.

## Running the Examples

1. Make sure you have ROS 2 installed and sourced.

2. To run the publisher:
```bash
python3 minimal_publisher.py
```

3. In a separate terminal, to run the subscriber:
```bash
python3 minimal_subscriber.py
```

You should see the publisher sending messages and the subscriber receiving them.