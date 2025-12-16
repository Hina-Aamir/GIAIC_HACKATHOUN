# Chapter 1: ROS 2 Fundamentals

This chapter introduces the core concepts of ROS 2 (Robot Operating System 2), which serves as the middleware for humanoid robot control. Understanding these fundamentals is essential for building complex robotic systems.

## What is ROS 2?

ROS 2 is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

Unlike traditional approaches where all robot functionality runs in a single process, ROS 2 uses a distributed architecture where different functionalities run in separate processes called "nodes". These nodes communicate with each other using a publish/subscribe model, service calls, and actions.

### Key Differences from ROS 1

- **Real-time support**: ROS 2 supports real-time programming requirements
- **Improved security**: Built-in security features for protected communication
- **Better cross-platform support**: Works on Linux, macOS, Windows, and embedded systems
- **Production-ready**: Designed for deployment in production environments
- **DDS-based middleware**: Uses Data Distribution Service (DDS) for communication

## Nodes

A node is an entity that performs computation in ROS. Nodes are combined together into a graph and communicate with one another using topics, services, and actions.

### Characteristics of Nodes:
- Each node runs in its own process
- Nodes can be written in different programming languages (Python, C++, etc.)
- Nodes can be started and stopped independently
- Nodes can be distributed across multiple machines

### Creating a Node in Python

Here's a basic example of creating a node using rclpy:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        self.get_logger().info('Node has been created')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics and Message Passing

Topics are named buses over which nodes exchange messages. Topic-based communication is asynchronous, allowing for multiple publishers and subscribers.

### Key Concepts:
- **Publisher**: A node that sends messages to a topic
- **Subscriber**: A node that receives messages from a topic
- **Message**: The data structure sent between nodes
- **Topic Name**: The string identifier for the communication channel

### Publisher Example:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1
```

### Subscriber Example:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
```

## Services

Services provide a request/response communication pattern. Unlike topics which are asynchronous, services are synchronous and allow for a single client and server.

### Key Concepts:
- **Service Server**: Provides a service and responds to requests
- **Service Client**: Sends requests to a service server
- **Service Type**: Defines the request and response message types

### Service Server Example:

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning: {response.sum}')
        return response
```

### Service Client Example:

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

## Actions

Actions are a more advanced communication pattern that provides feedback during long-running tasks and supports cancellation.

### Key Concepts:
- **Goal**: The request for an action to be performed
- **Feedback**: Updates during action execution
- **Result**: The final outcome of the action

## Quality of Service (QoS) Settings

QoS settings allow you to configure how messages are delivered between nodes, including reliability, durability, and history settings.

## Summary

In this chapter, we've covered the fundamental concepts of ROS 2:

- **Nodes** as the basic computational units
- **Topics** for asynchronous message passing
- **Services** for synchronous request/response communication
- **Actions** for long-running tasks with feedback

These concepts form the foundation for more advanced topics in the following chapters, where we'll explore how to connect AI agents to ROS 2 and model humanoid robots.