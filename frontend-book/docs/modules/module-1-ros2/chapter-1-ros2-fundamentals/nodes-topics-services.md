# Nodes, Topics, and Services in Depth

In the previous section, we introduced the fundamental concepts of ROS 2. Now, let's dive deeper into each of these communication patterns and understand how they work in practice for humanoid robot control.

## Nodes: The Building Blocks of ROS 2

Nodes are the fundamental building blocks of any ROS 2 system. Each node is responsible for a specific task or set of tasks in the robot's operation. In humanoid robotics, you might have nodes for:

- Sensor data processing (camera, IMU, joint encoders)
- Motion planning and control
- AI decision making
- Communication with actuators
- State estimation
- Behavior management

### Node Lifecycle

A ROS 2 node goes through several states in its lifecycle:

1. **Unconfigured**: The node is created but not yet configured
2. **Inactive**: The node is configured but not active
3. **Active**: The node is running and can communicate
4. **Finalized**: The node is shut down and cleaned up

### Node Parameters

Nodes can be configured using parameters, which allow you to modify node behavior without recompiling:

```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('robot_name', 'humanoid_robot')
        self.declare_parameter('control_frequency', 50)

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.control_frequency = self.get_parameter('control_frequency').value

        self.get_logger().info(f'Robot name: {self.robot_name}, Frequency: {self.control_frequency}')
```

## Topics: Asynchronous Communication

Topics enable asynchronous, many-to-many communication between nodes. This pattern is ideal for:

- Sensor data broadcasting (camera feeds, LIDAR scans, joint states)
- Robot state publishing (joint positions, velocities, efforts)
- Event notifications (collision warnings, battery levels)
- Control commands (velocity commands, joint positions)

### Topic Characteristics

- **Publish/Subscribe Model**: Publishers send messages, subscribers receive them
- **Decoupled**: Publishers and subscribers don't need to know about each other
- **Many-to-Many**: Multiple publishers can publish to the same topic; multiple subscribers can listen to the same topic
- **Asynchronous**: Publishers don't wait for subscribers to receive messages

### Quality of Service (QoS) for Topics

QoS settings allow you to control how messages are delivered:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# For sensor data where you want the latest values
qos_profile = QoSProfile(
    depth=1,  # Keep only the latest message
    reliability=ReliabilityPolicy.BEST_EFFORT,  # Accept some message loss
    history=HistoryPolicy.KEEP_LAST  # Keep only recent messages
)

publisher = self.create_publisher(String, 'sensor_data', qos_profile)
```

### Topic Names and Namespaces

Topic names follow a hierarchical structure with namespaces:

```
/joint_states          # Global topic
/sensors/camera/image  # Sensor-specific topic
/robot1/cmd_vel        # Robot-specific topic
```

## Services: Synchronous Request/Response

Services provide synchronous, one-to-one communication. They're ideal for:

- Configuration requests (setting parameters, calibration)
- One-time operations (navigation goals, action execution)
- Query operations (current robot state, map data)

### Service Architecture

```python
# Service definition (typically in .srv files)
# Request: a int64, b int64
# Response: sum int64
```

### When to Use Services vs Topics

| Use Services When... | Use Topics When... |
|---------------------|-------------------|
| You need a response | You're broadcasting information |
| Operation has a clear start/end | Data is continuously changing |
| You need guaranteed delivery | You can tolerate some message loss |
| Request/response pattern is natural | Many-to-many communication is needed |

## Practical Example: Humanoid Robot Control

Let's consider how these concepts apply to humanoid robot control:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from control_msgs.srv import SwitchController

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

        # Subscribe to joint states (topics)
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publish velocity commands (topics)
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Publish robot status (topics)
        self.status_publisher = self.create_publisher(
            String,
            '/robot_status',
            10
        )

        # Service for switching controllers (services)
        self.controller_service = self.create_service(
            SwitchController,
            '/switch_controller',
            self.switch_controller_callback
        )

        self.get_logger().info('Humanoid controller initialized')

    def joint_state_callback(self, msg):
        # Process joint state data
        self.get_logger().info(f'Received joint states for {len(msg.name)} joints')

    def switch_controller_callback(self, request, response):
        # Handle controller switching request
        self.get_logger().info(f'Switching controller to {request.start_controllers}')
        response.ok = True
        return response
```

## Best Practices

### For Nodes:
- Keep nodes focused on a single responsibility
- Use meaningful names that reflect the node's function
- Implement proper error handling and logging
- Consider resource usage and efficiency

### For Topics:
- Use descriptive topic names that clearly indicate the content
- Consider message frequency and network bandwidth
- Use appropriate QoS settings based on your application needs
- Follow naming conventions consistently

### For Services:
- Use services for operations that require a response
- Consider the impact of blocking calls on your application
- Implement timeouts for service calls
- Design service interfaces that are clear and intuitive

## Summary

Understanding nodes, topics, and services is crucial for building effective humanoid robot systems. These communication patterns provide the foundation for creating distributed, modular robot applications that can scale from simple to complex systems. The choice between topics and services depends on your specific use case and communication requirements.