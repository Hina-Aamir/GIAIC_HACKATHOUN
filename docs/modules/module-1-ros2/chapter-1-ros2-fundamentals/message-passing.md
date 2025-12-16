# Message Passing in ROS 2

Message passing is the core communication mechanism in ROS 2 that enables nodes to exchange information. Understanding how messages are structured, passed, and handled is essential for creating effective humanoid robot systems.

## Message Types and Structure

ROS 2 messages are structured data containers that carry information between nodes. Each message type is defined in a `.msg` file that specifies the fields and their data types.

### Common Message Types

#### Standard Messages
- `std_msgs`: Basic data types (Int32, Float64, String, etc.)
- `geometry_msgs`: Geometric primitives (Point, Pose, Twist, etc.)
- `sensor_msgs`: Sensor data (JointState, Image, LaserScan, etc.)
- `nav_msgs`: Navigation-related messages (Odometry, Path, etc.)

#### Example Message Definition
```
# geometry_msgs/Twist.msg
Vector3 linear
Vector3 angular
```

```
# sensor_msgs/JointState.msg
Header header
string[] name
float64[] position
float64[] velocity
float64[] effort
```

### Creating Custom Messages

For humanoid robotics applications, you might need custom message types:

```
# For humanoid robot state
# msg/HumanoidState.msg
Header header
string robot_name
float64[] joint_positions
float64[] joint_velocities
float64[] joint_efforts
float64[] imu_data
float64[] force_torque_data
```

To create custom messages, define them in `.msg` files in your package's `msg/` directory and update your package's CMakeLists.txt and package.xml accordingly.

## Message Passing Mechanisms

### Publisher-Subscriber Pattern

The publisher-subscriber pattern is the most common communication pattern in ROS 2:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class MessagePublisher(Node):
    def __init__(self):
        super().__init__('message_publisher')

        # Create publisher
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)

        # Timer to periodically publish messages
        self.timer = self.create_timer(0.1, self.publish_joint_states)

    def publish_joint_states(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Populate joint state data
        msg.name = ['hip_joint', 'knee_joint', 'ankle_joint']
        msg.position = [0.0, 0.5, -0.2]
        msg.velocity = [0.0, 0.0, 0.0]
        msg.effort = [0.0, 0.0, 0.0]

        self.publisher.publish(msg)
        self.get_logger().info(f'Published joint states: {msg.position}')

class MessageSubscriber(Node):
    def __init__(self):
        super().__init__('message_subscriber')

        # Create subscriber
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

    def joint_state_callback(self, msg):
        self.get_logger().info(f'Received joint positions: {msg.position}')
```

### Quality of Service (QoS) Profiles

QoS profiles control how messages are delivered and can be tailored to your specific needs:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# For critical control commands (guaranteed delivery)
control_qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST
)

# For sensor data (best effort, may lose some messages)
sensor_qos = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST
)

# For configuration data (keep all messages)
config_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_ALL
)
```

## Message Serialization and Transport

### Serialization Process

When a message is published, ROS 2 performs these steps:
1. Serializes the message data into bytes
2. Packages it according to the DDS protocol
3. Transports it to subscribers via the middleware
4. Deserializes the message at the subscriber end

### Transport Protocols

ROS 2 uses DDS (Data Distribution Service) as its middleware, which supports various transport protocols:
- **UDP**: Default for discovery and data transport
- **TCP**: For reliable delivery over networks
- **Shared Memory**: For high-performance local communication

## Message Synchronization

For humanoid robotics, it's often important to synchronize different types of data:

### Time Synchronization
```python
from rclpy.time import Time
from builtin_interfaces.msg import Time as TimeMsg

def create_timestamped_message(self):
    msg = JointState()
    current_time = self.get_clock().now()
    msg.header.stamp = current_time.to_msg()
    return msg
```

### Message Filters
For more complex synchronization needs, you can use message filters:

```python
from message_filters import ApproximateTimeSynchronizer, Subscriber

# Synchronize messages from different topics with approximate time matching
def sync_callback(self, image_msg, depth_msg):
    # Process synchronized image and depth data
    pass

# Note: This requires additional packages like message_filters
# This is more commonly used in ROS 1, but similar patterns can be implemented in ROS 2
```

## Performance Considerations

### Message Size
- Large messages can impact network performance
- Consider compressing large data (images, point clouds)
- Use appropriate data types (float32 vs float64)

### Message Frequency
- High-frequency messages can overwhelm the system
- Consider decimation for high-rate sensors
- Use callbacks efficiently to avoid blocking

### Memory Management
- Messages are copied between nodes
- Large messages can consume significant memory
- Consider message reuse patterns for performance-critical applications

## Practical Example: Humanoid Robot Message Passing

Here's a complete example showing message passing in a humanoid robot context:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy

class HumanoidMessageHandler(Node):
    def __init__(self):
        super().__init__('humanoid_message_handler')

        # QoS profiles for different data types
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        control_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)

        # Publishers for robot state and commands
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', sensor_qos)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', control_qos)
        self.status_pub = self.create_publisher(String, '/robot_status', 10)

        # Subscribers for sensor data and commands
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, sensor_qos)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, control_qos)

        # Timer for periodic state publishing
        self.timer = self.create_timer(0.02, self.publish_robot_state)  # 50 Hz

        self.get_logger().info('Humanoid message handler initialized')

    def joint_state_callback(self, msg):
        # Process incoming joint state data
        self.get_logger().info(f'Joint positions: {msg.position[:3]}...', throttle_duration_sec=1.0)

    def cmd_vel_callback(self, msg):
        # Process velocity commands
        self.get_logger().info(f'Received velocity command: {msg.linear.x}, {msg.angular.z}')

    def publish_robot_state(self):
        # Publish current robot state
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = ['hip_joint', 'knee_joint', 'ankle_joint', 'shoulder_joint', 'elbow_joint']
        joint_msg.position = [0.1, 0.2, 0.3, 0.4, 0.5]  # Example positions
        self.joint_state_pub.publish(joint_msg)

        status_msg = String()
        status_msg.data = 'Running'
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidMessageHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

Message passing in ROS 2 provides the essential communication infrastructure for humanoid robot systems. By understanding message types, QoS profiles, and performance considerations, you can design efficient and robust communication patterns that meet the real-time requirements of humanoid robotics applications. The flexibility of ROS 2's message passing system allows you to tailor communication to your specific needs while maintaining system modularity and scalability.