# Quickstart Guide: Module 1 - The Robotic Nervous System (ROS 2)

## Prerequisites

Before starting this module, you should have:

1. Basic programming knowledge (especially Python)
2. Understanding of robotics concepts (helpful but not required)
3. A Linux environment (Ubuntu 22.04 recommended) or Docker container with ROS 2

## Setting Up Your Environment

### Installing ROS 2 (Humble Hawksbill)

```bash
# Add the ROS 2 apt repository
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop
```

### Installing Python Dependencies

```bash
pip install rclpy
```

### Setting Up Your ROS 2 Environment

```bash
source /opt/ros/humble/setup.bash
```

## Chapter 1: ROS 2 Fundamentals

### Creating Your First Node

1. Create a new directory for your examples:
```bash
mkdir -p ~/ros2_examples/my_first_node
cd ~/ros2_examples/my_first_node
```

2. Create a simple Python node:
```python
# my_node.py
import rclpy
from rclpy.node import Node

class MyFirstNode(Node):
    def __init__(self):
        super().__init__('my_first_node')
        self.get_logger().info('Hello from my first ROS 2 node!')

def main(args=None):
    rclpy.init(args=args)
    node = MyFirstNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

3. Run your node:
```bash
python3 my_node.py
```

### Understanding Topics

Create a simple publisher and subscriber to understand topic-based communication:

**Publisher:**
```python
# publisher.py
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

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Subscriber:**
```python
# subscriber.py
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

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Run them in separate terminals:
```bash
# Terminal 1
python3 publisher.py

# Terminal 2
python3 subscriber.py
```

## Chapter 2: AI Agents with rclpy

### Simple AI Agent Node

Create an AI agent that processes sensor data and makes decisions:

```python
# ai_agent.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32

class AIAgent(Node):
    def __init__(self):
        super().__init__('ai_agent')

        # Subscribe to sensor data
        self.sensor_subscription = self.create_subscription(
            String,
            'sensor_data',
            self.sensor_callback,
            10)

        # Publish decisions
        self.decision_publisher = self.create_publisher(String, 'decisions', 10)

        self.get_logger().info('AI Agent initialized')

    def sensor_callback(self, msg):
        # Simple decision logic
        sensor_data = msg.data
        decision = self.make_decision(sensor_data)

        # Publish decision
        decision_msg = String()
        decision_msg.data = decision
        self.decision_publisher.publish(decision_msg)
        self.get_logger().info(f'AI Decision: {decision}')

    def make_decision(self, sensor_data):
        # Simple decision algorithm
        if 'obstacle' in sensor_data.lower():
            return 'STOP'
        elif 'clear' in sensor_data.lower():
            return 'MOVE_FORWARD'
        else:
            return 'STANDBY'

def main(args=None):
    rclpy.init(args=args)
    ai_agent = AIAgent()
    rclpy.spin(ai_agent)
    ai_agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Chapter 3: URDF Basics

### Simple URDF Example

Create a simple URDF file for a robot:

```xml
<!-- simple_robot.urdf -->
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Child link connected by a joint -->
  <link name="sensor_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 0.8"/>
      </material>
    </visual>
  </link>

  <joint name="sensor_joint" type="fixed">
    <parent link="base_link"/>
    <child link="sensor_link"/>
    <origin xyz="0.2 0 0.1"/>
  </joint>
</robot>
```

### Validating URDF

To check if your URDF is valid:

```bash
# Install check tools
sudo apt install ros-humble-urdfdom-py

# Validate your URDF file
python3 -c "from urdf_parser_py.urdf import URDF; robot = URDF.from_xml_file('simple_robot.urdf'); print('URDF is valid')"
```

## Next Steps

After completing this quickstart:

1. Review the detailed content in each chapter
2. Practice with more complex examples
3. Explore how ROS 2 concepts apply to humanoid robotics
4. Prepare for Module 2 which will cover simulation environments

## Troubleshooting

- If ROS 2 commands are not found, make sure you've sourced the setup.bash file
- If Python packages are not found, ensure you've installed rclpy
- Check that your ROS 2 environment is properly configured with `printenv | grep ROS`