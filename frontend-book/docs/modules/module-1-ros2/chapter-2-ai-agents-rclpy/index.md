# Chapter 2: AI Agents with rclpy

This chapter explores how to connect Python-based AI agents to ROS 2 using rclpy, the Python client library for ROS 2. We'll examine patterns for integrating AI decision-making with robot control systems.

## Introduction to rclpy

rclpy is the Python client library for ROS 2 that enables Python applications to interface with the ROS 2 middleware. It provides Python bindings for all core ROS 2 functionality including nodes, publishers, subscribers, services, and actions.

### Why Use rclpy for AI Integration?

- **Python Ecosystem**: Access to rich AI libraries (TensorFlow, PyTorch, scikit-learn)
- **Rapid Prototyping**: Faster development and iteration for AI algorithms
- **Community Support**: Large Python AI community and resources
- **ROS 2 Integration**: Seamless communication with other ROS 2 nodes

### Installation and Setup

```bash
# Install rclpy (usually comes with ROS 2 installation)
pip install rclpy

# Or with ROS 2 environment sourced
sudo apt install python3-rclpy
```

## Creating AI Agent Nodes

An AI agent in ROS 2 is typically implemented as a node that:

1. Subscribes to sensor data and other inputs
2. Processes information using AI algorithms
3. Publishes decisions or commands
4. May provide services for external interaction

### Basic AI Agent Structure

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np

class AIAgentNode(Node):
    def __init__(self):
        super().__init__('ai_agent')

        # Subscriptions for sensor data
        self.sensor_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.sensor_callback,
            10
        )

        # Publishers for AI decisions
        self.decision_publisher = self.create_publisher(
            Twist,
            'ai_decisions',
            10
        )

        # Initialize AI model or algorithm
        self.initialize_ai_model()

        self.get_logger().info('AI Agent initialized')

    def initialize_ai_model(self):
        # Initialize your AI model here
        # This could be a neural network, rule-based system, etc.
        pass

    def sensor_callback(self, msg):
        # Process sensor data
        sensor_data = self.extract_features(msg)

        # Make AI decision
        decision = self.make_decision(sensor_data)

        # Publish decision
        self.publish_decision(decision)

    def extract_features(self, sensor_msg):
        # Extract relevant features from sensor data
        features = np.array(sensor_msg.position)
        return features

    def make_decision(self, features):
        # Apply AI algorithm to make decision
        # This is where your AI logic goes
        decision = Twist()  # Placeholder
        return decision

    def publish_decision(self, decision):
        self.decision_publisher.publish(decision)
```

## AI Agent Communication Patterns

### Reactive Agents

Reactive agents respond immediately to sensor inputs:

```python
class ReactiveAIAgent(Node):
    def __init__(self):
        super().__init__('reactive_ai_agent')

        # Subscribe to sensor data
        self.laser_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )

        # Publish velocity commands
        self.cmd_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

    def laser_callback(self, msg):
        # Simple obstacle avoidance
        min_distance = min(msg.ranges)

        cmd = Twist()
        if min_distance < 1.0:  # If obstacle is too close
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Turn
        else:
            cmd.linear.x = 0.5  # Move forward
            cmd.angular.z = 0.0

        self.cmd_publisher.publish(cmd)
```

### Planning Agents

Planning agents consider future states and make decisions based on goals:

```python
class PlanningAIAgent(Node):
    def __init__(self):
        super().__init__('planning_ai_agent')

        # Subscribe to current state
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        # Subscribe to goal
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_callback,
            10
        )

        # Publish planned path
        self.path_publisher = self.create_publisher(
            Path,
            'planned_path',
            10
        )

        self.current_pose = None
        self.goal_pose = None

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        self.plan_path()

    def plan_path(self):
        if self.current_pose and self.goal_pose:
            # Implement path planning algorithm
            path = self.calculate_path(self.current_pose, self.goal_pose)
            self.path_publisher.publish(path)
```

## Integration with Machine Learning Models

### Using TensorFlow/Keras Models

```python
import tensorflow as tf

class MLBasedAIAgent(Node):
    def __init__(self):
        super().__init__('ml_ai_agent')

        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        self.publisher = self.create_publisher(
            String,
            'ai_classification',
            10
        )

        # Load pre-trained model
        self.model = tf.keras.models.load_model('/path/to/model.h5')

    def image_callback(self, msg):
        # Convert ROS image to format expected by model
        image_data = self.ros_image_to_numpy(msg)

        # Preprocess image
        processed_image = self.preprocess(image_data)

        # Run inference
        prediction = self.model.predict(processed_image[np.newaxis, ...])

        # Publish result
        result = String()
        result.data = f'class_{np.argmax(prediction)}'
        self.publisher.publish(result)
```

### Using PyTorch Models

```python
import torch
import torch.nn as nn

class PyTorchAIAgent(Node):
    def __init__(self):
        super().__init__('pytorch_ai_agent')

        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_callback,
            10
        )

        self.publisher = self.create_publisher(
            JointState,
            'ai_joint_commands',
            10
        )

        # Load model
        self.model = self.load_model()
        self.model.eval()

    def load_model(self):
        model = YourPyTorchModel()
        model.load_state_dict(torch.load('/path/to/model.pth'))
        return model

    def joint_callback(self, msg):
        # Convert ROS message to tensor
        joint_tensor = torch.tensor(msg.position, dtype=torch.float32)

        # Run inference
        with torch.no_grad():
            commands = self.model(joint_tensor)

        # Publish commands
        joint_cmd = JointState()
        joint_cmd.position = commands.numpy().tolist()
        self.publisher.publish(joint_cmd)
```

## Advanced Integration Patterns

### Behavior Trees

Behavior trees provide a structured way to implement complex AI behaviors:

```python
class BehaviorTreeAgent(Node):
    def __init__(self):
        super().__init__('behavior_tree_agent')

        # Set up subscriptions and publishers
        self.setup_communication()

        # Initialize behavior tree
        self.root = self.build_behavior_tree()

    def build_behavior_tree(self):
        # Build your behavior tree structure
        root = SequenceNode([
            CheckBatteryNode(),
            CheckObstaclesNode(),
            NavigateToGoalNode()
        ])
        return root

    def timer_callback(self):
        # Tick the behavior tree
        status = self.root.tick()
```

### State Machines

State machines are useful for agents that need to maintain different operational modes:

```python
from enum import Enum

class AgentState(Enum):
    IDLE = 1
    NAVIGATING = 2
    MANIPULATING = 3
    EMERGENCY = 4

class StateMachineAIAgent(Node):
    def __init__(self):
        super().__init__('state_machine_agent')

        self.current_state = AgentState.IDLE
        self.state_timer = self.create_timer(0.1, self.state_machine_callback)

    def state_machine_callback(self):
        if self.current_state == AgentState.IDLE:
            self.handle_idle_state()
        elif self.current_state == AgentState.NAVIGATING:
            self.handle_navigation_state()
        # ... handle other states
```

## Performance Considerations

### Threading and Concurrency

For computationally intensive AI operations, consider using separate threads:

```python
import threading
from concurrent.futures import ThreadPoolExecutor

class ThreadingAIAgent(Node):
    def __init__(self):
        super().__init__('threading_ai_agent')

        self.executor = ThreadPoolExecutor(max_workers=2)
        self.ai_lock = threading.Lock()

    def sensor_callback(self, msg):
        # Submit AI processing to thread pool
        future = self.executor.submit(self.process_ai, msg)
        future.add_done_callback(self.ai_callback)

    def ai_callback(self, future):
        try:
            result = future.result()
            # Publish result on main thread
            self.publish_result(result)
        except Exception as e:
            self.get_logger().error(f'AI processing failed: {e}')
```

### Memory Management

AI models can be memory-intensive, so proper management is important:

```python
class MemoryEfficientAIAgent(Node):
    def __init__(self):
        super().__init__('memory_efficient_agent')

        # Load models on demand
        self.models = {}
        self.active_model = None

    def switch_model(self, model_name):
        if model_name not in self.models:
            # Load model if not already loaded
            self.models[model_name] = self.load_model(model_name)

        self.active_model = self.models[model_name]
```

## Summary

This chapter has covered the essential concepts for integrating AI agents with ROS 2 using rclpy. We've explored different agent architectures, integration patterns with machine learning frameworks, and performance considerations. The key to successful AI integration is designing clear communication patterns between your AI algorithms and the rest of your ROS 2 system.