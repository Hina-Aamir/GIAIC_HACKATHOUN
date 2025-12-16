# rclpy Examples for AI Integration

This section provides practical examples of integrating AI agents with ROS 2 using rclpy. Each example demonstrates different aspects of AI-ROS2 integration with real-world scenarios.

## Example 1: Simple Object Detection AI Agent

This example shows how to create an AI agent that processes camera images to detect objects:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import cv2

class ObjectDetectionAgent(Node):
    def __init__(self):
        super().__init__('object_detection_agent')

        # Create CV bridge for image conversion
        self.bridge = CvBridge()

        # Subscribe to camera images
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publish detection results
        self.detection_pub = self.create_publisher(
            String,
            '/object_detections',
            10
        )

        self.get_logger().info('Object Detection Agent started')

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Simple color-based object detection (example)
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Define range for red color (in HSV)
            lower_red = np.array([0, 50, 50])
            upper_red = np.array([10, 255, 255])
            mask1 = cv2.inRange(hsv, lower_red, upper_red)

            lower_red = np.array([170, 50, 50])
            upper_red = np.array([180, 255, 255])
            mask2 = cv2.inRange(hsv, lower_red, upper_red)

            mask = mask1 + mask2

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Check if any red objects detected
            if len(contours) > 0:
                result = "RED_OBJECT_DETECTED"
            else:
                result = "NO_RED_OBJECTS"

            # Publish result
            detection_msg = String()
            detection_msg.data = result
            self.detection_pub.publish(detection_msg)

            self.get_logger().info(f'Detection result: {result}')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionAgent()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Example 2: Reinforcement Learning Agent for Navigation

This example demonstrates a simple reinforcement learning agent for robot navigation:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math

class NavigationRLAgent(Node):
    def __init__(self):
        super().__init__('navigation_rl_agent')

        # Initialize Q-table (simplified for demonstration)
        self.q_table = np.zeros((10, 5))  # 10 states, 5 actions
        self.learning_rate = 0.1
        self.discount_factor = 0.95
        self.exploration_rate = 0.1

        # Robot state
        self.current_position = None
        self.laser_data = None
        self.previous_state = None
        self.previous_action = None

        # Create subscribers
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Create publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer for decision making
        self.timer = self.create_timer(0.1, self.decision_callback)

        self.get_logger().info('Navigation RL Agent started')

    def laser_callback(self, msg):
        self.laser_data = msg.ranges

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose

    def discretize_state(self, laser_data):
        """Convert continuous laser data to discrete state"""
        if not laser_data:
            return 0

        # Simplified state discretization
        front_avg = sum(laser_data[300:420]) / len(laser_data[300:420])  # Front readings
        left_avg = sum(laser_data[0:60]) / len(laser_data[0:60])  # Left readings
        right_avg = sum(laser_data[540:600]) / len(laser_data[540:600])  # Right readings

        # Create discrete state based on distance ranges
        if front_avg < 0.5:  # Obstacle very close
            state = 0
        elif front_avg < 1.0:  # Obstacle close
            state = 1
        elif left_avg < 0.5:  # Obstacle on left
            state = 2
        elif right_avg < 0.5:  # Obstacle on right
            state = 3
        else:  # Clear path
            state = 4

        return state

    def choose_action(self, state):
        """Choose action using epsilon-greedy policy"""
        if np.random.random() < self.exploration_rate:
            # Explore: random action
            return np.random.choice(5)
        else:
            # Exploit: best known action
            return np.argmax(self.q_table[state])

    def update_q_table(self, state, action, reward, next_state):
        """Update Q-table using Q-learning algorithm"""
        current_q = self.q_table[state, action]
        max_next_q = np.max(self.q_table[next_state])

        new_q = current_q + self.learning_rate * (
            reward + self.discount_factor * max_next_q - current_q
        )

        self.q_table[state, action] = new_q

    def calculate_reward(self, laser_data, action):
        """Calculate reward based on current state and action taken"""
        if not laser_data:
            return -1.0

        front_avg = sum(laser_data[300:420]) / len(laser_data[300:420])

        # Negative reward for being too close to obstacles
        if front_avg < 0.3:
            return -10.0  # Collision penalty

        # Positive reward for moving forward in clear space
        if action == 2 and front_avg > 1.0:  # Forward action with clear path
            return 1.0

        # Small negative reward for each step (to encourage efficiency)
        return -0.01

    def decision_callback(self):
        if self.laser_data is None:
            return

        # Get current state
        current_state = self.discretize_state(self.laser_data)

        # Choose action
        action = self.choose_action(current_state)

        # Execute action
        cmd_vel = Twist()
        if action == 0:  # Turn left sharply
            cmd_vel.angular.z = 1.0
        elif action == 1:  # Turn right sharply
            cmd_vel.angular.z = -1.0
        elif action == 2:  # Move forward
            cmd_vel.linear.x = 0.5
        elif action == 3:  # Turn left slowly
            cmd_vel.angular.z = 0.3
            cmd_vel.linear.x = 0.2
        elif action == 4:  # Turn right slowly
            cmd_vel.angular.z = -0.3
            cmd_vel.linear.x = 0.2

        self.cmd_pub.publish(cmd_vel)

        # Update Q-table if we have previous state-action pair
        if self.previous_state is not None:
            reward = self.calculate_reward(self.laser_data, self.previous_action)
            self.update_q_table(self.previous_state, self.previous_action, reward, current_state)

        # Store current state-action for next iteration
        self.previous_state = current_state
        self.previous_action = action

def main(args=None):
    rclpy.init(args=args)
    node = NavigationRLAgent()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Example 3: Multi-Agent Coordination

This example shows how multiple AI agents can coordinate:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import json

class CoordinationAgent(Node):
    def __init__(self, agent_id):
        super().__init__(f'coordination_agent_{agent_id}')

        self.agent_id = agent_id
        self.agent_positions = {}
        self.task_assignments = {}

        # Publisher for agent status
        self.status_pub = self.create_publisher(
            String,
            f'/agent_{agent_id}_status',
            10
        )

        # Subscriber for other agents' status
        for i in range(3):  # Assuming 3 agents
            if i != agent_id:
                self.create_subscription(
                    String,
                    f'/agent_{i}_status',
                    self.status_callback,
                    10
                )

        # Timer for coordination logic
        self.timer = self.create_timer(1.0, self.coordination_callback)

        self.get_logger().info(f'Coordination Agent {agent_id} started')

    def status_callback(self, msg):
        try:
            status_data = json.loads(msg.data)
            agent_id = status_data['agent_id']
            self.agent_positions[agent_id] = status_data['position']
            self.task_assignments[agent_id] = status_data.get('task', 'idle')
        except Exception as e:
            self.get_logger().error(f'Error parsing status: {e}')

    def coordination_callback(self):
        # Simple coordination logic
        my_position = self.get_current_position()
        my_task = self.assign_task()

        # Prepare status message
        status_msg = String()
        status_msg.data = json.dumps({
            'agent_id': self.agent_id,
            'position': my_position,
            'task': my_task,
            'timestamp': self.get_clock().now().nanoseconds
        })

        self.status_pub.publish(status_msg)

        # Log coordination info
        self.get_logger().info(f'Agent {self.agent_id}: Task {my_task}, Coordination updated')

    def get_current_position(self):
        # In a real system, this would come from odometry or localization
        return {'x': 0.0, 'y': 0.0, 'theta': 0.0}

    def assign_task(self):
        # Simple task assignment based on other agents' tasks
        busy_tasks = set(self.task_assignments.values())

        # Define available tasks
        available_tasks = ['explore_north', 'explore_south', 'explore_east', 'monitor_area']

        # Assign first available task
        for task in available_tasks:
            if task not in busy_tasks:
                return task

        return 'idle'  # No tasks available

def main(args=None):
    rclpy.init(args=args)

    # Create multiple coordination agents
    agents = []
    for i in range(3):
        agent = CoordinationAgent(i)
        agents.append(agent)

    try:
        # Use MultiThreadedExecutor to run all agents
        executor = rclpy.executors.MultiThreadedExecutor()
        for agent in agents:
            executor.add_node(agent)

        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for agent in agents:
            agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Example 4: Deep Learning Integration with TensorFlow

This example demonstrates integrating a TensorFlow model:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from cv_bridge import CvBridge
import tensorflow as tf
import numpy as np

class DeepLearningAgent(Node):
    def __init__(self):
        super().__init__('deep_learning_agent')

        # Initialize TensorFlow
        self.bridge = CvBridge()

        # Load pre-trained model
        try:
            self.model = tf.keras.models.load_model('/path/to/your/model.h5')
            self.get_logger().info('Deep learning model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            self.model = None

        # Subscribe to sensor data
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        # Publish AI decisions
        self.decision_pub = self.create_publisher(
            String,
            '/ai_decisions',
            10
        )

        self.get_logger().info('Deep Learning Agent started')

    def preprocess_image(self, cv_image):
        """Preprocess image for model input"""
        # Resize image to model's expected input size
        resized = cv2.resize(cv_image, (224, 224))

        # Normalize pixel values
        normalized = resized.astype(np.float32) / 255.0

        # Add batch dimension
        batched = np.expand_dims(normalized, axis=0)

        return batched

    def image_callback(self, msg):
        if self.model is None:
            return

        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Preprocess image
            processed_image = self.preprocess_image(cv_image)

            # Run inference
            predictions = self.model.predict(processed_image, verbose=0)

            # Interpret results
            predicted_class = np.argmax(predictions[0])
            confidence = predictions[0][predicted_class]

            # Publish decision if confidence is high enough
            if confidence > 0.7:  # Confidence threshold
                decision_msg = String()
                decision_msg.data = f'CLASS_{predicted_class}_CONF_{confidence:.2f}'
                self.decision_pub.publish(decision_msg)

                self.get_logger().info(f'Prediction: Class {predicted_class}, Confidence: {confidence:.2f}')

        except Exception as e:
            self.get_logger().error(f'Error in image processing: {e}')

    def joint_callback(self, msg):
        # Process joint state data with AI
        if self.model is None:
            return

        try:
            # Convert joint positions to feature vector
            joint_features = np.array(msg.position).astype(np.float32)

            # Reshape for model input (assuming model expects this format)
            features = joint_features.reshape(1, -1)

            # Run inference
            prediction = self.model.predict(features, verbose=0)

            # Publish result
            decision_msg = String()
            decision_msg.data = f'JOINT_PREDICTION_{prediction[0][0]:.3f}'
            self.decision_pub.publish(decision_msg)

        except Exception as e:
            self.get_logger().error(f'Error in joint processing: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DeepLearningAgent()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Example 5: Humanoid Robot Control with AI

This example demonstrates AI control of a humanoid robot:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
import numpy as np
from collections import deque

class HumanoidControlAgent(Node):
    def __init__(self):
        super().__init__('humanoid_control_agent')

        # Robot state tracking
        self.joint_states = {}
        self.imu_data = None
        self.balance_buffer = deque(maxlen=10)  # Store recent balance data

        # Subscribe to robot sensors
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Publish trajectory commands
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory',
            10
        )

        # Timer for control loop
        self.control_timer = self.create_timer(0.02, self.control_callback)  # 50 Hz

        self.get_logger().info('Humanoid Control Agent started')

    def joint_callback(self, msg):
        # Store joint positions
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_states[name] = msg.position[i]

    def imu_callback(self, msg):
        # Store IMU data for balance control
        self.imu_data = {
            'orientation': msg.orientation,
            'angular_velocity': msg.angular_velocity,
            'linear_acceleration': msg.linear_acceleration
        }

        # Store orientation for balance analysis
        orientation = msg.orientation
        # Convert quaternion to roll/pitch for balance assessment
        roll = self.quaternion_to_roll(orientation)
        pitch = self.quaternion_to_pitch(orientation)

        self.balance_buffer.append((roll, pitch))

    def quaternion_to_roll(self, q):
        # Convert quaternion to roll angle
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        return np.arctan2(sinr_cosp, cosr_cosp)

    def quaternion_to_pitch(self, q):
        # Convert quaternion to pitch angle
        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            return np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
        return np.arcsin(sinp)

    def balance_control(self):
        """AI-based balance control algorithm"""
        if not self.balance_buffer:
            return {}

        # Calculate average orientation from recent data
        recent_orientations = list(self.balance_buffer)
        avg_roll = np.mean([x[0] for x in recent_orientations])
        avg_pitch = np.mean([x[1] for x in recent_orientations])

        # Simple balance correction algorithm
        corrections = {}

        # Correct roll (left/right balance)
        if abs(avg_roll) > 0.1:  # If tilted more than 5.7 degrees
            # Adjust hip joints to correct roll
            corrections['left_hip_roll'] = -avg_roll * 0.5
            corrections['right_hip_roll'] = avg_roll * 0.5

        # Correct pitch (forward/back balance)
        if abs(avg_pitch) > 0.1:  # If tilted more than 5.7 degrees
            # Adjust hip joints to correct pitch
            corrections['left_hip_pitch'] = -avg_pitch * 0.3
            corrections['right_hip_pitch'] = -avg_pitch * 0.3

        return corrections

    def generate_trajectory_command(self, corrections):
        """Generate trajectory command from corrections"""
        traj = JointTrajectory()
        traj.joint_names = list(corrections.keys())

        point = JointTrajectoryPoint()

        for joint_name in traj.joint_names:
            # Get current position and add correction
            current_pos = self.joint_states.get(joint_name, 0.0)
            target_pos = current_pos + corrections[joint_name]
            point.positions.append(target_pos)
            point.velocities.append(0.0)  # Start with zero velocity
            point.accelerations.append(0.0)  # Start with zero acceleration

        # Set timing (0.1 seconds to reach target)
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100000000  # 0.1 seconds

        traj.points = [point]
        return traj

    def control_callback(self):
        if not self.joint_states or not self.imu_data:
            return

        # Apply AI-based balance control
        corrections = self.balance_control()

        if corrections:
            # Generate and publish trajectory command
            traj_cmd = self.generate_trajectory_command(corrections)
            self.trajectory_pub.publish(traj_cmd)

            self.get_logger().info(f'Balance corrections applied: {corrections}')

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidControlAgent()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Running the Examples

To run these examples:

1. **Install dependencies**:
   ```bash
   pip install opencv-python tensorflow numpy
   ```

2. **Make the Python files executable**:
   ```bash
   chmod +x object_detection_agent.py
   ```

3. **Source ROS 2 environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

4. **Run the example**:
   ```bash
   python3 object_detection_agent.py
   ```

## Summary

These examples demonstrate various approaches to integrating AI with ROS 2 using rclpy. From simple object detection to complex humanoid control, the examples show how to structure AI agents that can effectively communicate with ROS 2 systems. Each example can be extended and modified for specific use cases in humanoid robotics applications.