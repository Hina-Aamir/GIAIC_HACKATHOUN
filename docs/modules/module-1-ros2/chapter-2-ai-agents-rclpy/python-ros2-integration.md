# Python-ROS2 Integration

This section explores the technical aspects of integrating Python-based AI agents with ROS 2 systems using rclpy. We'll cover the fundamental patterns, best practices, and advanced techniques for creating robust AI-ROS2 integrations.

## Understanding rclpy Architecture

rclpy is built as a Python wrapper around the ROS 2 client library (rcl), which itself interfaces with the DDS (Data Distribution Service) middleware. This architecture provides:

- **Pythonic API**: Familiar Python syntax for ROS 2 operations
- **Performance**: Direct access to the underlying C libraries
- **Compatibility**: Full compatibility with other ROS 2 client libraries (rclcpp, rclc, etc.)

### Core Components

```python
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
```

The main components you'll work with are:
- **Node**: The basic execution unit
- **Publisher**: For sending messages
- **Subscriber**: For receiving messages
- **Service**: For providing request/response communication
- **Client**: For making service requests
- **Action**: For long-running tasks with feedback
- **Executor**: For managing node execution and callbacks

## Node Design Patterns

### Basic Node Structure

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BasicROS2Node(Node):
    def __init__(self):
        # Initialize parent class with node name
        super().__init__('basic_ros2_node')

        # Create publisher
        self.publisher = self.create_publisher(String, 'topic_name', 10)

        # Create subscriber
        self.subscription = self.create_subscription(
            String,
            'input_topic',
            self.callback,
            10
        )

        # Create timer
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info('Basic ROS2 Node initialized')

    def callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
        # Process message and potentially publish result
        response = String()
        response.data = f'Processed: {msg.data}'
        self.publisher.publish(response)

    def timer_callback(self):
        msg = String()
        msg.data = 'Timer message'
        self.publisher.publish(msg)
```

### Advanced Node with Parameters

```python
class AdvancedROS2Node(Node):
    def __init__(self):
        super().__init__('advanced_ros2_node')

        # Declare parameters with default values and descriptions
        self.declare_parameter('model_path', '/default/path/to/model')
        self.declare_parameter('inference_frequency', 10.0)
        self.declare_parameter('confidence_threshold', 0.7)

        # Access parameter values
        self.model_path = self.get_parameter('model_path').value
        self.inf_freq = self.get_parameter('inference_frequency').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value

        # Set up communication
        self.setup_communication()

        # Initialize AI components
        self.initialize_ai_components()

    def setup_communication(self):
        # Set up publishers, subscribers, services, etc.
        pass

    def initialize_ai_components(self):
        # Load models, initialize algorithms, etc.
        pass
```

## Message Handling Strategies

### Synchronous vs Asynchronous Processing

For AI agents, choosing the right processing strategy is crucial:

```python
import asyncio
from threading import Thread

class ProcessingStrategyNode(Node):
    def __init__(self):
        super().__init__('processing_strategy_node')

        # For real-time applications: synchronous processing
        self.realtime_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.synchronous_callback,
            10
        )

        # For non-real-time applications: asynchronous processing
        self.async_sub = self.create_subscription(
            String,
            'async_topic',
            self.async_callback_wrapper,
            10
        )

    def synchronous_callback(self, msg):
        # Process immediately (blocks until done)
        result = self.ai_process_sync(msg)
        # Publish result immediately
        self.publish_result(result)

    def async_callback_wrapper(self, msg):
        # Start async processing in separate thread
        thread = Thread(target=self.ai_process_async, args=(msg,))
        thread.start()

    def ai_process_async(self, msg):
        # Non-blocking AI processing
        result = self.heavy_ai_computation(msg)
        # Use threading lock to safely publish from thread
        with self.lock:
            self.publish_result(result)
```

### Message Filtering and Decimation

For high-frequency sensors, filtering can improve performance:

```python
class FilteredProcessingNode(Node):
    def __init__(self):
        super().__init__('filtered_processing_node')

        # Subscribe with custom QoS for high-frequency data
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            qos
        )

        # Counter for decimation
        self.message_counter = 0
        self.decimation_factor = 5  # Process every 5th message

    def image_callback(self, msg):
        self.message_counter += 1

        if self.message_counter % self.decimation_factor == 0:
            # Process this message
            self.process_image(msg)
        else:
            # Skip processing, but maybe log that we skipped
            self.get_logger().debug(f'Skipped message {self.message_counter}')
```

## Advanced Communication Patterns

### Service Integration for AI Models

```python
from example_interfaces.srv import Trigger
from std_srvs.srv import SetBool

class ServiceIntegratedNode(Node):
    def __init__(self):
        super().__init__('service_integrated_node')

        # Service for triggering AI inference
        self.inference_service = self.create_service(
            Trigger,
            'trigger_inference',
            self.inference_service_callback
        )

        # Service for enabling/disabling AI
        self.enable_service = self.create_service(
            SetBool,
            'enable_ai',
            self.enable_service_callback
        )

        # Flag to enable/disable AI processing
        self.ai_enabled = True

    def inference_service_callback(self, request, response):
        if self.ai_enabled:
            # Perform AI inference
            result = self.perform_inference()
            response.success = True
            response.message = f'Inference result: {result}'
        else:
            response.success = False
            response.message = 'AI is disabled'

        return response

    def enable_service_callback(self, request, response):
        self.ai_enabled = request.data
        response.success = True
        response.message = f'AI enabled: {self.ai_enabled}'
        return response
```

### Action Integration for Long-Running AI Tasks

```python
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from example_interfaces.action import Fibonacci

class ActionIntegratedNode(Node):
    def __init__(self):
        super().__init__('action_integrated_node')

        # Create action server for AI tasks
        self._action_server = ActionServer(
            self,
            Fibonacci,  # Replace with your custom action
            'ai_task',
            execute_callback=self.execute_action_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

    def goal_callback(self, goal_request):
        # Accept or reject the goal
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        # Accept or reject the cancel request
        return CancelResponse.ACCEPT

    def execute_action_callback(self, goal_handle):
        self.get_logger().info('Executing AI task...')

        # Simulate long-running AI computation
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            # Check if goal was cancelled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('AI task canceled')
                return Fibonacci.Result()

            # Perform AI computation step
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1]
            )

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)

        # Complete the goal
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info('AI task completed')

        return result
```

## Performance Optimization

### Efficient Message Handling

```python
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class EfficientMessageNode(Node):
    def __init__(self):
        super().__init__('efficient_message_node')

        self.bridge = CvBridge()

        # For image processing, use efficient conversion
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.efficient_image_callback,
            10
        )

    def efficient_image_callback(self, msg):
        # Convert ROS image to OpenCV format efficiently
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        # Process with AI model
        result = self.ai_process_image(cv_image)

        # Publish result
        self.publish_result(result)
```

### Memory Management for AI Models

```python
import gc
import torch  # or tensorflow

class MemoryManagedNode(Node):
    def __init__(self):
        super().__init__('memory_managed_node')

        # Load models efficiently
        self.models = {}
        self.current_model = None
        self.model_lock = threading.Lock()

        # Timer to periodically clean up memory
        self.cleanup_timer = self.create_timer(30.0, self.cleanup_memory)

    def load_model(self, model_name, model_path):
        with self.model_lock:
            if model_name not in self.models:
                # Load model
                model = torch.load(model_path)
                model.eval()
                self.models[model_name] = model
                self.current_model = model_name

    def cleanup_memory(self):
        # Clean up Python garbage
        gc.collect()

        # If using GPU, clean up GPU memory
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
```

## Error Handling and Robustness

### Graceful Error Handling

```python
class RobustAINode(Node):
    def __init__(self):
        super().__init__('robust_ai_node')

        # Set up error handling
        self.error_count = 0
        self.max_errors = 10

        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.robust_callback,
            10
        )

    def robust_callback(self, msg):
        try:
            # AI processing that might fail
            result = self.ai_process_with_error_handling(msg)

            # Reset error count on success
            self.error_count = 0

            # Publish result
            self.publish_result(result)

        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f'AI processing error #{self.error_count}: {e}')

            # Check if we should shut down
            if self.error_count >= self.max_errors:
                self.get_logger().fatal('Too many errors, shutting down')
                rclpy.shutdown()
```

### Model Health Monitoring

```python
class HealthMonitoredNode(Node):
    def __init__(self):
        super().__init__('health_monitored_node')

        # Monitor AI model health
        self.last_inference_time = self.get_clock().now()
        self.inference_count = 0
        self.health_timer = self.create_timer(5.0, self.health_check)

        self.subscription = self.create_subscription(
            String,
            'input_topic',
            self.monitored_callback,
            10
        )

    def monitored_callback(self, msg):
        start_time = self.get_clock().now()

        try:
            result = self.ai_process(msg)

            # Update health metrics
            end_time = self.get_clock().now()
            inference_time = (end_time - start_time).nanoseconds / 1e9

            self.last_inference_time = end_time
            self.inference_count += 1

            # Log performance
            if inference_time > 1.0:  # If taking more than 1 second
                self.get_logger().warn(f'Slow inference: {inference_time:.2f}s')

            self.publish_result(result)

        except Exception as e:
            self.get_logger().error(f'Inference failed: {e}')

    def health_check(self):
        # Check if node is still responsive
        time_since_last = (
            self.get_clock().now() - self.last_inference_time
        ).nanoseconds / 1e9

        if time_since_last > 10.0:  # No inference in 10 seconds
            self.get_logger().warn('No inferences in last 10 seconds')

        self.get_logger().info(f'Health: {self.inference_count} inferences, '
                              f'last {time_since_last:.1f}s ago')
```

## Summary

Python-ROS2 integration using rclpy enables powerful AI applications in robotics. By understanding the architecture, implementing proper message handling strategies, and following performance optimization techniques, you can create robust and efficient AI-ROS2 systems. The key is to balance real-time requirements with computational complexity while maintaining system reliability.