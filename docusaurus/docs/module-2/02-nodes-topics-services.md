---
sidebar_position: 1
---

# Nodes, Topics, and Services in ROS 2

## Introduction

In the previous chapter, we established that ROS 2 provides the essential framework for building complex robotic systems, coordinating many individual components. But how do these components actually talk to each other? How does a camera node send its images to a vision processing node, or a navigation node request a map update? This is where **Nodes, Topics, and Services** come into play – the fundamental communication patterns that form the backbone of any ROS 2 application.

Imagine a busy orchestra. Each musician is like a **Node**, performing a specific role. When the first violinist plays a melody for everyone to hear, that's like publishing a message on a **Topic**. Everyone listening (subscribers) can hear it. If the conductor (client) asks the percussionist (server) for a specific drum roll, and waits for that specific sound before continuing, that's like a **Service** call. Each communication method serves a different purpose, ensuring that information flows efficiently and appropriately throughout the entire robotic "orchestra." Understanding these distinct communication patterns and their optimal use cases is crucial for designing robust and scalable ROS 2 systems.

## Deep Dive into ROS 2 Nodes

A **Node** is the smallest executable unit in a ROS 2 graph. Conceptually, a node is simply an instance of a program that uses the ROS 2 client library (like `rclpy` for Python or `rclcpp` for C++) to communicate with other nodes. Each node is designed to perform a single, well-defined task, promoting modularity and reusability in robot software development.

### What is a Node (Process-Level Explanation)

From an operating system perspective, each ROS 2 node is typically a separate process. This means nodes run independently of each other. If one node crashes, it doesn't necessarily bring down the entire robot system (though its failure might impact other nodes that depend on its output). This process-level isolation enhances system robustness and facilitates debugging. For example, a robot might have a node for its camera driver, another for its motor controller, and a third for a path planning algorithm. Each is a distinct executable.

### Node Lifecycle and States

ROS 2 introduces a managed lifecycle for nodes, particularly for "managed" nodes (often referred to as Lifecycle Nodes). This provides a more predictable startup and shutdown sequence, essential for mission-critical applications.
*   **Unconfigured**: Initial state after creation.
*   **Inactive**: Node is configured but not yet executing its main logic (e.g., waiting for external dependencies).
*   **Active**: Node is fully operational and executing its tasks (e.g., publishing data, processing commands).
*   **Finalized**: Node is shutting down.

While not all nodes are lifecycle nodes, understanding this concept helps in grasping the robustness ROS 2 aims for.

### Creating Nodes in Python (rclpy)

Creating a node in Python using `rclpy` is straightforward. You derive a class from `rclpy.node.Node` and implement your logic within its methods.

```python
# basic_node.py
import rclpy
from rclpy.node import Node

class MyCustomNode(Node):
    """
    A simple ROS 2 node that prints a message periodically.
    """
    def __init__(self):
        # Initialize the Node class with a unique name
        super().__init__('my_custom_node')
        self.get_logger().info('MyCustomNode has been initialized!')
        
        # Create a timer to call a callback function every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        """
        Callback function executed by the timer.
        """
        self.counter += 1
        self.get_logger().info(f'Hello from MyCustomNode! Count: {self.counter}')

def main(args=None):
    # Initialize the ROS 2 client library
    rclpy.init(args=args)
    
    # Create an instance of our custom node
    node = MyCustomNode()
    
    # Keep the node alive and processing events
    rclpy.spin(node)
    
    # Destroy the node when rclpy.spin() returns (e.g., on Ctrl+C)
    node.destroy_node()
    # Shutdown the ROS 2 client library
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Node Naming Conventions

*   **Unique Names**: Each node in the ROS 2 graph should have a unique name (e.g., `camera_driver`, `path_planner`). This is crucial for identification and debugging.
*   **Lowercase with Underscores**: Typically, node names follow a `snake_case` convention.
*   **Meaningful Names**: Choose names that clearly describe the node's function.

## Understanding Topics

**Topics** are the most common communication mechanism in ROS 2, implementing an asynchronous, **publish-subscribe** pattern. They are ideal for streaming continuous or periodic data where a publisher doesn't need to know if anyone is listening, and subscribers don't need to acknowledge receipt.

### Publisher-Subscriber Pattern Explained

*   **Publisher**: A node creates a publisher to send messages of a specific type to a named topic. It continuously "broadcasts" data without knowing or caring how many (if any) subscribers are listening.
*   **Subscriber**: A node creates a subscriber to receive messages of a specific type from a named topic. It registers a callback function that is executed every time a new message arrives on that topic.
*   **Topic Name**: A unique string identifier (e.g., `/robot/camera/image`, `/robot/cmd_vel`) that defines the channel for communication.
*   **Message Type**: The data structure (e.g., `sensor_msgs/msg/Image`, `geometry_msgs/msg/Twist`) that defines the content and format of the data being exchanged.

### When to Use Topics

Topics are best suited for:
*   **Streaming sensor data**: Camera images, LIDAR scans, IMU data.
*   **Continuous robot state updates**: Odometry, joint states.
*   **Broadcasting commands**: Velocity commands to motors.
*   **Any data that is generated frequently and does not require explicit acknowledgment from receivers.**

### Message Types and Custom Messages

ROS 2 uses strongly typed messages to ensure data consistency. Message definitions are stored in `.msg` files.
*   **Standard Message Types**: Provided by ROS 2 packages like `std_msgs` (e.g., `String`, `Int32`), `sensor_msgs` (e.g., `Image`, `Imu`), `geometry_msgs` (e.g., `Twist`, `Pose`).
*   **Custom Messages**: You can define your own message types by creating `.msg` files within your ROS 2 packages if standard types don't meet your needs.

### Topic Naming Best Practices

*   **Hierarchical Structure**: Use `/` to create a hierarchical namespace (e.g., `/robot_name/sensor_type/data`).
*   **Lowercase with Underscores**: Like nodes, topics typically use `snake_case`.
*   **Descriptive**: The name should clearly indicate the data being transmitted.

### Code Examples:

#### Simple Publisher Example (publishing robot sensor data)

Let's expand on our `basic_node.py` to publish simulated IMU data.

```python
# imu_publisher.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu # Import IMU message type
from geometry_msgs.msg import Quaternion, Vector3
import random
import math

class ImuPublisher(Node):
    """
    A ROS 2 node that publishes simulated IMU data to a topic.
    """
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, '/robot/imu_data', 10)
        self.timer_period = 0.1 # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('IMU Publisher node started. Publishing to /robot/imu_data')

    def timer_callback(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link' # Frame where sensor is located

        # Simulate orientation (quaternion)
        roll = random.uniform(-0.1, 0.1)
        pitch = random.uniform(-0.1, 0.1)
        yaw = random.uniform(-0.1, 0.1)
        
        # Convert Euler to Quaternion for simulation purposes
        # (Simplified conversion, real one is more complex)
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

        msg.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        msg.orientation_covariance[0] = 0.0025 # Example noise

        # Simulate angular velocity (radians/s)
        msg.angular_velocity = Vector3(
            x=random.uniform(-0.01, 0.01),
            y=random.uniform(-0.01, 0.01),
            z=random.uniform(-0.01, 0.01)
        )
        msg.angular_velocity_covariance[0] = 0.0025

        # Simulate linear acceleration (m/s^2)
        msg.linear_acceleration = Vector3(
            x=random.uniform(-0.05, 0.05),
            y=random.uniform(-0.05, 0.05),
            z=9.81 + random.uniform(-0.05, 0.05) # Gravity + noise
        )
        msg.linear_acceleration_covariance[0] = 0.0025

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing IMU data (Orientation W: {msg.orientation.w:.2f})')

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = ImuPublisher()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Simple Subscriber Example (receiving sensor data)

This node will listen to the `/robot/imu_data` topic and print a summary.

```python
# imu_subscriber.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuSubscriber(Node):
    """
    A ROS 2 node that subscribes to IMU data and prints it.
    """
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/robot/imu_data',
            self.listener_callback,
            10 # QoS history depth
        )
        self.get_logger().info('IMU Subscriber node started. Subscribing to /robot/imu_data')

    def listener_callback(self, msg):
        self.get_logger().info(
            f'Received IMU data from {msg.header.frame_id}: '
            f'Orientation (w:{msg.orientation.w:.2f}, x:{msg.orientation.x:.2f}, y:{msg.orientation.y:.2f}, z:{msg.orientation.z:.2f}) '
            f'Accel (x:{msg.linear_acceleration.x:.2f}, y:{msg.linear_acceleration.y:.2f}, z:{msg.linear_acceleration.z:.2f})'
        )

def main(args=None):
    rclpy.init(args=args)
    imu_subscriber = ImuSubscriber()
    rclpy.spin(imu_subscriber)
    imu_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Working with Services

**Services** provide a synchronous, **request-response** communication pattern. This means a client sends a request to a server and then typically blocks (waits) until it receives a response. Services are suitable for one-off data exchanges or for triggering specific actions that provide an immediate result.

### Request-Response Pattern

*   **Service Server**: A node that "advertises" a service of a specific type. It waits for requests from clients and executes a callback function to process the request and generate a response.
*   **Service Client**: A node that sends a request message to a service server and waits for a response message.
*   **Service Type**: Defined by a `.srv` file, which specifies both the request message and the response message.

### When to Use Services vs Topics

*   **Use Services when**:
    *   You need a direct, one-time interaction.
    *   You need an immediate response to a specific query.
    *   The operation is short-lived and doesn't require continuous feedback.
    *   Examples: "Get current robot pose," "Set motor speed to X," "Trigger a single camera capture."
*   **Use Topics when**:
    *   You need to broadcast continuous streams of data.
    *   The publisher doesn't need to know if anyone is listening.
    *   Low latency, high-frequency data updates are crucial.
    *   Examples: Sensor streams, odometry updates, continuous command velocity.

### Service Types (.srv files)

Service definitions are stored in `.srv` files. A `.srv` file is split into a request section and a response section by a `---` separator.

**Example: `robot_control/srv/SetJointAngle.srv`**
```
# Request
float32 joint_id
float32 angle
---
# Response
bool success
string message
```

### Synchronous vs Asynchronous Service Calls

*   **Synchronous**: The client sends a request and pauses its execution until it receives a response or a timeout occurs. This is simpler to program but can block the client node.
*   **Asynchronous**: The client sends a request and continues its execution immediately, receiving the response later via a future or a callback. This is more complex but prevents the client from blocking. `rclpy` services are primarily asynchronous.

### Code Examples:

#### Service Server Implementation (e.g., robot arm control service)

Let's create a service server that sets a simulated joint angle.

```python
# joint_angle_server.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # Using a standard service for simplicity

class MinimalService(Node):
    """
    A ROS 2 service server that adds two integers.
    """
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Service server for "add_two_ints" started.')

    def add_two_ints_callback(self, request, response):
        """
        Callback function for the service.
        It receives a request and populates the response.
        """
        # In a real robot, this would control a joint
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}')
        self.get_logger().info(f'Sending response: sum={response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Service Client Implementation

This client will call the `add_two_ints` service.

```python
# joint_angle_client.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # Import the service type

class MinimalClientAsync(Node):
    """
    A ROS 2 service client that calls the "add_two_ints" service asynchronously.
    """
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        
        # Wait until the service is available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                return
            self.get_logger().info('Service not available, waiting again...')
        
        self.req = AddTwoInts.Request() # Create a request object

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req) # Call the service asynchronously
        self.get_logger().info(f'Requesting to add: {a} + {b}')
        return self.future

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    
    # Example usage
    a = 5
    b = 7
    future = minimal_client.send_request(a, b)
    
    # Spin until the future is complete
    rclpy.spin_until_future_complete(minimal_client, future)
    
    if future.result() is not None:
        minimal_client.get_logger().info(f'Result of {a} + {b} = {future.result().sum}')
    else:
        minimal_client.get_logger().error(f'Service call failed for {a} + {b}')
    
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Comparison Table: Topics vs Services

| Feature            | Topics                                          | Services                                        |
| :----------------- | :---------------------------------------------- | :---------------------------------------------- |
| **Pattern**        | Publish-Subscribe (asynchronous)                | Request-Response (synchronous-like)             |
| **Communication**  | One-way streaming, many-to-many                 | Two-way, one-to-one                             |
| **Use Case**       | Continuous data streams, broadcasting           | One-off queries, triggering immediate actions   |
| **Blocking**       | Non-blocking (publisher doesn't wait)           | Blocking (client waits for response) or async   |
| **Feedback**       | No direct feedback/acknowledgment               | Direct response is the feedback                 |
| **Data Flow**      | Unidirectional                                  | Bidirectional (request and response)            |
| **Example**        | IMU data, camera feeds, odometry                | Get current pose, set motor joint angle, trigger map save |

## Multi-Node Communication Example

Let's illustrate how multiple nodes can interact in a practical scenario: a robot sensor system. We'll use three nodes:

*   **`SensorNode` (Publisher)**: Publishes simulated IMU data to `/robot/imu_data`.
*   **`ProcessingNode` (Subscriber + Service Server)**: Subscribes to `/robot/imu_data`, processes it (e.g., checks for anomalies), and offers a service `/robot/get_imu_stats` to provide current statistics.
*   **`ControlNode` (Subscriber + Service Client)**: Subscribes to processed data from `ProcessingNode` (if it published it) and periodically calls `ProcessingNode`'s service to get IMU stats.

This example highlights a common robotics pattern where raw sensor data is processed by a dedicated node, and its derived information or specific queries are handled by other communication mechanisms.

### Code for 3-Node System:

**(1) Sensor Node (publisher.py - similar to imu_publisher.py but for clarity in this example)**

```python
# multi_node_example/sensor_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
import random
import math

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.publisher_ = self.create_publisher(Imu, '/robot/imu_data', 10)
        self.timer_period = 0.1 # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('SensorNode started. Publishing IMU data.')
        self.sim_orientation = [0.0, 0.0, 0.0, 1.0] # w,x,y,z
        self.sim_accel = [0.0, 0.0, 9.81]

    def timer_callback(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Simulate small random changes to orientation and acceleration
        roll_change = random.uniform(-0.005, 0.005)
        pitch_change = random.uniform(-0.005, 0.005)
        yaw_change = random.uniform(-0.005, 0.005)

        # Simplified update (for demonstration, not physical accuracy)
        self.sim_orientation[1] += roll_change # x
        self.sim_orientation[2] += pitch_change # y
        self.sim_orientation[3] += yaw_change # z
        # Normalize quaternion (simplified)
        norm = math.sqrt(sum([x*x for x in self.sim_orientation]))
        self.sim_orientation = [x / norm for x in self.sim_orientation]
        msg.orientation = Quaternion(w=self.sim_orientation[0], x=self.sim_orientation[1], y=self.sim_orientation[2], z=self.sim_orientation[3])

        self.sim_accel[0] += random.uniform(-0.01, 0.01)
        self.sim_accel[1] += random.uniform(-0.01, 0.01)
        msg.linear_acceleration = Vector3(x=self.sim_accel[0], y=self.sim_accel[1], z=self.sim_accel[2])

        self.publisher_.publish(msg)
        self.get_logger().debug(f'Published IMU: w={msg.orientation.w:.2f}, z-accel={msg.linear_acceleration.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**(2) Processing Node (processing_node.py - subscriber + service server)**

```python
# multi_node_example/processing_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger # Using a standard service for simplicity
from geometry_msgs.msg import Quaternion, Vector3
import statistics

class ProcessingNode(Node):
    def __init__(self):
        super().__init__('processing_node')
        self.subscription = self.create_subscription(
            Imu,
            '/robot/imu_data',
            self.imu_callback,
            10
        )
        # Service server to provide IMU stats
        self.srv = self.create_service(Trigger, '/robot/get_imu_stats', self.get_imu_stats_callback)
        self.get_logger().info('ProcessingNode started. Subscribing to /robot/imu_data and offering service /robot/get_imu_stats')

        self.imu_data_buffer = [] # Store recent IMU data for stats
        self.buffer_size = 100 # Keep last 100 messages

    def imu_callback(self, msg):
        # Process incoming IMU data (e.g., check for anomalies, kalman filter, etc.)
        # For this example, just store it
        self.imu_data_buffer.append(msg)
        if len(self.imu_data_buffer) > self.buffer_size:
            self.imu_data_buffer.pop(0) # Remove oldest data
        self.get_logger().debug(f'Processed IMU msg (buffer size: {len(self.imu_data_buffer)})')

    def get_imu_stats_callback(self, request, response):
        """
        Service callback to provide statistics of buffered IMU data.
        """
        if not self.imu_data_buffer:
            response.success = False
            response.message = "No IMU data received yet."
            self.get_logger().warn('No IMU data in buffer for stats request.')
            return response
        
        # Calculate some stats (e.g., average Z-acceleration)
        z_accel_values = [msg.linear_acceleration.z for msg in self.imu_data_buffer]
        avg_z_accel = statistics.mean(z_accel_values)
        
        response.success = True
        response.message = (
            f"IMU Stats: Average Z-Acceleration over {len(self.imu_data_buffer)} samples = {avg_z_accel:.2f} m/s^2"
        )
        self.get_logger().info(response.message)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**(3) Control Node (control_node.py - subscriber + service client)**

```python
# multi_node_example/control_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu # Assuming it also subscribes to processed IMU from ProcessingNode
from std_srvs.srv import Trigger # For calling the service

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        # Subscriber (if ProcessingNode published processed data)
        # self.subscription_processed_imu = self.create_subscription(
        #     Imu,
        #     '/robot/processed_imu_data', # Assuming processing_node publishes this
        #     self.processed_imu_callback,
        #     10
        # )

        # Service client
        self.cli = self.create_client(Trigger, '/robot/get_imu_stats')
        self.get_logger().info('ControlNode started. Waiting for service /robot/get_imu_stats...')

        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for service.')
                return
            self.get_logger().info('Service not available, waiting again...')
        
        self.timer = self.create_timer(5.0, self.timer_callback) # Call service every 5 seconds
        self.get_logger().info('Service /robot/get_imu_stats is available. ControlNode is active.')

    # def processed_imu_callback(self, msg):
    #     self.get_logger().info(f'ControlNode received processed IMU: z-accel={msg.linear_acceleration.z:.2f}')
    #     # Logic to react to processed IMU data

    def timer_callback(self):
        """
        Timer callback to periodically request IMU stats from the ProcessingNode.
        """
        self.get_logger().info('Requesting IMU stats from ProcessingNode...')
        request = Trigger.Request()
        self.future = self.cli.call_async(request)
        self.future.add_done_callback(self.service_response_callback) # Handle response asynchronously

    def service_response_callback(self, future):
        """
        Callback to handle the response from the service.
        """
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Received IMU stats: {response.message}')
            else:
                self.get_logger().warn(f'Failed to get IMU stats: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Mermaid Diagram Showing Multi-Node Communication:

```mermaid
graph TD
    A[SensorNode] --> |/robot/imu_data (Imu)| B[ProcessingNode]
    B --> |/robot/get_imu_stats (Trigger Service)| C[ControlNode]
    C -- Calls periodically --> B
    style A fill:#D2E9FF,stroke:#3498DB
    style B fill:#FFF3CD,stroke:#F39C12
    style C fill:#E0FFEE,stroke:#2ECC71
```

## Best Practices

Adhering to best practices in ROS 2 development is crucial for creating maintainable, scalable, and robust robotic systems.

### Naming Conventions
*   **Nodes**: `snake_case`, descriptive (e.g., `camera_driver`, `path_planner`).
*   **Topics**: Hierarchical `snake_case`, starting with `/`, descriptive (e.g., `/robot_name/sensor_type/data`).
*   **Services**: Hierarchical `snake_case`, descriptive (e.g., `/robot_name/manipulation/set_joint_angle`).
*   **Message/Service Types**: `PascalCase` for type names, `snake_case` for field names.

### Error Handling
*   **`rclpy.ok()` checks**: Always check `rclpy.ok()` in loops (e.g., in `wait_for_service`) to ensure the ROS 2 context is still valid.
*   **Exceptions**: Use `try-except` blocks for external calls or potentially failing operations (e.g., `future.result()`).
*   **Logging**: Use `self.get_logger().info()`, `warn()`, `error()`, `debug()` for informative output.

### Node Organization
*   **Single Responsibility Principle**: Each node should ideally do one thing well.
*   **Small, Modular Nodes**: Easier to test, debug, and reuse.
*   **Separation of Concerns**: Keep hardware drivers, algorithms, and high-level control in distinct nodes.

### Performance Considerations
*   **QoS Profiles**: Carefully select QoS profiles for topics based on data importance and frequency (Reliable for commands, Best-Effort for high-frequency sensor streams).
*   **Efficient Messages**: Use the smallest message types possible. Avoid sending large amounts of redundant data.
*   **Asynchronous Operations**: For services or long-running computations, use asynchronous calls to prevent your node from blocking.

## Debugging Tools

ROS 2 provides a powerful set of command-line tools to inspect and debug your running system.

### `ros2 topic` commands

*   **`ros2 topic list`**: Lists all active topics in the ROS 2 graph.
    ```bash
    ros2 topic list
    # Expected output:
    # /parameter_events
    # /robot/imu_data
    # /rosout
    ```
*   **`ros2 topic info <topic_name>`**: Shows information about a specific topic, including its type and connected publishers/subscribers.
    ```bash
    ros2 topic info /robot/imu_data
    # Expected output:
    # Type: sensor_msgs/msg/Imu
    # Publisher count: 1
    # Subscriber count: 1
    ```
*   **`ros2 topic echo <topic_name>`**: Displays messages being published on a topic in real-time.
    ```bash
    ros2 topic echo /robot/imu_data
    # Expected output (streaming IMU messages):
    # header:
    #   stamp:
    #     sec: 1678886400
    #     nanosec: 123456789
    #   frame_id: imu_link
    # orientation:
    #   x: 0.0
    #   y: 0.0
    #   z: 0.0
    #   w: 1.0
    # ...
    ```
*   **`ros2 topic bw <topic_name>`**: Shows bandwidth usage of a topic.
*   **`ros2 topic hz <topic_name>`**: Shows the publishing frequency of a topic.

### `ros2 service` commands

*   **`ros2 service list`**: Lists all active services.
    ```bash
    ros2 service list
    # Expected output:
    # /robot/get_imu_stats
    # /minimal_service/add_two_ints
    # ... (other internal services)
    ```
*   **`ros2 service info <service_name>`**: Shows information about a service, including its type.
    ```bash
    ros2 service info /robot/get_imu_stats
    # Expected output:
    # Type: std_srvs/srv/Trigger
    ```
*   **`ros2 service call <service_name> <service_type> <request_args>`**: Calls a service with specified arguments.
    ```bash
    ros2 service call /minimal_service/add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 7}"
    # Expected output:
    # requester: ...
    # response:
    #   sum: 12
    ```

### `ros2 node` commands

*   **`ros2 node list`**: Lists all active nodes.
    ```bash
    ros2 node list
    # Expected output:
    # /control_node
    # /processing_node
    # /sensor_node
    ```
*   **`ros2 node info <node_name>`**: Shows information about a node, including its publishers, subscribers, services, and actions.
    ```bash
    ros2 node info /processing_node
    # Expected output (showing connections for processing_node):
    # /processing_node
    #   Subscribers:
    #     /robot/imu_data: sensor_msgs/msg/Imu
    #   Publishers:
    #     /parameter_events: rcl_interfaces/msg/ParameterEvent
    #     /rosout: rcl_interfaces/msg/Log
    #   Service Servers:
    #     /robot/get_imu_stats: std_srvs/srv/Trigger
    #   Service Clients:
    #     (none)
    #   Action Servers:
    #     (none)
    #   Action Clients:
    #     (none)
    ```

## Hands-on Exercises

These exercises will solidify your understanding of ROS 2 communication.

### Exercise 1: Create IMU Sensor Publisher

**Goal**: Create a ROS 2 Python node that publishes simulated IMU data.
**Instructions**:
1.  Create a new ROS 2 Python package (e.g., `my_sensors`).
2.  Implement the `imu_publisher.py` code provided earlier in this chapter.
3.  Modify `setup.py` to add an entry point for your `imu_publisher`.
4.  Build and run your `imu_publisher` node.
5.  Use `ros2 topic list`, `ros2 topic info /robot/imu_data`, and `ros2 topic echo /robot/imu_data` to verify it's working.
**Solution Hints**: Refer to the "Practical Example" section for package creation and `setup.py` modification.

### Exercise 2: Create Temperature Monitoring Service

**Goal**: Create a ROS 2 Python service server that provides a simulated temperature reading upon request, and a client that calls it.
**Instructions**:
1.  In your `my_sensors` package, define a new service `Temperature.srv` (e.g., `request: string location --- response: float32 temperature`).
2.  Implement a service server node that returns a random temperature for the requested location.
3.  Implement a service client node that calls this service for "engine" temperature.
4.  Modify `setup.py` for new entry points and add `install/share/<pkg_name>/srv` for the service definition.
5.  Build and test your service server and client.
**Solution Hints**: You'll need to learn how to define and use custom `.srv` files and add them to `setup.py` and `CMakeLists.txt` (for service generation). This is an advanced step not fully covered in this chapter, but a good challenge!

### Exercise 3: Multi-node Robot Control System

**Goal**: Set up and run the full 3-node system (Sensor, Processing, Control) demonstrated in the "Multi-Node Communication Example" section.
**Instructions**:
1.  Create a ROS 2 Python package `robot_control_system`.
2.  Place `sensor_node.py`, `processing_node.py`, and `control_node.py` in the package.
3.  Modify `setup.py` to create entry points for all three nodes.
4.  Build the package.
5.  Open three separate terminals, source your workspace in each, and run each node.
6.  Use `ros2 node list`, `ros2 topic list`, `ros2 service list`, and `rqt_graph` to observe the communication.
**Solution Hints**: Pay close attention to the `setup.py` modifications and ensure `std_srvs` is a dependency if using `Trigger.srv`.

## Common Pitfalls

:::warning
*   **Incorrect QoS Settings**: Mismatched QoS policies between publishers and subscribers can prevent communication entirely or lead to dropped messages. Always ensure compatible QoS for critical data.
*   **Incorrect Topic/Service Names**: Typos or extra `/` in names will prevent nodes from connecting. Use `ros2 topic list`/`ros2 service list` to verify.
*   **Missing `rclpy.spin()`**: Forgetting to call `rclpy.spin(node)` will prevent your node from processing callbacks (topic messages, service requests, timer events).
*   **Uninitialized ROS 2 Context**: Forgetting `rclpy.init()` or `rclpy.shutdown()` can lead to unexpected behavior or resource leaks.
*   **`setup.py` Errors**: Incorrect `entry_points` or missing `data_files` for messages/services will prevent your executables or custom types from being found.
*   **Blocking in Callbacks**: Performing long computations directly within a callback function can block the node's executor, causing messages to pile up or services to time out. Use executors or separate threads for heavy lifting.
:::

## Summary and Next Steps
    - Key concepts recap
    - Preview of next chapter (Python packages)

---
[Previous Chapter: ROS 2 Architecture and Core Concepts](./01-ros2-architecture.md)
[Next Chapter: Gazebo Simulation Environment and Robot Description Formats](../module-3/01-gazebo-setup-urdf.md)
