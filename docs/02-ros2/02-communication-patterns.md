# Communication Patterns: Publish-Subscribe and Request-Reply

---

## Introduction

In the previous section, we introduced topics, services, and actions as the three communication mechanisms in ROS 2. Now we'll explore the two fundamental patterns—**publish-subscribe** and **request-reply**—in detail. Understanding when and how to use each pattern is essential for designing effective robot systems.

**In This Section**:
- Master the publish-subscribe pattern for streaming data
- Understand the request-reply pattern for synchronous queries
- See complete code examples for publishers, subscribers, servers, and clients
- Learn to choose the right pattern for different communication needs

---

## Publish-Subscribe Pattern

### The Concept

Publish-subscribe (pub/sub) is an **asynchronous, many-to-many** communication pattern:

- **Publishers** send messages without knowing who (if anyone) is listening
- **Subscribers** receive messages without knowing who sent them
- **Topics** are the named channels that connect publishers and subscribers
- **Decoupling**: Publishers and subscribers don't know about each other directly

This decoupling is powerful: you can add new subscribers without modifying publishers, and publishers keep publishing even if no one is listening (useful during development).

### When to Use Topics

Use topics for:
- **Continuous data streams**: Sensor data that flows constantly (images, LiDAR scans, IMU readings)
- **State broadcasting**: Robot pose, joint states, battery level
- **Command streams**: Velocity commands, trajectory setpoints
- **Events**: Notifications that multiple nodes might care about

### Example: Publisher Node

Here's a complete example of a publisher that sends velocity commands:

```python
# PURPOSE: Demonstrate ROS 2 topic publishing with Twist messages
# NOTE: This is conceptual code, not production-ready

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):
    """Publishes velocity commands to control robot motion"""

    def __init__(self):
        super().__init__('velocity_publisher')

        # Create publisher for velocity commands
        # Topic: /cmd_vel (standard name for velocity commands)
        # Message type: geometry_msgs/Twist (linear + angular velocity)
        # Queue size: 10 (buffer this many messages if subscriber is slow)
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Create timer to publish at 10 Hz (every 0.1 seconds)
        self.timer = self.create_timer(0.1, self.publish_velocity)

        self.get_logger().info('Velocity publisher started')

    def publish_velocity(self):
        """Timer callback: publish velocity command"""
        msg = Twist()

        # Set linear velocity (m/s): move forward at 0.5 m/s
        msg.linear.x = 0.5
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        # Set angular velocity (rad/s): turn slowly left
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.1

        # Publish the message
        self.publisher.publish(msg)

        # Log (optional, can be noisy at high rates)
        self.get_logger().debug(f'Published: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

def main():
    rclpy.init()
    node = VelocityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Explanation**: This publisher sends velocity commands at 10 Hz to the `/cmd_vel` topic. The `Twist` message type is standard in ROS 2 for expressing 6-DOF velocity (3 linear + 3 angular). Any motor controller subscribing to `/cmd_vel` will receive these commands—the publisher doesn't need to know what's listening.

**Key Concepts Demonstrated**:
- **create_publisher()**: Advertises intent to publish on a topic
- **Timer callbacks**: Common pattern for periodic publishing
- **Standard messages**: Using `geometry_msgs/Twist` ensures interoperability

### Example: Subscriber Node

Here's the corresponding subscriber:

```python
# PURPOSE: Demonstrate ROS 2 topic subscription with callback processing
# NOTE: This is conceptual code, not production-ready

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocitySubscriber(Node):
    """Subscribes to velocity commands and processes them"""

    def __init__(self):
        super().__init__('velocity_subscriber')

        # Create subscription to /cmd_vel topic
        # callback_function is called whenever a message arrives
        self.subscription = self.create_subscription(
            Twist,                      # Message type to expect
            '/cmd_vel',                 # Topic to subscribe to
            self.velocity_callback,     # Function to call on message
            10                          # Queue size
        )

        self.get_logger().info('Velocity subscriber started')

    def velocity_callback(self, msg):
        """Called whenever a Twist message is received"""
        linear = msg.linear.x
        angular = msg.angular.z

        # Process the velocity command (in a real robot, this would
        # convert to motor commands and send to hardware)
        self.get_logger().info(
            f'Received velocity: linear={linear:.2f} m/s, angular={angular:.2f} rad/s'
        )

        # Example processing: calculate wheel velocities for differential drive
        # (conceptual - actual calculation depends on robot kinematics)
        wheel_base = 0.5  # meters between wheels
        left_wheel = linear - (angular * wheel_base / 2)
        right_wheel = linear + (angular * wheel_base / 2)

        self.get_logger().debug(f'Wheel velocities: L={left_wheel:.2f}, R={right_wheel:.2f}')

def main():
    rclpy.init()
    node = VelocitySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Explanation**: This subscriber listens to `/cmd_vel` and processes incoming velocity commands. The `velocity_callback` function is automatically called by ROS 2 whenever a message arrives. The subscriber doesn't know (or care) who is publishing—it just reacts to messages.

**Key Concepts Demonstrated**:
- **create_subscription()**: Registers interest in a topic with a callback
- **Callback pattern**: Message processing happens asynchronously in callbacks
- **Decoupling**: Subscriber logic is independent of publisher implementation

---

## Request-Reply Pattern

### The Concept

Request-reply (also called client-server) is a **synchronous, one-to-one** communication pattern:

- **Client** sends a request and **blocks** until a response arrives
- **Server** receives the request, processes it, and sends a response
- **Services** are the named interfaces that define request and response types

This is similar to a function call across process boundaries: you send inputs and wait for outputs.

### When to Use Services

Use services for:
- **Configuration**: Getting or setting parameters
- **One-time commands**: "Save the map", "Reset the arm", "Calibrate sensors"
- **Queries**: "What's the current robot pose?", "Is the gripper open?"
- **Discrete actions**: Operations that complete quickly and need confirmation

**Don't use services for**:
- Continuous data streams (use topics)
- Long-running operations (use actions)
- Fire-and-forget commands (use topics with acknowledgment if needed)

### Example: Service Server

Here's a service that adds two integers (simple example to show the pattern):

```python
# PURPOSE: Demonstrate ROS 2 service server pattern
# NOTE: This is conceptual code, not production-ready

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AdditionServer(Node):
    """Service server that adds two integers"""

    def __init__(self):
        super().__init__('addition_server')

        # Create service server
        # Service name: /add_two_ints
        # Service type: AddTwoInts (defines request and response structure)
        # Callback: handle_add_request (called when request arrives)
        self.service = self.create_service(
            AddTwoInts,
            '/add_two_ints',
            self.handle_add_request
        )

        self.get_logger().info('Addition service ready')

    def handle_add_request(self, request, response):
        """Process incoming service request"""
        # Extract values from request
        a = request.a
        b = request.b

        # Compute result
        result = a + b

        # Fill response
        response.sum = result

        self.get_logger().info(f'Request: {a} + {b} = {result}')

        # Return response (ROS 2 sends it back to client)
        return response

def main():
    rclpy.init()
    node = AdditionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Explanation**: This server offers the `/add_two_ints` service. When a client calls this service with two integers, the `handle_add_request` callback computes the sum and returns it. The service type (`AddTwoInts`) defines the structure of both request (two integers) and response (one integer).

### Example: Service Client

Here's the corresponding client:

```python
# PURPOSE: Demonstrate ROS 2 service client pattern with synchronous call
# NOTE: This is conceptual code, not production-ready

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AdditionClient(Node):
    """Service client that requests addition"""

    def __init__(self):
        super().__init__('addition_client')

        # Create service client
        self.client = self.create_client(
            AddTwoInts,
            '/add_two_ints'
        )

        # Wait for service to become available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

        self.get_logger().info('Service available')

    def send_request(self, a, b):
        """Send addition request and wait for response"""
        # Create request message
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # Call service (asynchronously internally, but we wait for result)
        future = self.client.call_async(request)

        # Wait for response (blocks until server responds)
        rclpy.spin_until_future_complete(self, future)

        # Extract result
        response = future.result()
        return response.sum

def main():
    rclpy.init()
    client = AdditionClient()

    # Make a service call
    result = client.send_request(3, 5)
    print(f'Result: 3 + 5 = {result}')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Explanation**: This client waits for the service to be available, then sends a request with two integers. The `call_async` method sends the request, and `spin_until_future_complete` blocks until the server responds. Unlike topics, the client explicitly waits for and receives a response.

**Key Concepts Demonstrated**:
- **wait_for_service()**: Block until server is ready (important for startup ordering)
- **call_async()**: Non-blocking call that returns a future
- **spin_until_future_complete()**: Wait for the future to resolve

---

## Choosing the Right Pattern

### Decision Guide

Use this flowchart to choose between topics and services:

1. **Is data continuous/streaming?** → Use **Topic**
   - Examples: camera images, sensor readings, velocity commands

2. **Is it a one-time query or command?** → Use **Service**
   - Examples: "What's the battery level?", "Save the configuration"

3. **Does the caller need to know when it's done?**
   - **Quick operation (< 1 second)**: Use **Service**
   - **Long operation with progress**: Use **Action** (covered in advanced topics)

4. **Can multiple receivers benefit from the data?** → Use **Topic**
   - Example: Robot pose used by planner, visualizer, logger

5. **Is strict request-response semantics needed?** → Use **Service**
   - Example: Configuration changes that need acknowledgment

### Common Patterns in Robot Systems

| Data Type | Pattern | Topic/Service Name | Rationale |
|-----------|---------|-------------------|-----------|
| Camera images | Topic | `/camera/image_raw` | Continuous stream, multiple consumers |
| Velocity commands | Topic | `/cmd_vel` | Continuous stream, 10-100 Hz |
| Robot pose | Topic | `/odom` | Continuous state update |
| Get parameters | Service | `/get_parameters` | One-time query, needs response |
| Trigger calibration | Service | `/calibrate` | One-time command, needs confirmation |
| Navigate to goal | Action | `/navigate_to_pose` | Long-running, needs feedback |

---

## Message Types

### Standard Messages

ROS 2 provides standard message types for common data:

- **geometry_msgs**: Poses, transforms, velocities, wrenches
- **sensor_msgs**: Images, point clouds, IMU data, joint states
- **std_msgs**: Basic types (String, Int32, Float64, etc.)
- **nav_msgs**: Odometry, occupancy grids, paths

Using standard messages ensures your nodes work with the broader ROS 2 ecosystem.

### Custom Messages

For domain-specific data, you can define custom messages:

```text
# Example: custom message for detected objects
# File: my_msgs/msg/Detection.msg

string class_name          # "person", "car", "chair"
float32 confidence         # 0.0 to 1.0
int32 x                    # bounding box x
int32 y                    # bounding box y
int32 width                # bounding box width
int32 height               # bounding box height
```

Custom messages are compiled into Python/C++ classes that can be used like standard messages.

---

## Summary

**Key Takeaways**:
- **Publish-subscribe** (topics) is asynchronous, many-to-many, ideal for streaming data
- **Request-reply** (services) is synchronous, one-to-one, ideal for queries and commands
- **Publishers** send without waiting; **subscribers** receive via callbacks
- **Clients** send requests and block; **servers** process and respond
- **Standard messages** enable interoperability; **custom messages** handle domain-specific data
- **Pattern choice** depends on data characteristics: continuous vs one-time, many consumers vs single response

**Connection to Next Section**: Now that you understand how individual nodes communicate, we'll zoom out to see how **distributed robot control** works—how ROS 2 enables systems that span multiple processes, computers, and even robots.

---

**[Continue to Section 3: Distributed Robot Control →](./03-distributed-control.md)**
