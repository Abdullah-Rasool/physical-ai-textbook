# Distributed Robot Control: Multi-Node Systems

---

## Introduction

Modern robots are complex systems with many subsystems: cameras, LiDAR, IMUs, motor controllers, AI accelerators, and more. Running all of this on a single processor is often impractical or impossible. ROS 2 is designed from the ground up for **distributed systems**—robot software that spans multiple processes, computers, and even multiple robots.

**In This Section**:
- Understand why distributed architectures are essential for modern robots
- Learn how ROS 2 enables seamless multi-machine communication
- Explore node composition and lifecycle management
- See how distributed design improves modularity, fault tolerance, and scalability

---

## Why Distributed?

### The Single-Computer Limit

Early robot systems often ran on a single computer. This approach hits limits quickly:

**Computational constraints**: Real-time control (1 kHz), perception (30 Hz vision, 10 Hz LiDAR), and AI inference (demanding GPU) compete for resources. A single CPU can't handle all of this with required latencies.

**Hardware diversity**: Cameras often connect via USB or Ethernet, motor controllers via CAN bus, AI chips via PCIe. Different hardware naturally lives on different processors.

**Real-time requirements**: Control loops need deterministic timing. Mixing real-time control with heavyweight perception on one system risks timing violations.

**Physical constraints**: Sensors and actuators are distributed across the robot body. Running cables to a central computer adds weight, complexity, and failure points.

### The Distributed Solution

ROS 2 addresses these challenges by treating distributed deployment as the default:

- **Multiple processes**: Even on one computer, nodes run in separate processes for isolation
- **Multiple computers**: Nodes communicate transparently over the network
- **Heterogeneous hardware**: Run control on a real-time microcontroller, perception on a GPU, planning on a CPU cluster

---

## Multi-Machine Communication

### Automatic Discovery

ROS 2 uses **DDS** (Data Distribution Service) for communication, which provides automatic peer-to-peer discovery:

1. **Node startup**: Each node announces its presence via multicast
2. **Discovery**: Other nodes on the same network discover it automatically
3. **Direct connection**: Once discovered, nodes communicate directly (no central broker)

This means you can:
- Start nodes on different computers without configuration
- Add new computers to the system dynamically
- Remove computers without affecting others

### Network Transparency

Nodes don't know (or care) where other nodes run. A subscriber on Computer A receives messages from a publisher on Computer B exactly the same way it would if both were on the same machine.

```python
# PURPOSE: Show that code is identical regardless of network topology
# NOTE: This is conceptual code, not production-ready

# This subscriber code works identically whether the publisher
# is in the same process, a different process, or on a different computer

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')

        # Subscribe to camera topic - works regardless of where
        # the camera node is running (same machine or remote)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        # Process image - no code changes needed for distributed deployment
        self.process_image(msg)
```

**Explanation**: The subscriber code has no references to network addresses, machine names, or any deployment topology. ROS 2 handles all networking transparently. When you move the camera node to a different computer, the subscriber code doesn't change.

### Multi-Computer Robot Architecture

**Architecture Overview**:
This diagram shows a typical distributed robot system where different computing tasks run on specialized hardware, all coordinated through ROS 2.

**Components**:

**Robot Computer (Jetson Orin)**:
- **Camera Driver Node**: Interfaces with cameras via MIPI/USB, publishes to `/camera/image`
- **LiDAR Driver Node**: Interfaces with LiDAR via Ethernet, publishes to `/scan`
- **Isaac ROS Perception**: GPU-accelerated object detection and SLAM
- **Local Topics**: High-bandwidth sensor data stays on-robot

**Edge Server (Workstation with GPU)**:
- **Planning Node**: Receives goals, computes paths, publishes plans
- **AI Inference Node**: Runs large neural networks (transformers, VLMs)
- **Simulation Bridge**: Connects to digital twin for testing
- **Remote Topics**: Commands and compressed data over WiFi/Ethernet

**Microcontroller (STM32 + micro-ROS)**:
- **Motor Controller Node**: Direct hardware PWM control
- **IMU Driver Node**: High-rate IMU data
- **Joint State Publisher**: Encoder feedback
- **Real-time Topics**: Deterministic control at 1 kHz

**Relationships**:
- Camera Driver → Perception: High-bandwidth local topic (stays on Jetson)
- Perception → Planning: Compressed data over network to edge server
- Planning → Motor Controller: Low-latency commands to microcontroller
- All nodes: Discovered automatically via DDS multicast

**Data Flow**:
1. Camera Driver captures images → publishes to `/camera/image` (local, high bandwidth)
2. Perception Node processes locally (GPU) → publishes `/detections` and `/map`
3. Planning Node (edge server) receives compressed data → computes path → publishes `/cmd_vel`
4. Motor Controller (microcontroller) receives commands → executes at 1 kHz → publishes `/joint_states`
5. All data logged to edge server for debugging

**Key Interfaces**:
- **Local topics**: Full-bandwidth on-robot communication
- **Compressed topics**: Image/pointcloud compression for network efficiency
- **micro-ROS**: ROS 2 for microcontrollers (real-time motor control)
- **DDS Domain**: All nodes in same domain discover each other automatically

**Takeaway**: ROS 2's distributed architecture allows optimal hardware placement—perception on GPU, control on real-time MCU, planning on powerful server—while maintaining seamless communication.

---

## Node Lifecycle Management

### The Lifecycle Challenge

In a distributed system, nodes start, stop, and sometimes crash. Managing this complexity requires structure:

- **Startup ordering**: Some nodes depend on others (planner needs map server)
- **Graceful shutdown**: Stop commanding motors before killing the driver
- **Error recovery**: Restart a crashed node without disrupting the system
- **State transitions**: Configure before activating, pause during calibration

### Managed Nodes

ROS 2 provides **managed nodes** (also called lifecycle nodes) with explicit states:

| State | Description |
|-------|-------------|
| **Unconfigured** | Initial state, node exists but isn't configured |
| **Inactive** | Configured but not processing (connections exist, no data flow) |
| **Active** | Fully operational, processing data |
| **Finalized** | Shutdown complete, ready for destruction |

Transitions between states are triggered explicitly:
- `configure()`: Unconfigured → Inactive
- `activate()`: Inactive → Active
- `deactivate()`: Active → Inactive
- `cleanup()`: Inactive → Unconfigured
- `shutdown()`: Any → Finalized

### Lifecycle Benefits

**Controlled startup**: A launch system can configure all nodes, verify readiness, then activate them together.

**Safe shutdown**: Deactivate nodes gracefully before destruction—motors stop before the controller dies.

**Error handling**: A node can transition to Inactive on error, attempt recovery, and reactivate.

**Hot reconfiguration**: Deactivate, reconfigure, reactivate—without full restart.

```python
# PURPOSE: Illustrate lifecycle node state transitions
# NOTE: This is conceptual code, not production-ready

from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn

class ManagedMotorController(LifecycleNode):
    """Motor controller with explicit lifecycle management"""

    def __init__(self):
        super().__init__('managed_motor_controller')
        self.motor = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configure hardware connections (Unconfigured → Inactive)"""
        self.get_logger().info('Configuring motor connection...')

        # Initialize hardware (but don't start moving yet)
        self.motor = MotorHardware()
        if not self.motor.connect():
            self.get_logger().error('Failed to connect to motor')
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info('Motor configured and connected')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Start processing commands (Inactive → Active)"""
        self.get_logger().info('Activating motor control...')

        # Enable motor and start accepting commands
        self.motor.enable()

        # Create subscription only when active
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.velocity_callback, 10
        )

        self.get_logger().info('Motor controller active')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Stop processing commands (Active → Inactive)"""
        self.get_logger().info('Deactivating motor control...')

        # Stop motors safely before deactivating
        self.motor.stop()
        self.motor.disable()

        # Remove subscription
        self.destroy_subscription(self.cmd_sub)

        self.get_logger().info('Motor controller deactivated safely')
        return TransitionCallbackReturn.SUCCESS

    def velocity_callback(self, msg):
        """Process velocity commands (only called when Active)"""
        if self.motor.is_enabled():
            self.motor.set_velocity(msg.linear.x, msg.angular.z)
```

**Explanation**: This lifecycle node explicitly separates configuration (connecting to hardware), activation (enabling motors and accepting commands), and deactivation (safely stopping before shutdown). The motor only receives commands when the node is Active, preventing accidents during startup/shutdown.

**Key Concepts Demonstrated**:
- **Explicit states**: Clear separation of configuration, activation, operation
- **Safe transitions**: Motors stop before deactivation, connect before activation
- **Callback hooks**: Override on_configure, on_activate, etc. for custom behavior

---

## Fault Tolerance

### Node Independence

Because nodes run in separate processes:

- **Crash isolation**: One node crashing doesn't corrupt others' memory
- **Independent restart**: Crashed nodes can restart without system-wide reset
- **Graceful degradation**: If perception fails, stop the robot safely

### Watchdogs and Monitoring

Distributed systems need monitoring:

- **Heartbeats**: Nodes publish periodic "I'm alive" messages
- **Watchdogs**: Other nodes detect missing heartbeats and take action
- **Health topics**: Nodes publish diagnostic information

### Example: Safe Stop on Failure

```python
# PURPOSE: Show watchdog pattern for safety in distributed systems
# NOTE: This is conceptual code, not production-ready

class SafetyWatchdog(Node):
    """Monitors critical nodes and triggers safe stop if they fail"""

    def __init__(self):
        super().__init__('safety_watchdog')

        # Track when we last heard from critical nodes
        self.last_perception_time = self.get_clock().now()
        self.last_planning_time = self.get_clock().now()

        # Subscribe to heartbeat topics
        self.create_subscription(
            Empty, '/perception/heartbeat',
            lambda msg: self.update_time('perception'), 10
        )
        self.create_subscription(
            Empty, '/planning/heartbeat',
            lambda msg: self.update_time('planning'), 10
        )

        # Check for timeouts periodically
        self.create_timer(0.1, self.check_timeouts)

        # Publisher to command safe stop
        self.stop_pub = self.create_publisher(Empty, '/emergency_stop', 10)

    def check_timeouts(self):
        """Check if any critical node has timed out"""
        now = self.get_clock().now()
        timeout = Duration(seconds=1.0)  # 1 second without heartbeat = failure

        perception_age = now - self.last_perception_time
        planning_age = now - self.last_planning_time

        if perception_age > timeout or planning_age > timeout:
            self.get_logger().error('Critical node timeout! Triggering safe stop.')
            self.stop_pub.publish(Empty())
```

**Explanation**: This watchdog node monitors heartbeats from perception and planning nodes. If either stops publishing for more than 1 second, the watchdog triggers an emergency stop. This pattern is essential for safety in distributed robot systems—you can't trust that all nodes are always running.

---

## Scalability

### Adding Capabilities

The distributed architecture scales naturally:

**New sensors**: Add a driver node. Other nodes discover it via its topics.

**More compute**: Spin up additional perception nodes on more GPUs.

**Multi-robot**: Each robot runs its own nodes, optionally sharing a planning server.

### Namespacing for Multi-Robot

ROS 2 supports **namespaces** to isolate robot-specific topics:

```text
Robot 1:                    Robot 2:
/robot1/camera/image        /robot2/camera/image
/robot1/cmd_vel             /robot2/cmd_vel
/robot1/odom                /robot2/odom
```

Same node code, different namespaces—instant multi-robot support.

---

## Summary

**Key Takeaways**:
- **Distributed by default**: ROS 2 treats multi-process, multi-machine deployment as normal
- **Automatic discovery**: DDS multicast enables nodes to find each other without configuration
- **Network transparency**: Code doesn't change when moving nodes between computers
- **Lifecycle management**: Managed nodes provide controlled startup, shutdown, and error recovery
- **Fault tolerance**: Process isolation, watchdogs, and graceful degradation enable robust systems
- **Scalability**: Add sensors, compute, or robots by adding nodes—no architectural changes needed

**Connection to Next Section**: The magic enabling all this transparent communication is **DDS middleware**. In the next section, we'll explore how DDS works and how **Quality of Service (QoS)** policies let you tune communication for different needs.

---

**[Continue to Section 4: Middleware Concepts →](./04-middleware-concepts.md)**
