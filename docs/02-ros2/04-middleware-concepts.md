# Middleware Concepts: DDS and Quality of Service

---

## Introduction

In previous sections, we discussed how ROS 2 nodes communicate via topics and services. But what actually delivers those messages across processes and networks? The answer is **DDS** (Data Distribution Service)—the middleware layer that powers ROS 2 communication. Understanding DDS concepts, particularly **Quality of Service (QoS)** policies, is essential for building reliable robot systems.

**In This Section**:
- Understand what DDS middleware provides and why ROS 2 chose it
- Learn the key QoS policies that affect robot communication
- See how to configure QoS for different scenarios (sensors, commands, logging)
- Understand the trade-offs between reliability, latency, and resource usage

---

## What Is DDS?

### Data Distribution Service

**DDS** (Data Distribution Service) is an industry-standard middleware protocol developed by the Object Management Group (OMG). It's used in mission-critical systems including:

- Military systems (radar, weapons)
- Financial trading platforms
- Air traffic control
- Medical devices
- And now: robots via ROS 2

DDS provides:

- **Publish-subscribe communication**: The foundation for ROS 2 topics
- **Automatic discovery**: Nodes find each other without configuration
- **Quality of Service**: Fine-grained control over delivery guarantees
- **Security**: Authentication, encryption, access control (DDS-Security)
- **Interoperability**: Different DDS implementations can communicate

### Why DDS for ROS 2?

ROS 1 used a custom protocol (TCPROS) that had limitations:

| ROS 1 (TCPROS) | ROS 2 (DDS) |
|----------------|-------------|
| Custom protocol | Industry standard |
| Centralized (rosmaster) | Decentralized (peer-to-peer) |
| No QoS control | Rich QoS policies |
| No built-in security | DDS-Security |
| Limited real-time | Real-time capable |

By adopting DDS, ROS 2 gets battle-tested networking for free, with decades of development and optimization.

### DDS Implementations

ROS 2 supports multiple DDS implementations (called **RMW**—ROS Middleware):

- **Fast DDS** (default): Open-source, actively developed by eProsima
- **Cyclone DDS**: Open-source, Eclipse Foundation project
- **RTI Connext**: Commercial, high-performance, used in defense
- **GurumDDS**: Commercial, Korean robotics market

You can switch implementations without changing your code—they all speak the same DDS protocol.

---

## Quality of Service (QoS)

### The Need for QoS

Not all data has the same requirements:

- **Camera images**: High bandwidth, some loss acceptable (skip a frame, no big deal)
- **Emergency stop commands**: Must be delivered, even if slow
- **Joint commands**: Low latency critical (stale commands are dangerous)
- **Log messages**: Persist for debugging, not time-critical

**Quality of Service (QoS)** policies let you configure how DDS handles each topic to match these requirements.

### Key QoS Policies

#### 1. Reliability

**Reliable**: Guarantees delivery. If a message is lost, DDS retransmits it.
- Use for: Commands, configuration, safety-critical data
- Trade-off: Higher latency (waits for acknowledgment)

**Best Effort**: No delivery guarantee. Lost messages are gone.
- Use for: Sensor streams where latest data matters most
- Trade-off: May lose messages under congestion

```python
# PURPOSE: Show reliability QoS configuration
# NOTE: This is conceptual code, not production-ready

from rclpy.qos import QoSProfile, ReliabilityPolicy

# Reliable: guaranteed delivery (use for commands)
reliable_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    depth=10
)

# Best effort: may lose messages (use for sensor streams)
best_effort_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=1
)

# Create publisher with specific QoS
cmd_pub = node.create_publisher(Twist, '/cmd_vel', reliable_qos)
image_pub = node.create_publisher(Image, '/camera/image', best_effort_qos)
```

#### 2. Durability

**Volatile**: Only delivers to current subscribers. Past messages are gone.
- Use for: Real-time sensor data (old data is useless)

**Transient Local**: Keeps recent messages for late subscribers.
- Use for: Configuration, static data (new nodes get current state)

```python
# PURPOSE: Show durability QoS for late-joining subscribers
# NOTE: This is conceptual code, not production-ready

from rclpy.qos import QoSProfile, DurabilityPolicy

# Transient local: new subscribers get last message immediately
transient_qos = QoSProfile(
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=1
)

# Publish static map - late subscribers still receive it
map_pub = node.create_publisher(OccupancyGrid, '/map', transient_qos)
```

**Explanation**: With transient local durability, if you publish a map and then a new navigation node starts 30 seconds later, that node immediately receives the most recent map—no need to wait for the next publish.

#### 3. History and Depth

**Keep Last N**: Store only the N most recent messages.
- Use for: Sensor data (old readings obsolete)

**Keep All**: Store all messages until delivered.
- Use for: Logging, reliable message delivery

**Depth**: How many messages to keep (for Keep Last).

```python
# PURPOSE: Show history depth configuration
# NOTE: This is conceptual code, not production-ready

# Keep only latest image (old images obsolete)
camera_qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1  # Only buffer 1 message
)

# Keep more messages for logging
log_qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=100  # Buffer up to 100 messages
)
```

#### 4. Deadline

**Deadline**: Notify if messages don't arrive within a time limit.
- Use for: Detecting stale sensor data, communication failures

```python
# PURPOSE: Show deadline QoS for detecting stale data
# NOTE: This is conceptual code, not production-ready

from rclpy.qos import QoSProfile
from rclpy.duration import Duration

# Expect messages at least every 100ms (10 Hz sensor)
deadline_qos = QoSProfile(
    deadline=Duration(seconds=0.1)
)

# Subscriber can handle deadline misses
def deadline_callback():
    node.get_logger().warn('Sensor data deadline missed!')
```

#### 5. Liveliness

**Liveliness**: Detect when a publisher has died or become unresponsive.
- Use for: Safety monitoring, failover systems

---

## Common QoS Profiles

ROS 2 provides predefined profiles for common scenarios:

### Sensor Data

```python
from rclpy.qos import qos_profile_sensor_data

# Best effort, volatile, keep last 5
# Good for: cameras, LiDAR, IMU
image_sub = node.create_subscription(
    Image, '/camera/image', callback, qos_profile_sensor_data
)
```

**Why**: Sensor data is high-frequency. Missing a frame is fine. We want low latency, not guaranteed delivery.

### System Default

```python
from rclpy.qos import qos_profile_system_default

# Reliable, volatile, keep last 10
# Good for: general-purpose communication
cmd_pub = node.create_publisher(
    Twist, '/cmd_vel', qos_profile_system_default
)
```

**Why**: Balanced defaults—reliable enough for most uses, not excessive resource usage.

### Services

```python
from rclpy.qos import qos_profile_services_default

# Reliable, volatile
# Good for: request-reply patterns
```

**Why**: Service calls need reliable delivery—the client is waiting for a response.

### Parameters

```python
from rclpy.qos import qos_profile_parameter_events

# Reliable, transient local
# Good for: configuration that late subscribers need
```

**Why**: New nodes should immediately know current parameter values.

---

## QoS Compatibility

### Matching Publishers and Subscribers

QoS policies must be **compatible** between publisher and subscriber:

| Publisher | Subscriber | Compatible? |
|-----------|------------|-------------|
| Reliable | Reliable | Yes |
| Best Effort | Best Effort | Yes |
| Reliable | Best Effort | Yes (subscriber accepts less) |
| Best Effort | Reliable | **No** (subscriber wants more) |

If incompatible, the connection fails—no data flows.

### Debugging QoS Issues

Common issues:
- **No data received**: Check QoS compatibility (use `ros2 topic info -v`)
- **High latency**: Reliable QoS adds acknowledgment overhead
- **Messages dropped**: Best effort QoS under network congestion

```bash
# Check topic QoS settings
ros2 topic info /camera/image --verbose

# Shows publisher and subscriber QoS, helping debug mismatches
```

---

## DDS Architecture in ROS 2

### DDS Communication Layers

**Architecture Overview**:
This diagram shows how ROS 2 uses DDS for communication, from application code down to network transport.

**Components (Top to Bottom)**:

**Application Layer**:
- **ROS 2 Node Code**: Your Python/C++ code using rclpy/rclcpp
- **ROS 2 Client Library**: rclpy, rclcpp (language bindings)
- **Abstraction**: You write `create_publisher()`, library handles the rest

**ROS Middleware Interface (RMW)**:
- **RMW API**: Abstract interface hiding DDS implementation details
- **RMW Implementation**: rmw_fastrtps, rmw_cyclonedds, etc.
- **Purpose**: Swap DDS vendors without changing application code

**DDS Layer**:
- **DDS API**: Standard interface defined by OMG
- **QoS Policies**: Reliability, durability, deadline, etc.
- **Discovery**: SPDP (Simple Participant Discovery Protocol)
- **Data Writers/Readers**: DDS equivalent of publishers/subscribers

**Transport Layer**:
- **RTPS Protocol**: Real-Time Publish-Subscribe wire protocol
- **UDP/TCP Sockets**: Actual network communication
- **Serialization**: CDR (Common Data Representation) for messages

**Data Flow (Publish Path)**:
1. Node calls `publisher.publish(msg)`
2. rclpy serializes message to CDR format
3. RMW passes to DDS Data Writer
4. DDS applies QoS policies (buffering, reliability)
5. RTPS protocol sends via UDP/TCP
6. Network delivers to subscribers

**Data Flow (Subscribe Path)**:
1. RTPS receives data from network
2. DDS Data Reader applies QoS (filtering, ordering)
3. RMW retrieves data from DDS
4. rclpy deserializes message
5. Callback function invoked with message

**Key Interfaces**:
- **RMW API**: C interface between ROS 2 and DDS implementations
- **DDS API**: Vendor-specific but OMG-standardized
- **RTPS**: Wire protocol enabling interoperability between DDS vendors

**Takeaway**: ROS 2's layered architecture abstracts DDS complexity—you write simple publish/subscribe code, and DDS handles discovery, reliability, and network transport transparently.

---

## Practical QoS Guidelines

### Choosing QoS for Your Topics

| Data Type | Reliability | Durability | Depth | Rationale |
|-----------|-------------|------------|-------|-----------|
| Camera images | Best Effort | Volatile | 1 | High bandwidth, old frames useless |
| LiDAR scans | Best Effort | Volatile | 1-5 | Same as camera |
| Velocity commands | Reliable | Volatile | 1 | Commands must arrive |
| Emergency stop | Reliable | Volatile | 1 | Safety-critical |
| Map data | Reliable | Transient Local | 1 | Late subscribers need it |
| TF transforms | Reliable | Volatile | 100 | Need history for lookups |
| Log messages | Best Effort | Volatile | 100 | Not critical, buffer for bursts |

### Performance Considerations

**Reliable + Large Depth = Memory**: Reliable QoS with deep history uses more memory for retransmission buffers.

**Best Effort + Fast Publisher = Drops**: If subscriber is slower than publisher, messages are lost.

**Transient Local = Storage**: DDS must store messages for late subscribers—use sparingly on high-rate topics.

---

## Summary

**Key Takeaways**:
- **DDS** is the industry-standard middleware powering ROS 2 communication
- **Quality of Service (QoS)** policies control how messages are delivered
- **Reliability**: Reliable (guaranteed) vs Best Effort (may lose)
- **Durability**: Volatile (no history) vs Transient Local (late subscribers get data)
- **History/Depth**: How many messages to buffer
- **Predefined profiles** (sensor_data, system_default) simplify common cases
- **QoS compatibility** is required—mismatched publisher/subscriber QoS prevents connection
- **Layered architecture** abstracts DDS complexity while providing its benefits

**Connection to Next Section**: You now understand the conceptual foundations of ROS 2—architecture, communication patterns, distributed systems, and middleware. In the **Advanced Topics** section, we'll preview what comes next: hands-on tutorials, hardware integration, and production deployment patterns covered in Iteration 3.

---

**[Continue to Section 5: Advanced Topics →](./05-advanced-topics.md)**
