# ROS 2 Architecture: Nodes, Topics, and Services

---

## Introduction

ROS 2 provides a structured way to build robot software as a collection of independent, communicating processes. Instead of writing one monolithic program that handles everything—sensing, planning, control—you decompose the system into **nodes** that communicate via well-defined **interfaces**.

**In This Section**:
- Understand nodes as the fundamental building blocks of ROS 2 applications
- Learn about the computational graph and how nodes discover each other
- Explore the three primary communication mechanisms: topics, services, and actions
- See how this architecture enables modular, maintainable robot software

---

## Nodes: Independent Processes

### What Is a Node?

A **node** is an independent process that performs a single, well-defined function within a robot system. Each node is responsible for one aspect of the robot's behavior:

- **Camera Driver Node**: Interfaces with hardware camera, publishes images
- **Object Detection Node**: Receives images, runs neural network inference, outputs detected objects
- **Path Planner Node**: Receives goals and obstacle information, computes collision-free paths
- **Motor Controller Node**: Receives velocity commands, sends signals to motor hardware
- **Localization Node**: Fuses sensor data to estimate robot position

### Node Properties

ROS 2 nodes have several important properties:

**Single Responsibility**: Each node does one job well. This makes code easier to understand, test, and maintain. If the camera driver crashes, the path planner keeps running.

**Process Isolation**: Nodes run in separate operating system processes. A bug in one node cannot corrupt memory in another node. This provides fault isolation—critical for safety in physical systems.

**Language Agnostic**: Nodes communicate via typed messages, not function calls. A Python node can talk to a C++ node seamlessly. You choose the right language for each task.

**Location Transparent**: Nodes don't care where other nodes run. The camera driver might run on an embedded ARM processor, while object detection runs on a powerful GPU server. ROS 2 handles the networking.

### Creating a Node

Here's the basic structure of a ROS 2 node in Python:

```python
# PURPOSE: Demonstrate the minimal structure of a ROS 2 node
# NOTE: This is conceptual code, not production-ready

import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    """A minimal ROS 2 node that does nothing but exist"""

    def __init__(self):
        # Initialize the node with a unique name
        super().__init__('minimal_node')

        # Log that we're alive
        self.get_logger().info('Minimal node has started!')

def main():
    # Initialize the ROS 2 Python client library
    rclpy.init()

    # Create an instance of our node
    node = MinimalNode()

    # Spin: keep the node running and processing callbacks
    rclpy.spin(node)

    # Cleanup when done
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Explanation**: This code shows the skeleton of every ROS 2 Python node. The `rclpy.init()` call sets up communication, the node constructor gives it a unique name, and `rclpy.spin()` keeps the node alive, processing any callbacks (from topics, services, timers). This structure is the foundation for all ROS 2 applications.

**Key Concepts Demonstrated**:
- **Node naming**: Each node has a unique name for identification in the system
- **Client library (rclpy)**: Language-specific library that handles ROS 2 mechanics
- **Spinning**: The event loop that processes incoming messages and timers

---

## The Computational Graph

### Nodes Form a Graph

When multiple nodes run and communicate, they form a **computational graph**. This graph has:

- **Nodes**: Vertices in the graph (processes doing work)
- **Topics/Services**: Edges connecting nodes (communication channels)

The graph is **dynamic**: nodes can join, leave, and discover each other at runtime. There's no central coordinator that must know about all nodes in advance.

### Peer-to-Peer Discovery

Unlike ROS 1 (which required a central `rosmaster`), ROS 2 uses **decentralized discovery**:

1. When a node starts, it announces itself on the network
2. Other nodes discover it automatically via multicast
3. Direct peer-to-peer connections are established for data transfer
4. If a node crashes, others detect this and can adapt

This decentralized approach is more robust: no single point of failure, and nodes can run on different machines naturally.

### ROS 2 Computational Graph Architecture

**Architecture Overview**:
This diagram shows how ROS 2 nodes form a distributed computational graph with peer-to-peer communication.

**Components**:
- **Node A (Camera Driver)**: Captures images from hardware, publishes to `/camera/image` topic at 30 Hz
- **Node B (Object Detector)**: Subscribes to `/camera/image`, runs YOLO inference, publishes to `/detections` topic
- **Node C (Path Planner)**: Subscribes to `/detections` and `/odom`, publishes to `/plan` topic
- **Node D (Motor Controller)**: Subscribes to `/plan`, publishes to `/cmd_vel` topic
- **Node E (Motor Driver)**: Subscribes to `/cmd_vel`, interfaces with hardware motors
- **DDS Discovery Service**: Enables nodes to find each other (multicast announcements)
- **DDS Data Transport**: Peer-to-peer data transfer between nodes (UDP/TCP)

**Relationships**:
- Node A → Node B: Publish-subscribe via `/camera/image` topic (streaming images)
- Node B → Node C: Publish-subscribe via `/detections` topic (detected objects)
- Node C → Node D: Publish-subscribe via `/plan` topic (path waypoints)
- Node D → Node E: Publish-subscribe via `/cmd_vel` topic (velocity commands)
- All nodes ↔ DDS Discovery: Multicast announcements for node discovery

**Data Flow**:
1. Camera Driver captures frame → publishes to `/camera/image`
2. Object Detector receives frame → processes → publishes detections
3. Path Planner receives detections → computes path → publishes plan
4. Motor Controller receives plan → generates velocity → publishes commands
5. Motor Driver receives commands → actuates motors → robot moves
6. Robot motion changes camera view → loop continues

**Key Interfaces**:
- **Topics**: Named channels carrying typed messages (e.g., `sensor_msgs/Image`, `geometry_msgs/Twist`)
- **DDS**: Data Distribution Service provides reliable, real-time data delivery
- **QoS Policies**: Configure reliability, latency, history for each topic

**Takeaway**: ROS 2's computational graph enables modular robot software where components discover each other automatically and communicate via typed message channels—no central coordinator required.

---

## Communication Mechanisms

ROS 2 provides three primary ways for nodes to communicate:

### Topics: Streaming Data (Publish-Subscribe)

**Topics** are named channels for streaming data:

- **Publisher**: Node that sends messages to a topic
- **Subscriber**: Node that receives messages from a topic
- **Many-to-many**: Multiple publishers and subscribers per topic
- **Asynchronous**: Publishers don't wait for subscribers; messages are delivered when ready

**Use cases**: Sensor data (camera images, LiDAR scans), continuous state updates (robot pose, joint states), command streams (velocity commands)

**Example**: A camera node publishes images to `/camera/image` at 30 Hz. Any node needing camera data simply subscribes—the camera doesn't need to know who's listening.

### Services: Request-Reply (Synchronous Queries)

**Services** provide synchronous request-reply communication:

- **Client**: Node that sends a request and waits for response
- **Server**: Node that receives requests, processes them, sends replies
- **One-to-one**: Each request gets exactly one response
- **Synchronous**: Client blocks until server responds

**Use cases**: Configuration queries ("What are the current parameters?"), one-time commands ("Save the map to disk"), state queries ("Is the arm at home position?")

**Example**: A mapping node offers a `/save_map` service. When called, it writes the current map to disk and returns success/failure.

### Actions: Long-Running Tasks (Goals with Feedback)

**Actions** handle long-running tasks with progress feedback:

- **Goal**: Client sends a goal to achieve
- **Feedback**: Server sends periodic progress updates
- **Result**: Server sends final result when complete
- **Cancelable**: Client can cancel a goal in progress

**Use cases**: Navigation ("Go to position X,Y"), manipulation ("Pick up the red block"), any task that takes time and benefits from progress updates

**Example**: A navigation action receives a goal position. While the robot moves, it sends feedback (current position, time remaining). When arrived, it sends success.

### Comparison: Topics vs Services vs Actions

| Aspect | Topics | Services | Actions |
|--------|--------|----------|---------|
| Pattern | Pub/Sub | Request/Reply | Goal/Feedback/Result |
| Timing | Asynchronous | Synchronous | Asynchronous with feedback |
| Cardinality | Many-to-many | One-to-one | One-to-one |
| Duration | Continuous | Instantaneous | Long-running |
| Example | Camera stream | Get parameter | Navigate to goal |

---

## Why This Architecture Matters

### Modularity

By decomposing robot software into nodes, you can:
- **Develop independently**: Different teams work on different nodes
- **Swap implementations**: Replace one object detector with another without touching navigation
- **Test in isolation**: Unit test a planner without needing real cameras

### Fault Tolerance

Node isolation provides robustness:
- **Crash isolation**: One node crashing doesn't bring down the system
- **Graceful degradation**: If perception fails, robot can stop safely
- **Hot restart**: Crashed nodes can restart without affecting others

### Scalability

The computational graph scales naturally:
- **Add capabilities**: New sensor? Add a node. New feature? Add a node.
- **Distribute load**: Spread nodes across multiple computers
- **Multi-robot**: Same architecture works for robot fleets

### Reusability

Standard interfaces enable code sharing:
- **Community packages**: Thousands of pre-built nodes available
- **Vendor-neutral**: Your code works with any ROS 2-compatible hardware
- **Research-to-production**: Same abstractions from prototyping to deployment

---

## Summary

**Key Takeaways**:
- **Nodes** are independent processes with single responsibilities—the building blocks of ROS 2 applications
- **Computational graph** connects nodes via topics, services, and actions—dynamic, decentralized, no single point of failure
- **Topics** provide asynchronous publish-subscribe for streaming data (cameras, sensors, commands)
- **Services** offer synchronous request-reply for queries and configuration
- **Actions** handle long-running tasks with feedback (navigation, manipulation)
- **Modularity** enables independent development, testing, and fault tolerance

**Connection to Next Section**: Now that you understand the basic building blocks, we'll dive deeper into **communication patterns**—exploring publish-subscribe and request-reply in detail with code examples.

---

**[Continue to Section 2: Communication Patterns →](./02-communication-patterns.md)**
