# Diagram Description Template

## Usage

Use this template when describing system architectures or concept diagrams in text format (no image assets required in Iteration 2).

**Requirements**:
- FR-004: Module must have at least 2 diagram descriptions
- Descriptions must enable reader visualization without images
- Must describe components, relationships, and data flow clearly

---

## Template

```markdown
### [Diagram Title]

**Architecture Overview**:
[1-2 sentences describing what this diagram shows and why it's important]

**Components**:
- **[Component 1 Name]**: [Description, role, responsibility]
- **[Component 2 Name]**: [Description, role, responsibility]
- **[Component 3 Name]**: [Description, role, responsibility]
- **[Component N Name]**: [Description, role, responsibility]

**Relationships**:
- [Component A] ↔ [Component B]: [How they interact]
- [Component B] → [Component C]: [How they interact]
- [Component C] ↔ [Component D]: [How they interact]

**Data Flow** (if applicable):
1. [Step 1]: [What happens first]
2. [Step 2]: [What happens next]
3. [Step 3]: [What happens after that]
4. [Step N]: [Final step or loop back]

**Key Interfaces** (if applicable):
- [Interface 1]: [Protocol, message type, purpose]
- [Interface 2]: [Protocol, message type, purpose]

**Takeaway**: [1 sentence summarizing the main insight from this diagram]
```

---

## Example 1: Physical AI vs Traditional AI Architecture

```markdown
### Physical AI vs Traditional AI: Architectural Comparison

**Architecture Overview**:
This diagram contrasts the fundamental architectural differences between traditional AI systems (which operate on digital data) and Physical AI systems (which interact with the physical world through sensors and actuators).

**Components**:

**Traditional AI (Left Side)**:
- **Digital Input Sources**: Text, images, structured datasets, web data
- **AI Model**: Neural networks, transformers, statistical models running on cloud/GPU
- **Digital Outputs**: Predictions, classifications, generated text/images
- **User Interface**: Web browser, API, mobile app (digital-only interaction)

**Physical AI (Right Side)**:
- **Sensors**: Cameras, LiDAR, IMU, tactile sensors, joint encoders
- **Perception Module**: Computer vision, sensor fusion, SLAM running on robot
- **World Model**: Internal representation of environment and robot state
- **Action Planner**: Task planning, motion planning, control algorithms
- **Actuators**: Motors, grippers, wheels, joints (physical interaction)
- **Physical Environment**: Real-world objects, obstacles, humans

**Relationships**:
- Traditional AI: **One-way flow** from digital input → model → digital output
- Physical AI: **Closed loop** where actions affect environment, which affects sensors
- Traditional AI: **No feedback** from output to input (batch processing)
- Physical AI: **Continuous feedback** via sensorimotor coupling

**Data Flow (Physical AI)**:
1. **Environment → Sensors**: Physical world generates sensor data (light, sound, force)
2. **Sensors → Perception**: Raw data processed into structured information (objects, obstacles)
3. **Perception → World Model**: Environment representation updated continuously
4. **World Model → Action Planner**: Plans next action based on current state and goals
5. **Action Planner → Actuators**: Motor commands executed
6. **Actuators → Environment**: Robot physically changes the world (moves object, navigates)
7. **Environment → Sensors** (Loop): Changed environment affects next sensor readings

**Key Interfaces**:
- Traditional AI: **HTTP APIs**, file I/O, database connections (digital protocols)
- Physical AI: **Sensor protocols** (I2C, SPI, Ethernet), **motor drivers** (CAN bus, PWM), **ROS 2 topics** (DDS middleware)

**Takeaway**: Physical AI closes the perception-action loop with the real world, creating continuous sensorimotor coupling, whereas traditional AI operates in a digital-only, open-loop manner.
```

---

## Example 2: ROS 2 Node Communication Architecture

```markdown
### ROS 2 Distributed Node Communication

**Architecture Overview**:
This diagram shows how multiple ROS 2 nodes communicate using topics and services to create a distributed robot control system.

**Components**:
- **Camera Node**: Captures images from camera sensor, publishes to `/camera/image` topic
- **Perception Node**: Subscribes to `/camera/image`, processes images, publishes detected objects to `/objects` topic
- **Planner Node**: Subscribes to `/objects`, plans paths, publishes goals to `/goal` topic
- **Controller Node**: Subscribes to `/goal`, computes motor commands, publishes to `/cmd_vel` topic
- **Driver Node**: Subscribes to `/cmd_vel`, sends commands to motor hardware
- **TF Node**: Publishes coordinate transforms to `/tf` topic (robot pose, sensor frames)
- **DDS Middleware (invisible layer)**: Handles message delivery, discovery, QoS

**Relationships**:
- Camera → Perception: **Pub/sub** via `/camera/image` topic (asynchronous, streaming)
- Perception → Planner: **Pub/sub** via `/objects` topic (asynchronous, state updates)
- Planner → Controller: **Pub/sub** via `/goal` topic (asynchronous, commands)
- Controller → Driver: **Pub/sub** via `/cmd_vel` topic (high-frequency commands)
- All nodes ↔ TF Node: **Broadcast** of coordinate transforms (any node can listen)
- Any node ↔ Parameter Server: **Service call** for configuration (synchronous, request/reply)

**Data Flow (Example: Object Detection → Navigation)**:
1. **Camera Node** captures image at 30 Hz → publishes to `/camera/image`
2. **Perception Node** receives image → runs object detection model → publishes bounding boxes to `/objects`
3. **Planner Node** receives object list → computes obstacle-free path → publishes waypoints to `/goal`
4. **Controller Node** receives goal waypoint → computes velocity commands (PID control) → publishes to `/cmd_vel`
5. **Driver Node** receives velocity command → sends motor control signals → robot moves
6. **TF Node** continuously publishes robot pose → all nodes use for coordinate transforms
7. (Loop continues at camera frame rate)

**Key Interfaces**:
- **Topic**: Asynchronous pub/sub, many-to-many, best-effort or reliable QoS
- **Service**: Synchronous request/reply, one-to-one, used for configuration or queries
- **Action**: Long-running tasks with feedback, used for navigation or manipulation
- **DDS Protocol**: Underlying network transport (UDP multicast for discovery, TCP/UDP for data)

**Takeaway**: ROS 2 enables modular, distributed robot systems where nodes communicate via topics (streaming data) and services (request/reply), with DDS middleware handling network complexity transparently.
```

---

## Example 3: Isaac Sim → ROS 2 → Robot Pipeline

```markdown
### NVIDIA Isaac Sim to Physical Robot Pipeline

**Architecture Overview**:
This diagram illustrates the end-to-end workflow from simulation in Isaac Sim to deployment on a physical robot using ROS 2 as the integration layer.

**Components**:

**Simulation Environment (Isaac Sim)**:
- **Virtual Robot Model**: High-fidelity URDF/USD robot with accurate physics
- **Simulated Sensors**: GPU-accelerated cameras, LiDAR, IMU with realistic noise
- **Physics Engine**: NVIDIA PhysX for collision, contact, dynamics
- **Isaac ROS Bridge**: Publishes simulated sensor data to ROS 2 topics

**ROS 2 Middleware Layer**:
- **Isaac ROS Perception**: GPU-accelerated perception nodes (object detection, SLAM)
- **Navigation Stack**: Path planning, obstacle avoidance (Nav2)
- **Control Nodes**: Motion controllers, inverse kinematics
- **Message Topics**: Sensor data, commands, state information

**Physical Robot**:
- **Real Sensors**: Physical cameras, LiDAR, IMU
- **ROS 2 Hardware Drivers**: Interface hardware to ROS 2 topics (same interface as sim!)
- **Motor Controllers**: Execute commands on real actuators
- **Robot Computer**: Jetson Orin or similar edge device

**Relationships**:
- Isaac Sim ↔ ROS 2: **Bidirectional** (sensor data out, commands in) via ROS 2 Bridge
- ROS 2 ↔ Physical Robot: **Identical interface** as sim (swap sim bridge for hardware drivers)
- Isaac ROS Nodes: **Same code** runs in sim and real robot (GPU acceleration on both)

**Data Flow (Simulation)**:
1. Isaac Sim renders virtual camera image (GPU) → Isaac ROS Bridge → publishes to `/camera/image` (ROS 2 topic)
2. Isaac ROS Perception Node subscribes → runs DNN inference (GPU) → publishes detected objects
3. Nav2 Planner receives objects → computes path → publishes velocity commands to `/cmd_vel`
4. Isaac Sim receives `/cmd_vel` → applies to virtual robot → robot moves in simulation
5. Robot motion affects sensor view → loop continues

**Data Flow (Physical Robot)**:
1. Real camera captures image → Hardware Driver → publishes to `/camera/image` (same topic!)
2. **Identical perception and planning nodes as simulation**
3. Driver Node receives `/cmd_vel` → sends to motor controllers → robot moves
4. Robot motion affects sensor view → loop continues

**Key Interfaces**:
- **Isaac ROS Bridge**: Connects Isaac Sim to ROS 2 (UDP/TCP network)
- **ROS 2 Topics**: `/camera/image`, `/scan`, `/odom`, `/cmd_vel`, etc. (same in sim and real)
- **Isaac ROS GEMs**: GPU-accelerated perception, optimized for Jetson

**Takeaway**: Isaac Sim enables risk-free development and testing in simulation with the exact same ROS 2 interface as the physical robot, making sim-to-real transfer seamless by design.
```

---

## Guidelines

### Description Length
- **Minimum**: 100 words (enough to visualize basic architecture)
- **Typical**: 200-300 words (clear components + flow)
- **Maximum**: 500 words (complex multi-layer systems)

### Component Descriptions
- **Be specific**: Not "AI module" but "Isaac ROS Perception Node (GPU-accelerated object detection)"
- **Include role**: What each component does and why it exists
- **Avoid jargon**: Or define terms inline (e.g., "DDS (Data Distribution Service) middleware")

### Data Flow Clarity
- **Use numbered steps** for sequential flows
- **Indicate loops** explicitly ("loop continues", "feedback to step 1")
- **Show causality**: "Action A causes B, which triggers C"

### Visual Language
Use words that help readers visualize:
- "Left side", "right side", "top layer", "bottom layer"
- "Arrows flow from A to B"
- "Components arranged in pipeline"
- "Feedback loop connects output back to input"

### When to Use Diagrams
- **System architectures**: How components connect
- **Data flows**: Sequential processing pipelines
- **Comparisons**: Side-by-side architectural differences
- **Layered models**: Abstraction layers (perception → planning → control)

---

## Anti-Patterns (Avoid These)

❌ **Too vague**:
```markdown
**Components**:
- Input module
- Processing module
- Output module
```

❌ **No data flow**:
```markdown
[Lists components but doesn't explain how they interact]
```

❌ **Jargon overload**:
```markdown
The DDS QoS-aware pub/sub middleware leverages RTPS for best-effort
unreliable datagram delivery over UDP multicast...
[Reader can't visualize this]
```

✅ **Good Example** (clear, specific, visualizable):
```markdown
**Components**:
- **Camera Node**: Captures RGB images at 30 Hz, publishes to `/camera/image` topic
- **Perception Node**: Runs YOLO object detection, outputs bounding boxes to `/objects`

**Data Flow**:
1. Camera captures frame → publishes to topic
2. Perception receives frame → detects objects → publishes results
3. (Loop continues at 30 Hz)

**Takeaway**: Asynchronous pub/sub enables real-time streaming.
```

---

## Checklist

Before embedding a diagram description, verify:

- [ ] Title is clear and descriptive
- [ ] Overview sentence explains what diagram shows
- [ ] Components listed with specific roles (not vague "modules")
- [ ] Relationships describe how components interact
- [ ] Data flow shows sequential steps (if applicable)
- [ ] Interfaces specify protocols/message types (if applicable)
- [ ] Takeaway summarizes key insight
- [ ] Length appropriate (100-500 words)
- [ ] Reader can visualize architecture from text alone

---

**Use this template consistently to create clear, visualizable architecture descriptions across all modules.**
