# Module Connections: How the Pieces Fit Together

**Estimated Reading Time**: 8 minutes

---

## Learning Objectives

After completing this section, you will be able to:

1. **Trace** the specific interfaces between system modules
2. **Explain** how ROS 2 messages flow between Isaac perception, VLA intelligence, and control
3. **Identify** the message types and topics that connect components
4. **Understand** how each module's concepts enable the next

---

## From Concepts to Connections

The previous section showed the layered architecture. Now we examine the *interfaces* between layers—the specific ways that modules connect and communicate.

Understanding interfaces reveals:
- What information flows where
- What each component expects from others
- Where system integration happens
- How to extend or modify the system

---

## The Module Connection Map

```
┌─────────────────────────────────────────────────────────────────────┐
│              Module Interface Map                                   │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│                    Human Command                                    │
│                         │                                           │
│                         ▼                                           │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                    Module 5: VLA                             │   │
│  │  Receives: Images (sensor_msgs/Image)                        │   │
│  │            Language (std_msgs/String)                        │   │
│  │            Robot state (sensor_msgs/JointState)              │   │
│  │  Produces: Actions (trajectory_msgs/JointTrajectory)         │   │
│  │            or goal poses (geometry_msgs/PoseStamped)         │   │
│  └──────────────────────────┬──────────────────────────────────┘   │
│                              │                                      │
│         ┌────────────────────┼────────────────────┐                │
│         │                    │                    │                │
│         ▼                    ▼                    ▼                │
│  ┌─────────────┐    ┌─────────────────┐    ┌───────────────┐       │
│  │ Module 4:   │    │   Module 2:     │    │   Module 1:   │       │
│  │   Isaac     │◀──▶│     ROS 2       │◀──▶│   Control     │       │
│  │ Perception  │    │  Communication  │    │    Layer      │       │
│  └──────┬──────┘    └────────┬────────┘    └───────┬───────┘       │
│         │                    │                     │                │
│         │                    ▼                     │                │
│         │           ┌─────────────────┐            │                │
│         │           │   Module 3:     │            │                │
│         └──────────▶│  Digital Twin   │◀───────────┘                │
│                     │   Simulation    │                             │
│                     └─────────────────┘                             │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

**Diagram Explanation**: This map shows how modules interface through ROS 2. The VLA module sits at the top, consuming perception and producing actions. ROS 2 mediates all communication. Simulation connects to perception and control for development.

---

## Interface 1: Perception to Intelligence (Isaac → VLA)

Isaac perception provides the visual understanding that VLA needs.

### What Flows

| From Isaac | Message Type | To VLA |
|------------|--------------|--------|
| Camera images | `sensor_msgs/Image` | Visual input for VLA encoder |
| Object detections | `vision_msgs/Detection3DArray` | Grounding reference |
| Depth maps | `sensor_msgs/Image` | Spatial reasoning |
| Segmentation masks | `sensor_msgs/Image` | Object boundaries |

### How It Works

1. Camera driver publishes raw images to `/camera/image_raw`
2. Isaac ROS perception nodes process images:
   - Object detection node publishes to `/detections`
   - Depth estimation publishes to `/depth`
3. VLA node subscribes to these topics
4. VLA fuses perception with language for action generation

### Key Consideration

Isaac runs GPU-accelerated inference. VLA also needs GPU resources. Resource management between these components is crucial for real-time operation.

---

## Interface 2: Intelligence to Control (VLA → Motion)

VLA produces actions that the control system executes.

### What Flows

| From VLA | Message Type | To Control |
|----------|--------------|------------|
| Joint trajectories | `trajectory_msgs/JointTrajectory` | Direct execution |
| Cartesian goals | `geometry_msgs/PoseStamped` | Motion planning target |
| Gripper commands | `control_msgs/GripperCommand` | End-effector control |
| Velocity commands | `geometry_msgs/Twist` | Base movement |

### How It Works

```
┌─────────────────────────────────────────────────────────────────────┐
│              VLA to Control Interface                               │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   VLA Model Output                                                  │
│        │                                                            │
│        ▼                                                            │
│   ┌──────────────────┐                                              │
│   │ Action Interface │                                              │
│   │    Node          │                                              │
│   └────────┬─────────┘                                              │
│            │                                                        │
│      ┌─────┴─────────────────────────┐                              │
│      │                               │                              │
│      ▼                               ▼                              │
│  ┌────────────────┐          ┌────────────────┐                     │
│  │ Motion Planner │          │Direct Trajectory│                    │
│  │ (MoveIt/etc)   │          │   Execution     │                    │
│  └───────┬────────┘          └───────┬────────┘                     │
│          │                           │                              │
│          └───────────┬───────────────┘                              │
│                      │                                              │
│                      ▼                                              │
│              Joint Controllers                                      │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### Two Modes

1. **VLA produces goals**: Motion planner generates collision-free path
2. **VLA produces trajectories**: Direct execution (VLA handles planning)

The choice depends on VLA architecture and safety requirements.

---

## Interface 3: Control to Hardware (Controllers → Motors)

The control layer interfaces with physical actuators.

### What Flows

| From Controllers | Interface | To Hardware |
|------------------|-----------|-------------|
| Joint commands | ros2_control | Motor drivers |
| Joint states | ros2_control | Encoder feedback |
| Force/torque | sensor topics | Force sensors |

### ROS 2 Control Framework

```
┌─────────────────────────────────────────────────────────────────────┐
│              ros2_control Architecture                              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   ┌─────────────────────────────────────────────────────────────┐  │
│   │                 Controller Manager                           │  │
│   └─────────────────────────┬───────────────────────────────────┘  │
│                              │                                      │
│         ┌────────────────────┼────────────────────┐                │
│         │                    │                    │                │
│         ▼                    ▼                    ▼                │
│   ┌───────────┐      ┌───────────┐      ┌───────────┐             │
│   │ Joint     │      │ Joint     │      │  Gripper  │             │
│   │Trajectory │      │ Position  │      │Controller │             │
│   │Controller │      │Controller │      │           │             │
│   └─────┬─────┘      └─────┬─────┘      └─────┬─────┘             │
│         │                  │                  │                    │
│         └──────────────────┼──────────────────┘                    │
│                            │                                       │
│                            ▼                                       │
│   ┌─────────────────────────────────────────────────────────────┐  │
│   │                 Hardware Interface                           │  │
│   │         (talks to actual motor drivers/encoders)             │  │
│   └─────────────────────────────────────────────────────────────┘  │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Interface 4: Simulation to All (Digital Twin ↔ Stack)

Simulation interfaces with multiple layers for development.

### Simulation Mode

In simulation:
- Physics engine replaces hardware
- Simulated sensors produce same message types as real sensors
- Simulated actuators receive same commands as real actuators

```
┌─────────────────────────────────────────────────────────────────────┐
│              Simulation Interface                                   │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   Real System                         Simulated System              │
│                                                                     │
│   Camera ────▶ /camera/image          Sim Camera ───▶ /camera/image│
│                                                                     │
│   Robot  ◀──── /joint_commands        Sim Robot ◀──── /joint_commands
│                                                                     │
│   Encoder ───▶ /joint_states          Sim Physics ──▶ /joint_states│
│                                                                     │
│   ════════════════════════════════════════════════════════════════ │
│                                                                     │
│   Same topics → Same software stack works in both environments      │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

This is the key insight of the digital twin approach: **identical interfaces enable seamless transition between simulation and reality**.

---

## ROS 2 as the Integration Layer

ROS 2 (Module 2) serves as the universal connector:

### Communication Patterns Used

| Pattern | Use Case | Example |
|---------|----------|---------|
| Topics | Continuous data streams | Camera images, joint states |
| Services | Request-response queries | Get object pose |
| Actions | Long-running tasks | Execute trajectory |
| Parameters | Configuration | Control gains, thresholds |

### Message Flow Example

Complete flow for "pick up the red cup":

```
1. /camera/image_raw        [topic]    Camera → Isaac perception
2. /detections              [topic]    Isaac → VLA (cup detected)
3. /language_command        [topic]    UI → VLA ("pick up red cup")
4. /arm_controller/follow_  [action]   VLA → Controller (trajectory)
   joint_trajectory
5. /joint_states            [topic]    Controller → VLA (feedback)
6. /gripper_controller/     [action]   VLA → Gripper (close)
   gripper_cmd
```

---

## Conceptual Dependencies

Beyond data interfaces, modules depend on concepts from prior modules:

```
┌─────────────────────────────────────────────────────────────────────┐
│              Conceptual Dependencies                                │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   Module 5 (VLA) depends on:                                        │
│   ├── Module 4 concepts: perception pipelines, AI inference         │
│   ├── Module 2 concepts: message passing, action interfaces         │
│   └── Module 1 concepts: perception-action loop, embodiment         │
│                                                                     │
│   Module 4 (Isaac) depends on:                                      │
│   ├── Module 3 concepts: simulation, synthetic data                 │
│   ├── Module 2 concepts: ROS 2 nodes, topics                        │
│   └── Module 1 concepts: sensing, physical world                    │
│                                                                     │
│   Module 3 (Digital Twin) depends on:                               │
│   ├── Module 2 concepts: ROS 2 interfaces                           │
│   └── Module 1 concepts: physics, sensors, actuators                │
│                                                                     │
│   Module 2 (ROS 2) depends on:                                      │
│   └── Module 1 concepts: distributed systems, real-time             │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

Each module builds on understanding from earlier modules.

---

## Connection to Previous Modules

- **Module 1 (Foundations)**: Provides the conceptual framework that all interfaces serve (perception-action loop)
- **Module 2 (ROS 2)**: The actual mechanism for all interfaces—topics, services, actions
- **Module 3 (Digital Twin)**: Enables testing interfaces in simulation before hardware
- **Module 4 (Isaac)**: Produces perception outputs that VLA consumes
- **Module 5 (VLA)**: Consumes perception, produces actions—the intelligence hub

---

## Key Takeaways

- Modules connect through **well-defined interfaces** using ROS 2 message types
- **Perception → Intelligence**: Images, detections, depth flow to VLA for understanding
- **Intelligence → Control**: Trajectories and goals flow to motion execution
- **Control → Hardware**: ros2_control framework bridges software and motors
- **Simulation ↔ All**: Identical interfaces enable sim-to-real transfer
- **ROS 2 is the glue**: All communication flows through ROS 2 patterns

---

**Next**: [Reference Architecture →](./03-reference-architecture.md)
