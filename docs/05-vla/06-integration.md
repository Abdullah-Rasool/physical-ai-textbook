# Integration Patterns: VLA to Robot Control

**Estimated Reading Time**: 8 minutes

---

## Learning Objectives

After completing this section, you will be able to:

1. **Describe** how VLA model outputs connect to ROS 2 control systems
2. **Explain** VLA input interfaces (camera topics, robot state, language)
3. **Understand** VLA output interfaces (velocities, trajectories, action goals)
4. **Identify** control loop patterns (open-loop, closed-loop, hierarchical)

---

## The Integration Challenge

VLA models exist as neural networks that process tensors and produce tensors. Physical robots exist as hardware that receives control signals and produces sensor readings. **Integration** bridges this gap.

The challenge:
- VLA operates on batched tensors at model inference rate
- ROS 2 operates on timestamped messages at various rates
- Hardware operates on electrical signals at high frequency

This section describes the patterns that connect these worlds.

---

## VLA Input Interfaces

### Camera Input

VLA models require visual input from robot cameras:

```
┌─────────────────────────────────────────────────────────────────────┐
│              Camera Input Pipeline                                  │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   ROS 2 Topic:              VLA Preprocessing:                      │
│   /camera/image_raw         ┌─────────────────────────────────┐    │
│         │                   │ • Resize to model input size    │    │
│         │                   │   (224×224, 336×336, etc.)      │    │
│         ▼                   │ • Normalize pixel values        │    │
│   sensor_msgs/Image         │   (ImageNet mean/std or custom) │    │
│   ┌────────────────┐        │ • Convert BGR→RGB if needed     │    │
│   │ width: 1280    │───────▶│ • Stack frames if temporal      │    │
│   │ height: 720    │        │ • Move to GPU tensor            │    │
│   │ encoding: bgr8 │        └─────────────────────────────────┘    │
│   │ data: [bytes]  │                      │                        │
│   └────────────────┘                      ▼                        │
│                                    VLA Model Input                  │
│                                    [B, T, C, H, W] tensor           │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

**Key Considerations**:
- **Synchronization**: Match image timestamps with other inputs
- **Latency**: Minimize time from capture to inference
- **Frame rate**: Balance freshness against compute load

### Robot State Input

VLA models often need current robot state (proprioception):

| ROS 2 Topic | Message Type | VLA Use |
|-------------|--------------|---------|
| `/joint_states` | `sensor_msgs/JointState` | Current joint positions/velocities |
| `/tf` | `tf2_msgs/TFMessage` | End-effector pose |
| `/gripper_state` | Custom or `sensor_msgs/JointState` | Gripper opening |

State preprocessing:
- Normalize joint values to [-1, 1] range
- Compute forward kinematics for end-effector pose
- Concatenate into state vector

### Language Input

Language commands enter through various interfaces:

```
┌─────────────────────────────────────────────────────────────────────┐
│              Language Input Options                                 │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   Option 1: Direct Text                                             │
│   /language_command (std_msgs/String)                               │
│   └─▶ "Pick up the red cup"                                        │
│                                                                     │
│   Option 2: Speech Recognition                                      │
│   /audio_raw (audio_msgs/Audio) ──▶ ASR Node ──▶ /language_command │
│                                                                     │
│   Option 3: UI Interface                                            │
│   Web/GUI ──▶ ROS 2 Bridge ──▶ /language_command                   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## VLA Output Interfaces

VLA outputs must be converted to ROS 2 messages for execution.

### Velocity Commands

For continuous control at each timestep:

```python
# Purpose: Convert VLA velocity output to ROS 2 message
# Note: Illustrative pseudocode

def publish_velocity(vla_output, publisher):
    """
    Convert VLA velocity prediction to Twist message.

    vla_output: [vx, vy, vz, wx, wy, wz] in normalized space
    """
    # Denormalize from [-1, 1] to robot velocity limits
    twist = Twist()
    twist.linear.x = vla_output[0] * MAX_LINEAR_VEL
    twist.linear.y = vla_output[1] * MAX_LINEAR_VEL
    twist.linear.z = vla_output[2] * MAX_LINEAR_VEL
    twist.angular.x = vla_output[3] * MAX_ANGULAR_VEL
    twist.angular.y = vla_output[4] * MAX_ANGULAR_VEL
    twist.angular.z = vla_output[5] * MAX_ANGULAR_VEL

    publisher.publish(twist)
```

### Joint Trajectory Commands

For sending complete motion plans:

```
VLA Action Chunk: [16 waypoints × 7 joints]
                          │
                          ▼
             ┌────────────────────────┐
             │ JointTrajectory        │
             │ • header               │
             │ • joint_names: [...]   │
             │ • points:              │
             │   - positions: [7]     │
             │   - time_from_start: t │
             │   (repeated 16×)       │
             └────────────────────────┘
                          │
                          ▼
         /arm_controller/follow_joint_trajectory (action)
```

### Action Goals

For task-level interfaces:

```
VLA High-Level Output: "navigate_to_kitchen"
                          │
                          ▼
              ┌───────────────────────┐
              │ NavigateToPose Action │
              │ • goal:               │
              │   - pose: kitchen_loc │
              │   - behavior_tree     │
              └───────────────────────┘
                          │
                          ▼
              /navigate_to_pose (action server)
```

---

## Control Loop Patterns

### Open-Loop Execution

VLA predicts trajectory; robot executes without VLA feedback:

```
┌─────────────────────────────────────────────────────────────────────┐
│              Open-Loop Pattern                                      │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   t=0: VLA inference ──▶ Trajectory [t=0 to t=1.6s]                │
│                                  │                                  │
│                                  ▼                                  │
│        t=0.0s: ════▶ Waypoint 1 ────────────────────────▶          │
│        t=0.1s: ════▶ Waypoint 2 ────────────────────────▶          │
│        ...                                                          │
│        t=1.6s: ════▶ Waypoint 16 ───────────────────────▶          │
│                                                                     │
│   t=1.6s: VLA inference ──▶ Next trajectory                        │
│                                                                     │
│   VLA runs only at trajectory boundaries                            │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

**Advantages**: Simple, low inference cost
**Disadvantages**: Can't react to unexpected changes

### Closed-Loop Execution

VLA continuously updates based on current perception:

```
┌─────────────────────────────────────────────────────────────────────┐
│              Closed-Loop Pattern                                    │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   t=0.0s: Observe ──▶ VLA ──▶ Action ──▶ Execute                   │
│                                   │                                 │
│                                   ▼                                 │
│   t=0.1s: Observe ──▶ VLA ──▶ Action ──▶ Execute                   │
│                                   │                                 │
│                                   ▼                                 │
│   t=0.2s: Observe ──▶ VLA ──▶ Action ──▶ Execute                   │
│                                   │                                 │
│   ...                             │                                 │
│                                                                     │
│   VLA runs every control cycle (10 Hz typical)                      │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

**Advantages**: Reactive, handles disturbances
**Disadvantages**: High inference load, potential instability

### Receding Horizon (Recommended)

Predict chunk, execute some, re-predict with overlap:

```
┌─────────────────────────────────────────────────────────────────────┐
│              Receding Horizon Pattern                               │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   t=0.0s: VLA predicts 16 actions                                   │
│           Execute actions 1-4                                       │
│                                                                     │
│   t=0.4s: VLA predicts 16 actions (new observation)                 │
│           Execute actions 1-4 of NEW prediction                     │
│                                                                     │
│   t=0.8s: VLA predicts 16 actions                                   │
│           Execute actions 1-4                                       │
│                                                                     │
│   ┌──────────────────────────────────────────────────────────┐     │
│   │ Prediction 1: [####============]                         │     │
│   │ Prediction 2:     [####============]                     │     │
│   │ Prediction 3:         [####============]                 │     │
│   │               ────▶ time                                 │     │
│   │               [####] = executed, [====] = discarded      │     │
│   └──────────────────────────────────────────────────────────┘     │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

**Advantages**: Smooth (planned trajectory), reactive (frequent re-planning)
**Disadvantages**: More complex implementation

---

## Integration Architecture

A complete VLA-ROS 2 integration node:

```
┌─────────────────────────────────────────────────────────────────────┐
│              VLA ROS 2 Node Architecture                            │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   ┌─────────────────────────────────────────────────────────────┐  │
│   │                     VLA Node                                 │  │
│   │                                                              │  │
│   │  Subscribers:                                                │  │
│   │  ├─ /camera/image_raw (Image)                               │  │
│   │  ├─ /joint_states (JointState)                              │  │
│   │  └─ /language_command (String)                              │  │
│   │                                                              │  │
│   │  Internal:                                                   │  │
│   │  ├─ Image buffer (latest N frames)                          │  │
│   │  ├─ State buffer (latest state)                             │  │
│   │  ├─ VLA Model (on GPU)                                      │  │
│   │  └─ Inference timer (10 Hz)                                 │  │
│   │                                                              │  │
│   │  Publishers/Clients:                                         │  │
│   │  ├─ /arm_controller/command (JointTrajectory)               │  │
│   │  └─ /gripper_controller/gripper_cmd (GripperCommand)        │  │
│   │                                                              │  │
│   └─────────────────────────────────────────────────────────────┘  │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Safety Considerations

VLA outputs should pass through safety layers before execution:

### Safety Checks

1. **Joint limit checking**: Reject actions outside joint range
2. **Velocity limiting**: Cap maximum velocities
3. **Collision checking**: Verify trajectory doesn't collide
4. **Force limiting**: Monitor and limit contact forces
5. **Watchdog**: Stop if VLA node fails

### Implementation Pattern

```
VLA Output ──▶ Safety Monitor ──▶ Controller
                    │
                    ├─ If safe: pass through
                    └─ If unsafe: stop/modify
```

---

## Connection to Previous Modules

- **Module 1 (Foundations)**: Integration completes the perception-action loop by connecting VLA intelligence to physical execution
- **Module 2 (ROS 2)**: All interfaces use ROS 2 messages, topics, services, and actions
- **Module 3 (Digital Twin)**: Integration can be tested identically in simulation via ROS 2 abstraction
- **Module 4 (Isaac)**: Isaac perception provides the visual input that VLA integrates

---

## Key Takeaways

- VLA **inputs** include camera images, robot state, and language commands—all via ROS 2 topics
- VLA **outputs** must be converted to ROS 2 messages (Twist, JointTrajectory, Action Goals)
- **Control loop patterns** range from open-loop to closed-loop to receding horizon
- The **receding horizon** pattern balances smoothness with reactivity
- **Safety layers** between VLA and execution are essential for physical deployment

---

**Next**: Return to [Module 5 Index](./index.md) or continue to [Module 6: Capstone](../06-capstone/index.md)
