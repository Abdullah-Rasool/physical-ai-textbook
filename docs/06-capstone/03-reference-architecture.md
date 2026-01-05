# Reference Architecture: Complete Data Flow

**Estimated Reading Time**: 10 minutes

---

## Learning Objectives

After completing this section, you will be able to:

1. **Trace** complete data flows through a reference humanoid AI architecture
2. **Identify** all components involved in executing a language-driven task
3. **Understand** timing relationships between components
4. **Describe** the architecture as a mental model for humanoid AI systems

---

## A Reference Humanoid AI Architecture

This section presents a **reference architecture**—a concrete, complete example of how a humanoid AI system is structured. Use this as a mental model when reasoning about humanoid AI systems.

### The Reference Robot

Our reference system is a humanoid robot with:
- **Head**: 2 RGB cameras, microphone
- **Torso**: IMU, compute platform
- **Arms**: Two 7-DOF arms with parallel jaw grippers
- **Base**: Mobile platform or legs (abstracted)

### The Complete Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│              Reference Humanoid AI Architecture                     │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                      USER INTERFACE                          │   │
│  │  • Speech recognition (audio → text)                         │   │
│  │  • Command display and feedback                              │   │
│  └─────────────────────────────┬───────────────────────────────┘   │
│                                │ language commands                  │
│                                ▼                                    │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                    VLA INFERENCE NODE                        │   │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐       │   │
│  │  │Vision Encoder│  │ Language     │  │ Action       │       │   │
│  │  │  (SigLIP)    │  │ Model (LLM)  │  │ Decoder      │       │   │
│  │  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘       │   │
│  │         └─────────────────┴─────────────────┘               │   │
│  │                    Fused reasoning → action tokens           │   │
│  └─────────────────────────────┬───────────────────────────────┘   │
│                                │ action trajectories                │
│                                ▼                                    │
│  ┌───────────────────────┬─────────────────┬───────────────────┐   │
│  │    LEFT ARM           │    RIGHT ARM    │     GRIPPER       │   │
│  │    CONTROLLER         │    CONTROLLER   │   CONTROLLERS     │   │
│  │  • Trajectory exec    │  • Trajectory   │  • Open/close     │   │
│  │  • Collision check    │    exec         │  • Force control  │   │
│  └───────────┬───────────┴────────┬────────┴─────────┬─────────┘   │
│              │                    │                  │              │
│              ▼                    ▼                  ▼              │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                    HARDWARE INTERFACE                        │   │
│  │  • Motor drivers (7 per arm + grippers)                      │   │
│  │  • Joint encoders (position/velocity feedback)               │   │
│  │  • Force/torque sensors (wrist)                              │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
│  ═══════════════════════════════════════════════════════════════   │
│                         PERCEPTION PIPELINE                         │
│  ═══════════════════════════════════════════════════════════════   │
│                                                                     │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                    SENSOR DRIVERS                            │   │
│  │  ┌────────────┐  ┌────────────┐  ┌────────────┐             │   │
│  │  │ Left Cam   │  │ Right Cam  │  │    IMU     │             │   │
│  │  │ 1280x720   │  │ 1280x720   │  │ 200 Hz     │             │   │
│  │  │ 30 fps     │  │ 30 fps     │  │            │             │   │
│  │  └─────┬──────┘  └─────┬──────┘  └─────┬──────┘             │   │
│  └────────┼───────────────┼───────────────┼────────────────────┘   │
│           │               │               │                        │
│           ▼               ▼               ▼                        │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │              ISAAC ROS PERCEPTION NODES                      │   │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐       │   │
│  │  │ Stereo Depth │  │ Object       │  │ Pose         │       │   │
│  │  │ Estimation   │  │ Detection    │  │ Estimation   │       │   │
│  │  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘       │   │
│  │         │                 │                 │                │   │
│  │         └─────────────────┴─────────────────┘                │   │
│  │                      Scene Understanding                     │   │
│  └─────────────────────────────┬───────────────────────────────┘   │
│                                │                                    │
│                                └───▶ To VLA Inference Node          │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Data Flow Walkthrough: "Pick Up the Red Cup"

Let's trace exactly what happens when a human says "Pick up the red cup."

### Phase 1: Command Reception (t = 0 ms)

```
Human speaks: "Pick up the red cup"
      │
      ▼
┌──────────────────┐
│Speech Recognition│  Audio → Text conversion
│   Node           │
└────────┬─────────┘
         │
         ▼
/language_command topic: "Pick up the red cup"
```

**ROS 2 message**: `std_msgs/String` on `/language_command`

### Phase 2: Perception Processing (t = 0-50 ms)

Concurrent with command reception, perception continuously processes:

```
Camera images (30 fps, every 33 ms)
      │
      ▼
┌──────────────────┐
│ Isaac Object     │  GPU-accelerated inference
│ Detection        │  ~15 ms latency
└────────┬─────────┘
         │
         ▼
/detections topic:
  - object: "cup", color: "red", position: (0.45, 0.12, 0.08)
  - object: "cup", color: "blue", position: (0.30, 0.25, 0.08)
  - object: "table", position: (0.40, 0.20, 0.00)
```

**ROS 2 message**: `vision_msgs/Detection3DArray` on `/detections`

### Phase 3: VLA Inference (t = 50-150 ms)

The VLA model receives perception and language, produces actions:

```
Inputs:
  - Current image frame
  - Language: "Pick up the red cup"
  - Robot state (joint positions)
  - Detection: red cup at (0.45, 0.12, 0.08)
      │
      ▼
┌──────────────────────────────────────────┐
│            VLA Model Inference            │
│                                           │
│  1. Vision encoder processes image        │
│  2. Language tokens: ["pick", "up",       │
│     "the", "red", "cup"]                  │
│  3. Cross-attention: language attends     │
│     to red cup region in visual features  │
│  4. Action decoder generates trajectory   │
│     chunk (16 actions, ~100ms inference)  │
└──────────────────────────────────────────┘
      │
      ▼
Output: JointTrajectory with 16 waypoints
  - Approach phase: move arm toward cup
  - Pre-grasp: position gripper above cup
```

**ROS 2 message**: `trajectory_msgs/JointTrajectory` on `/arm_controller/command`

### Phase 4: Motion Execution (t = 150-2000 ms)

Control system executes the trajectory:

```
Trajectory received
      │
      ▼
┌──────────────────────────────────────────┐
│        Joint Trajectory Controller        │
│                                           │
│  For each waypoint (every 10 ms):         │
│  1. Interpolate target joint positions    │
│  2. Compute control commands              │
│  3. Send to hardware interface            │
│  4. Read encoder feedback                 │
│  5. Adjust for tracking error             │
└──────────────────────────────────────────┘
      │
      ▼
Motor commands → Physical arm movement
```

### Phase 5: Grasp Execution (t = 2000-2500 ms)

VLA continues predicting actions for grasp:

```
New VLA inference (with updated visual input showing arm near cup)
      │
      ▼
Action chunk:
  - Final approach (fine positioning)
  - Gripper close command
      │
      ▼
┌──────────────────┐
│Gripper Controller│
└────────┬─────────┘
         │
         ▼
Gripper closes on cup
Force sensors detect contact
```

### Phase 6: Lift and Complete (t = 2500-3500 ms)

```
VLA observes successful grasp
      │
      ▼
Action chunk:
  - Lift trajectory (move arm up)
  - Hold position
      │
      ▼
Robot lifts cup
Task complete
```

---

## Timing Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│              Timing: "Pick up the red cup"                          │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  Time (ms):  0    100   200   500   1000  1500  2000  2500  3000   │
│              │     │     │     │     │     │     │     │     │      │
│  Speech:     █─────┘                                                │
│                                                                     │
│  Perception: ████████████████████████████████████████████████████   │
│              (continuous 30 Hz)                                     │
│                                                                     │
│  VLA:              █████       █████       █████       █████        │
│              (inference at ~10 Hz, 100ms per chunk)                 │
│                                                                     │
│  Control:                ████████████████████████████████████████   │
│              (continuous 100 Hz trajectory tracking)                │
│                                                                     │
│  Motion:                      ████████████████████████████████      │
│              (approach → pre-grasp → grasp → lift)                  │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Key Architecture Patterns

### Pattern 1: Asynchronous Pipelines

Components run independently at their own rates:
- Perception doesn't wait for VLA
- VLA doesn't wait for control
- Control runs continuously regardless of VLA updates

### Pattern 2: Latest-Message Semantics

When VLA is ready for input:
- Use *latest* perception frame
- Use *latest* robot state
- Don't queue old data

### Pattern 3: Action Chunking + Receding Horizon

VLA predicts chunk of actions, executes some, re-predicts:
- Smooth motion (planned trajectory)
- Reactive (re-planning on new observations)

### Pattern 4: Hierarchical Control

```
VLA (10 Hz) → Trajectory Controller (100 Hz) → Joint PID (1000 Hz)
```

High-level decisions at low frequency, low-level control at high frequency.

---

## Using This Reference

This reference architecture can be used to:

1. **Understand real systems**: Production robots follow similar patterns
2. **Design new systems**: Start from this template, modify as needed
3. **Debug problems**: Identify which component/interface is failing
4. **Evaluate tradeoffs**: Understand where changes have impact

---

## Connection to Previous Modules

- **Module 1 (Foundations)**: This architecture implements the perception-action loop with all supporting infrastructure
- **Module 2 (ROS 2)**: Every connection in this diagram is a ROS 2 topic, service, or action
- **Module 3 (Digital Twin)**: This entire architecture can run in simulation for development
- **Module 4 (Isaac)**: The perception pipeline uses Isaac ROS for GPU-accelerated processing
- **Module 5 (VLA)**: The VLA inference node is the intelligence center of this architecture

---

## Key Takeaways

- The **reference architecture** provides a concrete mental model for humanoid AI systems
- A simple task like "pick up the red cup" involves **coordinated operation** of speech, perception, intelligence, control, and hardware
- **Timing matters**: different components run at different frequencies, requiring careful coordination
- Key patterns include **asynchronous pipelines**, **latest-message semantics**, and **hierarchical control**
- This architecture can **run identically** in simulation and on real hardware due to ROS 2 abstraction

---

**Next**: [System Concerns →](./04-system-concerns.md)
