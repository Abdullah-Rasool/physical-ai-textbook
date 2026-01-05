# System Concerns: Real-Time, Safety, and Modularity

**Estimated Reading Time**: 8 minutes

---

## Learning Objectives

After completing this section, you will be able to:

1. **Explain** real-time requirements in humanoid AI systems
2. **Describe** safety layers and their implementation
3. **Understand** modularity principles for maintainable robot systems
4. **Identify** failure handling patterns

---

## Beyond Functional Requirements

The previous sections focused on *what* humanoid AI systems do. This section examines *how* they must do it—the cross-cutting concerns that affect every component:

- **Real-time**: Meeting timing deadlines
- **Safety**: Preventing harm
- **Modularity**: Enabling maintenance and evolution
- **Reliability**: Handling failures gracefully

---

## Real-Time Requirements

Humanoid robots interact with the physical world at specific timescales. Missing deadlines has consequences.

### Timing Hierarchy

```
┌─────────────────────────────────────────────────────────────────────┐
│              Real-Time Timing Requirements                          │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   Layer              Frequency     Deadline      Consequence        │
│   ─────────────────────────────────────────────────────────────    │
│   Joint control      1000 Hz       1 ms         Instability         │
│   Trajectory track   100 Hz        10 ms        Jerky motion        │
│   Perception         30 Hz         33 ms        Stale visual data   │
│   VLA inference      10 Hz         100 ms       Delayed response    │
│   Task planning      1 Hz          1000 ms      Slow adaptation     │
│                                                                     │
│   ┌───────────────────────────────────────────────────────────┐    │
│   │                                                           │    │
│   │   1 ms  ◀─────────────────────────────────────▶  1000 ms │    │
│   │   │                                               │       │    │
│   │   Joint ─── Trajectory ─── Perception ─── VLA ─── Task   │    │
│   │   control   tracking       processing     inference plan │    │
│   │   (hard)    (firm)         (soft)         (soft)  (soft) │    │
│   │                                                           │    │
│   └───────────────────────────────────────────────────────────┘    │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### Hard vs. Soft Real-Time

- **Hard real-time**: Missing deadline causes failure (joint control)
- **Firm real-time**: Missing deadline degrades performance (trajectory tracking)
- **Soft real-time**: Missing deadline acceptable occasionally (VLA inference)

### Implementation Strategies

1. **Separate rate domains**: Run components at appropriate frequencies
2. **Predictable latency**: Use GPU inference with consistent timing
3. **Buffering**: Decouple producers and consumers
4. **Deadline monitoring**: Detect and handle missed deadlines

---

## Safety Layers

Safety in humanoid robots is non-negotiable. Multiple layers protect against harm.

### Defense in Depth

```
┌─────────────────────────────────────────────────────────────────────┐
│              Safety Layer Architecture                              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   ┌─────────────────────────────────────────────────────────────┐  │
│   │  Layer 5: Task Level                                         │  │
│   │  • Task validator: Is this command reasonable?               │  │
│   │  • Human approval gates for dangerous actions                │  │
│   └─────────────────────────────────────────────────────────────┘  │
│                              │                                      │
│                              ▼                                      │
│   ┌─────────────────────────────────────────────────────────────┐  │
│   │  Layer 4: Motion Level                                       │  │
│   │  • Collision checking (self and environment)                 │  │
│   │  • Trajectory validation                                     │  │
│   └─────────────────────────────────────────────────────────────┘  │
│                              │                                      │
│                              ▼                                      │
│   ┌─────────────────────────────────────────────────────────────┐  │
│   │  Layer 3: Control Level                                      │  │
│   │  • Joint limit enforcement                                   │  │
│   │  • Velocity/acceleration limits                              │  │
│   │  • Torque limits                                             │  │
│   └─────────────────────────────────────────────────────────────┘  │
│                              │                                      │
│                              ▼                                      │
│   ┌─────────────────────────────────────────────────────────────┐  │
│   │  Layer 2: Hardware Level                                     │  │
│   │  • Motor driver current limits                               │  │
│   │  • Hardware joint stops                                      │  │
│   │  • Force/torque sensor thresholds                            │  │
│   └─────────────────────────────────────────────────────────────┘  │
│                              │                                      │
│                              ▼                                      │
│   ┌─────────────────────────────────────────────────────────────┐  │
│   │  Layer 1: Emergency Stop                                     │  │
│   │  • Physical E-stop button                                    │  │
│   │  • Power cutoff                                              │  │
│   │  • Always accessible, bypasses all software                  │  │
│   └─────────────────────────────────────────────────────────────┘  │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

**Diagram Explanation**: Safety layers form a hierarchy from high-level task validation down to physical emergency stop. Each layer catches failures that slip through higher layers.

### Key Safety Patterns

**1. Fail-Safe Defaults**
```
If (communication_lost OR sensor_failed OR timeout):
    Stop all motion
    Hold current position
    Alert operator
```

**2. Watchdog Timers**
```
Control loop sets watchdog = True every cycle
Watchdog monitor expects heartbeat
If heartbeat missing: trigger safe stop
```

**3. Redundant Sensing**
- Multiple sensors for critical measurements
- Cross-check between sensors
- Degrade gracefully if sensors disagree

---

## Modularity Principles

Well-designed humanoid AI systems are modular—components can be understood, tested, and replaced independently.

### Component Boundaries

```
┌─────────────────────────────────────────────────────────────────────┐
│              Modular System Design                                  │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   Good Modularity:                                                  │
│   ┌──────────┐    ┌──────────┐    ┌──────────┐                     │
│   │Perception│───▶│Intelligence──▶│ Control  │                     │
│   │          │msg │          │msg │          │                     │
│   └──────────┘    └──────────┘    └──────────┘                     │
│   • Clear interfaces (ROS 2 messages)                              │
│   • Each component testable independently                           │
│   • Can swap implementations (e.g., different VLA models)          │
│                                                                     │
│   ─────────────────────────────────────────────────────────────    │
│                                                                     │
│   Poor Modularity:                                                  │
│   ┌────────────────────────────────────────────┐                   │
│   │  Monolithic system with shared memory      │                   │
│   │  ┌────┐ ┌────┐ ┌────┐                      │                   │
│   │  │ A  │◀┼─┼──┼▶│ B  │◀┼──┼─▶│ C  │        │                   │
│   │  └────┘ └────┘ └────┘                      │                   │
│   │  Everything coupled, hard to test/change   │                   │
│   └────────────────────────────────────────────┘                   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### ROS 2 Enables Modularity

ROS 2 architecture naturally supports modularity:
- **Nodes**: Independent processes with single responsibilities
- **Topics**: Decoupled pub/sub communication
- **Services**: Request/response with defined contracts
- **Actions**: Long-running tasks with feedback
- **Parameters**: Runtime configuration

### Testing Modularity

Each module should be testable in isolation:

| Module | Unit Test Approach |
|--------|-------------------|
| Perception | Feed recorded images, verify detections |
| VLA | Feed images + language, verify action output |
| Controller | Feed trajectory, verify motor commands |
| Safety | Feed dangerous commands, verify rejection |

---

## Failure Handling

Systems fail. The question is how gracefully.

### Failure Categories

1. **Sensor failures**: Camera blackout, encoder noise
2. **Communication failures**: Network partition, message loss
3. **Compute failures**: GPU error, memory exhaustion
4. **Actuator failures**: Motor fault, stuck gripper
5. **Software failures**: Crash, exception, deadlock

### Recovery Patterns

**Graceful Degradation**
```
┌─────────────────────────────────────────────────────────────────────┐
│              Graceful Degradation Example                           │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   Normal: VLA + Isaac perception + Full control                     │
│                     │                                               │
│                     ▼ (VLA fails)                                   │
│   Degraded 1: Fall back to scripted behaviors                       │
│                     │                                               │
│                     ▼ (Perception fails)                            │
│   Degraded 2: Stop task, hold position                              │
│                     │                                               │
│                     ▼ (Control fails)                               │
│   Emergency: E-stop, power down safely                              │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

**Supervision Trees**
- Parent nodes monitor child nodes
- Failed children are restarted
- Persistent failures escalate to parent

---

## System Integration Concerns

### Configuration Management

Robot systems have many parameters:
- Sensor calibration
- Control gains
- Safety limits
- Model paths

**Best Practice**: Centralized parameter files (YAML) loaded at startup, with runtime reconfiguration for tuning.

### Logging and Monitoring

Essential for debugging and improvement:
- **Logging**: Record events, errors, decisions
- **Metrics**: Track latency, throughput, success rates
- **Tracing**: Follow request through system
- **Rosbag**: Record all messages for replay

### Deployment Patterns

Moving from development to deployment:

1. **Simulation first**: Validate in digital twin
2. **Hardware-in-loop**: Sim + real sensors/actuators
3. **Staged rollout**: Controlled environment first
4. **Monitoring**: Watch metrics in production
5. **Rollback**: Quick revert if issues arise

---

## Connection to Previous Modules

- **Module 1 (Foundations)**: Safety and real-time are core to embodied systems operating in the physical world
- **Module 2 (ROS 2)**: ROS 2 provides the modularity framework (nodes, topics) and QoS for real-time
- **Module 3 (Digital Twin)**: Simulation enables testing safety systems without risk
- **Module 4 (Isaac)**: Isaac provides hardware-accelerated perception that meets real-time requirements
- **Module 5 (VLA)**: VLA inference timing and safety integration are critical concerns

---

## Key Takeaways

- **Real-time requirements** vary by layer: hard (control), firm (trajectory), soft (perception/VLA)
- **Safety layers** provide defense in depth from task validation to physical E-stop
- **Modularity** enables independent testing, maintenance, and evolution of components
- **Failure handling** requires graceful degradation and supervision patterns
- ROS 2 architecture naturally supports these concerns through nodes, topics, and QoS

---

**Next**: [Synthesis →](./05-synthesis.md)
