# Gazebo Physics Simulation

---

## Introduction

Gazebo is the most widely used open-source robot simulator, deeply integrated with ROS 2. It provides physics-based simulation where robots interact with virtual environments following the laws of physics—gravity pulls objects down, collisions produce forces, and joints move according to applied torques.

**In This Section**:
- Understand what Gazebo provides for robot simulation
- Learn about physics engines and how they approximate reality
- See how robots are modeled using URDF/SDF descriptions
- Explore the integration between Gazebo and ROS 2

---

## What Is Gazebo?

### A Physics Simulation Platform

Gazebo simulates physical phenomena:

- **Rigid Body Dynamics**: Objects have mass, inertia, and respond to forces
- **Collision Detection**: Objects interact when they touch
- **Joint Constraints**: Links connected by hinges, sliders, and other joints
- **Friction and Contact**: Surfaces interact with realistic friction
- **Gravity and Forces**: Environmental and applied forces affect motion

Unlike game engines that prioritize visual appearance, Gazebo prioritizes **physical accuracy**. A simulated robot arm should move the same way a physical one does (within simulation fidelity limits).

### Gazebo Versions

The Gazebo ecosystem has evolved:

| Version | Description | ROS Integration |
|---------|-------------|-----------------|
| Gazebo Classic | Original version (2004-2022) | ROS 1, ROS 2 (legacy) |
| Gazebo (formerly Ignition) | Modern rewrite, modular architecture | ROS 2 native |

This module discusses concepts applicable to both, with focus on modern Gazebo's architecture.

---

## Physics Engines

### How Physics Simulation Works

A physics engine steps through time, computing how objects move:

1. **Collect forces**: Gravity, applied torques, contact forces
2. **Solve constraints**: Joint limits, collision responses
3. **Integrate motion**: Update positions and velocities
4. **Detect collisions**: Find new contacts for next step
5. **Repeat**: Advance to next time step

This happens many times per simulated second (typically 1000 Hz for robot control).

### Available Physics Engines

Gazebo supports multiple physics engines:

| Engine | Characteristics | Use Case |
|--------|----------------|----------|
| **ODE** (Open Dynamics Engine) | Fast, widely tested | General robotics |
| **Bullet** | Good for deformables | Manipulation with soft objects |
| **DART** | Accurate joint dynamics | Precise articulated robots |
| **Simbody** | Biomechanics focus | Human-like motion |

The choice affects simulation speed vs. accuracy tradeoffs.

### Gazebo Physics Architecture

**Architecture Overview**:
This diagram shows how Gazebo's physics simulation pipeline processes each time step.

**Components**:
- **World State**: Current positions, velocities of all objects
- **Force Accumulator**: Collects gravity, applied forces, contact forces
- **Constraint Solver**: Resolves joint constraints and collision responses
- **Integrator**: Updates positions and velocities based on forces
- **Collision Detector**: Finds new contacts between objects
- **Sensor Plugins**: Generate sensor data from updated world state

**Data Flow**:
1. Read current world state (positions, velocities)
2. Apply external forces (gravity, motor torques, user commands)
3. Detect collisions and compute contact forces
4. Solve constraint equations (joints, contacts)
5. Integrate equations of motion (new positions, velocities)
6. Update sensor readings based on new state
7. Publish state and sensor data to ROS 2
8. Wait for next time step, repeat

**Key Interfaces**:
- **SDF/URDF Parser**: Loads robot and world descriptions
- **Plugin API**: Extend physics with custom models
- **ros_gz Bridge**: Connects Gazebo topics to ROS 2 topics

**Takeaway**: Physics simulation is a loop of force accumulation, constraint solving, and integration—repeated thousands of times per second to approximate continuous physics.

---

## Robot Description: URDF and SDF

### Describing a Robot

To simulate a robot, Gazebo needs a description:
- What are the robot's parts (links)?
- How are they connected (joints)?
- What are their physical properties (mass, inertia)?
- What do they look like (visual geometry)?
- Where are collisions detected (collision geometry)?

Two formats are used: **URDF** (ROS standard) and **SDF** (Gazebo native).

### URDF: Unified Robot Description Format

URDF is an XML format that describes robot kinematics and some dynamics:

```xml
<!-- PURPOSE: Show URDF structure for a simple robot arm link -->
<!-- NOTE: This is a simplified example, not complete robot description -->

<robot name="simple_arm">
  <!-- Base link (fixed to world) -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <!-- First arm segment -->
  <link name="arm_link_1">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.004" iyy="0.004" izz="0.001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <!-- Joint connecting base to arm -->
  <joint name="joint_1" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link_1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>  <!-- Rotation around Z axis -->
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
</robot>
```

**Key Elements**:
- **link**: A rigid body with mass, inertia, visual, and collision geometry
- **joint**: Connects two links with constraints (revolute, prismatic, fixed)
- **inertial**: Mass and inertia tensor for dynamics
- **visual**: What the robot looks like (for rendering)
- **collision**: Simplified geometry for collision detection

### SDF: Simulation Description Format

SDF is more expressive than URDF, supporting:
- Multiple robots in one file
- Lights, cameras, physics parameters
- Sensors defined inline
- World properties

Gazebo can use either format, but SDF is native.

---

## ROS 2 Integration

### Gazebo-ROS Bridge

Gazebo and ROS 2 communicate through the **ros_gz** bridge:

- Gazebo topics ↔ ROS 2 topics
- Gazebo services ↔ ROS 2 services
- Clock synchronization
- TF transforms

This means your ROS 2 nodes don't know (or care) whether data comes from simulation or real hardware.

```python
# PURPOSE: Show that ROS 2 code is identical for simulation and real robot
# NOTE: This is conceptual code, not production-ready

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class RobotController(Node):
    """
    This controller works identically whether connected to:
    - Gazebo simulation (via ros_gz bridge)
    - Real robot hardware (via hardware drivers)
    """

    def __init__(self):
        super().__init__('robot_controller')

        # Subscribe to camera - works with simulated or real camera
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',  # Same topic name in sim and real
            self.image_callback,
            10
        )

        # Publish velocity commands - works with simulated or real base
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',  # Same topic name in sim and real
            10
        )

        self.get_logger().info('Controller started - works in sim or real!')

    def image_callback(self, msg):
        """Process camera image (simulated or real)"""
        # Detect objects in image
        objects = self.detect_objects(msg)

        # Compute velocity command
        cmd = self.compute_velocity(objects)

        # Send command (to simulation or real motors)
        self.cmd_pub.publish(cmd)
```

**Explanation**: This controller subscribes to camera images and publishes velocity commands. The same code runs unchanged in simulation (Gazebo provides simulated camera and accepts simulated motor commands) and on real hardware (drivers provide real camera and send real motor commands).

**Key Insight**: ROS 2's abstraction layer makes simulation and reality interchangeable from the software's perspective.

### Simulation Time

Gazebo can run faster or slower than real-time. ROS 2 handles this through **/clock** topic:

- Gazebo publishes simulation time to `/clock`
- ROS 2 nodes use simulation time instead of wall-clock time
- Timestamps, timers, and rates stay consistent

This enables accelerated training (simulation runs 10x real-time) while keeping software logic correct.

---

## Simulation Workflow

### Typical Development Flow

1. **Create URDF/SDF**: Define robot geometry, joints, inertia
2. **Spawn in Gazebo**: Load robot into simulation world
3. **Run ROS 2 Nodes**: Connect controllers, perception, planning
4. **Test and Iterate**: Fix bugs, tune parameters
5. **Validate**: Ensure behavior matches expectations
6. **Deploy**: Move to real robot (same ROS 2 code!)

### Example: Simulating a Mobile Robot

```python
# PURPOSE: Illustrate spawning and controlling a robot in Gazebo via ROS 2
# NOTE: This is conceptual code, not production-ready

from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Twist
import rclpy

def spawn_robot(node):
    """Spawn a robot model in Gazebo"""
    # Create service client for spawning
    spawn_client = node.create_client(SpawnEntity, '/spawn_entity')
    spawn_client.wait_for_service()

    # Load robot description from file
    with open('robot.urdf', 'r') as f:
        robot_urdf = f.read()

    # Create spawn request
    request = SpawnEntity.Request()
    request.name = 'my_robot'
    request.xml = robot_urdf
    request.initial_pose.position.x = 0.0
    request.initial_pose.position.y = 0.0
    request.initial_pose.position.z = 0.1

    # Spawn robot in Gazebo
    future = spawn_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    return future.result()

def drive_robot(node, linear_vel, angular_vel):
    """Send velocity command to simulated robot"""
    cmd_pub = node.create_publisher(Twist, '/cmd_vel', 10)

    cmd = Twist()
    cmd.linear.x = linear_vel
    cmd.angular.z = angular_vel

    # Same command interface as real robot!
    cmd_pub.publish(cmd)
```

**Explanation**: This code demonstrates spawning a robot in Gazebo and commanding it to move. The `SpawnEntity` service loads the robot model, and `/cmd_vel` commands work identically to how they would with a real robot.

---

## Gazebo Limitations

### What Gazebo Does Well
- Rigid body dynamics with multiple physics engines
- ROS 2 integration (native bridge)
- Open source and community-supported
- Reasonable accuracy for many robot types

### Where Gazebo Falls Short
- **Rendering quality**: Not photorealistic (important for vision AI)
- **Soft body simulation**: Limited deformable object support
- **GPU acceleration**: Primarily CPU-based physics
- **Large-scale scenes**: Performance degrades with many objects

These limitations are why high-fidelity rendering tools like Unity (next section) and NVIDIA Isaac Sim (Module 4) complement Gazebo.

---

## Summary

**Key Takeaways**:
- **Gazebo** is the standard open-source physics simulator for ROS 2 robotics
- **Physics engines** (ODE, Bullet, DART) compute forces, collisions, and motion
- **URDF/SDF** describe robots: links, joints, mass, inertia, geometry
- **ros_gz bridge** connects Gazebo to ROS 2—same code works in sim and real
- **Simulation time** via `/clock` enables accelerated or slowed simulation
- **Limitations** in rendering quality motivate complementary tools

**Connection to Next Section**: For perception AI training, visual fidelity matters. We'll explore **Unity**, which provides photorealistic rendering for synthetic data generation.

---

**[Continue to Section 3: Unity Visualization →](./03-unity-visualization.md)**
