# Code Snippet Template

## Usage

Use this template when embedding illustrative code examples in ConceptSections.

**Requirements**:
- FR-003: Module must have at least 3 code snippets
- FR-028: Code must include comments explaining purpose
- SC-006: Code must be syntactically correct
- All code marked as "illustrative, non-production"

---

## Template

````markdown
```python
# PURPOSE: [Clear explanation of what this code demonstrates]
# NOTE: This is conceptual code, not production-ready

[Code goes here]
[10-50 lines typical]
[Include inline comments for key concepts]
```

**Explanation**: [1-3 sentences explaining what concept this code illustrates and how it relates to the surrounding content. Highlight the key takeaway for readers.]

**Key Concepts Demonstrated**:
- [Concept 1]: [Brief explanation]
- [Concept 2]: [Brief explanation]
- [Optional: Concept 3]
````

---

## Example 1: Perception-Action Loop

````markdown
```python
# PURPOSE: Illustrate the continuous perception-action loop in embodied AI
# NOTE: This is conceptual code, not production-ready

class EmbodiedRobot:
    def __init__(self):
        self.sensors = SensorArray()
        self.actuators = ActuatorArray()
        self.world_model = WorldModel()

    def perceive(self):
        """Gather data from sensors"""
        # Read camera, LiDAR, IMU, etc.
        sensor_data = self.sensors.read_all()
        return sensor_data

    def update_world_model(self, sensor_data):
        """Process sensor data into structured representation"""
        # Extract objects, obstacles, robot pose
        self.world_model.update(sensor_data)

    def decide_action(self):
        """Plan next action based on world model"""
        # Task planning: what to do?
        # Motion planning: how to do it?
        action = self.world_model.plan_next_action()
        return action

    def act(self, action):
        """Execute action via actuators"""
        # Send motor commands
        self.actuators.execute(action)

    def run(self):
        """Main perception-action loop"""
        while True:
            # 1. Perceive environment
            data = self.perceive()

            # 2. Update internal representation
            self.update_world_model(data)

            # 3. Decide action
            action = self.decide_action()

            # 4. Act on environment
            self.act(action)

            # Loop continues: action changes world,
            # which affects next perception
```

**Explanation**: This code demonstrates the fundamental perception-action loop in embodied AI systems. The robot continuously senses its environment, updates an internal world model, decides on actions, and executes them—creating a closed feedback loop where actions influence future perceptions.

**Key Concepts Demonstrated**:
- **Continuous Loop**: Unlike batch processing in traditional AI, embodied agents run continuously
- **World Model**: Internal representation built from sensor data
- **Sensorimotor Coupling**: Actions directly affect what the robot will perceive next
````

---

## Example 2: ROS 2 Publisher-Subscriber Pattern

````markdown
```python
# PURPOSE: Demonstrate basic ROS 2 pub/sub communication pattern
# NOTE: This is conceptual code, not production-ready

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):
    """Publishes velocity commands to /cmd_vel topic"""

    def __init__(self):
        super().__init__('velocity_publisher')

        # Create publisher for velocity commands
        self.publisher = self.create_publisher(
            Twist,           # Message type
            '/cmd_vel',      # Topic name
            10               # Queue size
        )

        # Publish at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_velocity)

    def publish_velocity(self):
        """Publish velocity command"""
        msg = Twist()
        msg.linear.x = 0.5   # Move forward at 0.5 m/s
        msg.angular.z = 0.1  # Turn slowly

        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: linear={msg.linear.x}, angular={msg.angular.z}')

def main():
    rclpy.init()
    node = VelocityPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Explanation**: This code shows a ROS 2 publisher node that sends velocity commands to a robot. The `create_publisher()` method sets up communication on the `/cmd_vel` topic, and the timer ensures commands are sent at 10 Hz—demonstrating the decoupled, asynchronous nature of ROS 2 communication.

**Key Concepts Demonstrated**:
- **Topic-Based Communication**: Publishers and subscribers don't directly reference each other
- **Message Types**: Standard messages (Twist) enable interoperability
- **Asynchronous Publishing**: Timer-based publishing at fixed rate
````

---

## Guidelines

### Code Length
- **Minimum**: 10 lines (enough to show structure)
- **Typical**: 20-30 lines (demonstrates concept without overwhelming)
- **Maximum**: 50 lines (any longer should be broken into multiple snippets)

### Comment Density
- **Required**: PURPOSE and NOTE at top
- **Inline**: Comment every 3-5 lines to explain key concepts
- **Avoid**: Over-commenting obvious code (e.g., `x = 5 # Set x to 5`)

### Language Choice
- **Default**: Python (most common in robotics education)
- **Alternative**: C++ for performance-critical ROS 2 examples (if needed in Iteration 3)

### Syntax Validation
Before embedding code:
1. Copy code to a `.py` file
2. Run: `python -m py_compile filename.py`
3. Fix any syntax errors
4. Verify code executes conceptually (logic is sound)

### Educational Focus
- **Prioritize**: Conceptual clarity over production robustness
- **Simplify**: Remove error handling, edge cases if they obscure the concept
- **Highlight**: The "aha moment" readers should have

---

## Anti-Patterns (Avoid These)

❌ **No PURPOSE comment**:
```python
class Robot:
    def move(self):
        pass
```

❌ **Production-grade code** (too complex for illustration):
```python
# Complex error handling, logging, type hints obscure the concept
def perceive(self) -> Optional[SensorData]:
    try:
        with self.sensor_lock:
            data = self.sensors.read()
            if not self.validate(data):
                self.logger.error("Invalid sensor data")
                return None
            return data
    except SensorException as e:
        self.logger.exception(f"Sensor read failed: {e}")
        return None
```

❌ **No explanation** (just code, no context):
```python
# PURPOSE: ...
# NOTE: ...
[code]
# [Missing explanation of what this demonstrates!]
```

✅ **Good Example** (clear, commented, explained):
```python
# PURPOSE: Show how ROS 2 nodes communicate via topics
# NOTE: Conceptual code, not production-ready

# [10-30 lines of clear code with inline comments]

**Explanation**: [What this illustrates and why it matters]
```

---

## Checklist

Before embedding a code snippet, verify:

- [ ] PURPOSE comment at top (explains what this demonstrates)
- [ ] NOTE about illustrative/non-production nature
- [ ] Inline comments for key concepts (not every line)
- [ ] Explanation paragraph after code block
- [ ] Syntax is valid (runs through linter)
- [ ] Length appropriate (10-50 lines)
- [ ] Concept is clear from code + explanation

---

**Use this template consistently across all modules to ensure quality and uniformity.**
