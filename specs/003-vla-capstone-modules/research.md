# Research: VLA & Capstone Modules

**Feature**: `003-vla-capstone-modules` | **Date**: 2026-01-04 | **Plan**: [plan.md](./plan.md)

---

## Research Summary

This document consolidates research findings for Module 5 (VLA) and Module 6 (Capstone) content development. Research is organized by topic with decisions, rationale, and alternatives documented.

---

## VLA Fundamentals

### Decision: Define VLA as the Intelligence Bridge

**Definition**: Vision-Language-Action (VLA) models are multimodal AI systems that take visual observations and language instructions as input and produce robot actions as output. They represent the "intelligence layer" that bridges perception (what the robot sees) with control (what the robot does).

**Rationale**:
- VLA models extend Vision-Language Models (VLMs) like CLIP, Flamingo, and GPT-4V by adding action prediction capability
- The perception → language → action loop is the core abstraction enabling language-conditioned robot control
- This positions VLA as distinct from pure perception (Module 4) and pure control (Module 2)

**Alternatives Considered**:
- Present VLA as extension of imitation learning → Too narrow, misses language-grounding aspect
- Present VLA as robotics foundation model → Too broad, loses focus on perception-language-action loop

**Key Sources**:
- Brohan et al. (2023). RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control
- Driess et al. (2023). PaLM-E: An Embodied Multimodal Language Model
- Kim et al. (2024). OpenVLA: An Open-Source Vision-Language-Action Model

---

## Perception → Language → Action Loop

### Decision: Present as Three-Stage Pipeline

**Architecture**:
```
[Visual Input] → [Vision Encoder] → [Multimodal Fusion] → [Language Model] → [Action Decoder] → [Robot Actions]
     ↑                                      ↑
     |                                      |
  Camera/Sensors                    "Pick up the red cup"
```

**Stage Breakdown**:

1. **Visual Perception**: Vision encoders (ViT, ResNet) convert camera images into visual tokens/embeddings
2. **Language Understanding**: Language models process instructions and visual context together
3. **Action Generation**: Action heads decode language model outputs into robot control signals

**Rationale**:
- Three-stage model matches how actual VLA architectures work (RT-2, OpenVLA)
- Each stage maps to content readers already understand (vision from Module 4, actions from Module 2)
- Provides clear mental model for understanding different VLA design choices

**Alternatives Considered**:
- Two-stage (perception + action) → Loses the critical language-grounding component
- Four-stage (add planning) → Overcomplicates for conceptual understanding

---

## Multimodal Perception

### Decision: Cover Vision Encoders and Cross-Modal Fusion

**Key Concepts to Explain**:

1. **Vision Encoders**:
   - Vision Transformer (ViT): Treats images as sequences of patches
   - CLIP-style encoders: Pre-trained on image-text pairs
   - EfficientNet/ResNet: CNN-based encoders (still used in some VLAs)

2. **Cross-Modal Fusion**:
   - Early fusion: Combine image and text embeddings before transformer
   - Late fusion: Process separately, combine at decision layer
   - Cross-attention: Allow text to attend to image regions

**Rationale**:
- Readers need to understand how robots "see" before understanding how they act
- Vision encoder choice significantly impacts VLA performance
- Cross-modal fusion is the key architectural innovation enabling language-conditioned perception

**Key Sources**:
- Dosovitskiy et al. (2021). An Image is Worth 16x16 Words (ViT paper)
- Radford et al. (2021). Learning Transferable Visual Models (CLIP paper)
- Alayrac et al. (2022). Flamingo: a Visual Language Model

---

## Language Grounding

### Decision: Focus on Grounding Language to Robot State

**Definition**: Language grounding is the process of connecting natural language instructions to the robot's perceptual and physical state—understanding that "red cup" refers to a specific object in the scene.

**Key Concepts**:

1. **Object Grounding**: Linking language references ("the cup") to detected objects
2. **Spatial Grounding**: Understanding spatial relations ("on the table", "near the door")
3. **Action Grounding**: Mapping verbs ("pick up", "place") to executable skills
4. **Temporal Grounding**: Understanding sequences ("first X, then Y")

**Rationale**:
- Grounding is what distinguishes robot language understanding from general NLP
- Without grounding, language instructions are meaningless to embodied agents
- Connects to existing NLP knowledge while introducing robotics-specific challenges

**Alternatives Considered**:
- Focus on instruction parsing only → Misses the embodied grounding problem
- Deep dive into semantic representations → Too theoretical for practical understanding

---

## Action Representation

### Decision: Cover Action Tokenization and Continuous Control

**Action Representation Strategies**:

1. **Discrete Action Tokens**:
   - Discretize continuous actions into bins (e.g., 256 bins per dimension)
   - Treat actions as language tokens for autoregressive prediction
   - Used by RT-1, RT-2, RT-X

2. **Continuous Action Outputs**:
   - Direct regression to continuous values (x, y, z, gripper)
   - Diffusion-based action generation
   - Used by Diffusion Policy, Action Chunking

3. **Action Chunking**:
   - Predict sequences of actions, not single steps
   - Enables smoother trajectories and better temporal consistency
   - Used by ACT, OpenVLA

**Rationale**:
- Action representation is the key interface between VLA and robot control
- Understanding tokenization explains how language models can output robot actions
- Connects to ROS 2 action concepts from Module 2

**Key Sources**:
- Brohan et al. (2022). RT-1: Robotics Transformer
- Chi et al. (2023). Diffusion Policy: Visuomotor Policy Learning via Action Diffusion
- Zhao et al. (2023). Learning Fine-Grained Bimanual Manipulation (ACT)

---

## VLA Architectures

### Decision: Compare End-to-End vs Modular Approaches

**End-to-End VLA (e.g., RT-2, OpenVLA)**:
- Single model from pixels to actions
- Pre-trained vision-language model fine-tuned for actions
- Pros: Unified training, emergent capabilities
- Cons: Data-hungry, harder to debug, limited interpretability

**Modular VLA (e.g., SayCan, Code-as-Policies)**:
- Separate components for vision, language, planning, control
- Language model plans, separate policies execute
- Pros: Interpretable, reusable components, easier debugging
- Cons: Error propagation, interface design complexity

**Comparison Table**:

| Aspect | End-to-End | Modular |
|--------|-----------|---------|
| Training | Joint, end-to-end | Separate per component |
| Data needs | Very high | Moderate per component |
| Generalization | Emergent from scale | Designed per interface |
| Debugging | Black box | Component-level inspection |
| Flexibility | Fixed architecture | Swap components easily |
| Example | RT-2, PaLM-E | SayCan, Code-as-Policies |

**Rationale**:
- Both approaches have production relevance
- Tradeoff understanding helps readers evaluate different systems
- Matches spec requirement for tradeoff explanations

**Key Sources**:
- Ahn et al. (2022). Do As I Can, Not As I Say: Grounding Language in Robotic Affordances (SayCan)
- Liang et al. (2023). Code as Policies: Language Model Programs for Embodied Control

---

## ROS 2 Integration Patterns

### Decision: Focus on Action Servers and Control Interfaces

**VLA → ROS 2 Integration Points**:

1. **Input Interface**:
   - Subscribe to camera topics (`sensor_msgs/Image`)
   - Subscribe to robot state topics (joint positions, gripper state)
   - Receive language commands (string topic or service)

2. **Output Interface**:
   - Publish to velocity commands (`geometry_msgs/Twist`)
   - Send action goals (MoveIt actions, custom action servers)
   - Control gripper (joint trajectory controllers)

3. **Control Patterns**:
   - Open-loop: VLA predicts action, executes directly
   - Closed-loop: VLA runs at high frequency, continuous replanning
   - Hierarchical: VLA provides goals, low-level controller executes

**Rationale**:
- Readers need to understand how VLA connects to physical robots
- ROS 2 is already covered in Module 2, so integration patterns build on existing knowledge
- Practical for readers who will implement VLA systems

---

## End-to-End Humanoid AI Architecture

### Decision: Present as Layered Stack

**Reference Architecture**:

```
┌─────────────────────────────────────────────────────────────┐
│                     APPLICATION LAYER                        │
│  Task-specific behaviors (pick & place, navigation, HRI)    │
├─────────────────────────────────────────────────────────────┤
│                    INTELLIGENCE LAYER (VLA)                  │
│  Vision-Language-Action models, reasoning, planning          │
├─────────────────────────────────────────────────────────────┤
│                     PERCEPTION LAYER                         │
│  Object detection, SLAM, pose estimation (Isaac ROS)         │
├─────────────────────────────────────────────────────────────┤
│                    COMMUNICATION LAYER                       │
│  ROS 2 middleware, topics, services, actions                 │
├─────────────────────────────────────────────────────────────┤
│                     SIMULATION LAYER                         │
│  Digital twin, sim-to-real transfer (Isaac Sim, Gazebo)     │
├─────────────────────────────────────────────────────────────┤
│                      CONTROL LAYER                           │
│  Motor control, joint trajectories, safety limits           │
├─────────────────────────────────────────────────────────────┤
│                      HARDWARE LAYER                          │
│  Sensors, actuators, compute (Jetson, motors, cameras)       │
└─────────────────────────────────────────────────────────────┘
```

**Rationale**:
- Layered architecture is the standard mental model for complex systems
- Each layer maps to a textbook module for clear cross-referencing
- Provides reusable diagram for Capstone integration narrative

---

## Data Flow in Humanoid AI

### Decision: Trace Complete Path from Sensor to Actuator

**Data Flow Narrative**:

1. **Physical → Digital**: Cameras, LiDAR, IMU, joint encoders produce sensor data
2. **Sensor → Perception**: Isaac ROS processes raw data into semantic understanding
3. **Perception → VLA**: Visual features and state flow to intelligence layer
4. **Language → VLA**: Human instruction enters the loop
5. **VLA → Planning**: High-level actions or goals produced
6. **Planning → Control**: Motion plans, trajectory optimization
7. **Control → Actuators**: Joint commands to motor controllers
8. **Actuators → Physical**: Robot moves, changes world state
9. **Feedback Loop**: New sensor data reflects changed state

**Rationale**:
- Complete data flow is the core Capstone learning objective
- Narrative format enables readers to trace the full loop
- Each step references prior module concepts

---

## System-Level Concerns

### Decision: Address Real-Time, Safety, and Modularity

**Real-Time Constraints**:
- Perception: 30+ Hz for visual feedback
- Control: 100+ Hz for stable motion
- VLA inference: Currently 1-10 Hz (bottleneck)
- ROS 2 QoS: Critical for deadline guarantees

**Safety Considerations**:
- Collision avoidance (from perception and planning)
- Force/torque limits (from control)
- Emergency stop (hardware and software)
- Workspace boundaries (configurable limits)

**Modularity Benefits**:
- Component replacement without full retraining
- Independent testing and debugging
- Scaling different layers independently
- ROS 2 enables modularity through node architecture

**Rationale**:
- System concerns distinguish conceptual understanding from toy examples
- Prepares readers for production humanoid AI development
- Connects to ROS 2 (Module 2) and Isaac (Module 4) content

---

## Example Task Walkthrough

### Decision: Use "Pick Up the Red Cup" as Running Example

**Task**: Robot receives command "Pick up the red cup and place it on the table"

**Walkthrough**:

1. **Language Input**: Human speaks or types command
2. **Visual Input**: Cameras capture scene with red cup
3. **Perception**: Object detector identifies cup location and pose
4. **VLA Processing**:
   - Fuses visual tokens with language instruction
   - Generates action sequence: approach → grasp → lift → move → place
5. **Action Execution**:
   - Action tokens converted to end-effector trajectory
   - Motion planner generates collision-free path
   - Joint controllers execute trajectory
6. **Feedback**:
   - Force sensors detect grasp success
   - Visual feedback confirms cup location
   - VLA adjusts if needed (closed-loop)

**Rationale**:
- Concrete example makes abstract architecture tangible
- Pick-and-place is canonical manipulation task
- Allows tracing full pipeline with specific details

---

## Deferred Topics

The following topics are explicitly deferred to future iterations:

| Topic | Reason for Deferral | Future Iteration |
|-------|---------------------|------------------|
| VLA Training | Requires compute resources, datasets | Iteration 4+ |
| Dataset Creation | Out of scope for conceptual textbook | Iteration 4+ |
| Model Scaling | Advanced topic, rapidly evolving | Iteration 4+ |
| Production Deployment | Hands-on implementation needed | Iteration 4+ |
| Real Hardware Integration | Requires physical equipment | Iteration 4+ |
| Fine-tuning VLA Models | Requires training infrastructure | Iteration 4+ |

---

## Research Gaps (Resolved)

All NEEDS CLARIFICATION items have been resolved:

| Original Gap | Resolution |
|--------------|------------|
| VLA definition | Defined as multimodal model with perception→language→action |
| Action representation | Documented tokenization vs continuous approaches |
| ROS 2 integration | Identified action servers and topic interfaces |
| System architecture | Created layered stack reference architecture |

---

**Research Status**: Complete. Ready for Phase 1 content design.
