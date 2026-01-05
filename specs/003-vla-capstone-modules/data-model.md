# Content Architecture: VLA & Capstone Modules

**Feature**: `003-vla-capstone-modules` | **Date**: 2026-01-04 | **Plan**: [plan.md](./plan.md)

---

## Content Model Overview

This document defines the content architecture for Module 5 (VLA) and Module 6 (Capstone). Each section is specified with learning objectives, key concepts, diagrams, and cross-references.

---

## Module 5: Vision-Language-Action Integration

### Module-Level Structure

| Attribute | Value |
|-----------|-------|
| **Estimated Reading Time** | 35 minutes |
| **Prerequisites** | Modules 1-4 (Foundations, ROS 2, Digital Twin, Isaac) |
| **Section Count** | 7 (index + 6 content sections) |
| **Target Word Count** | 6,000-8,000 words |
| **Diagrams** | 5+ text-described diagrams |
| **Pseudocode Examples** | 3-4 illustrative snippets |

---

### index.md - Module Overview

**Purpose**: Introduce VLA concepts and establish learning context

**Content Outline**:
1. Learning Objectives (5 bullet points)
2. Prerequisites
3. Module Overview (what VLA is, why it matters)
4. Key Concepts Preview (terminology list)
5. Connection to Previous Modules
6. What This Module Covers vs Defers

**Key Diagram**: VLA Loop Overview
```
[Visual Observation] → [Language Instruction] → [Action Output]
         ↓                       ↓                    ↓
    "What I see"         "What to do"          "How to do it"
```

**Cross-References**: Module 1 (perception-action), Module 4 (Isaac perception)

---

### 01-vla-fundamentals.md - The Perception-Language-Action Loop

**Learning Objectives**:
- Explain what VLA models are and how they differ from VLMs
- Describe the perception → language → action pipeline
- Identify where VLA fits in the humanoid AI stack

**Content Outline**:
1. What is a VLA Model?
   - Definition and core concept
   - Comparison with Vision-Language Models (VLMs)
2. The Core Loop
   - Visual observation (see)
   - Language instruction (understand)
   - Action prediction (act)
3. Why VLA Matters for Robotics
   - Language as natural interface
   - Generalization from web knowledge
4. VLA in the Robot Stack
   - Position between perception and control

**Key Diagram**: VLA Pipeline
```
┌─────────────┐   ┌─────────────┐   ┌─────────────┐   ┌─────────────┐
│   Camera    │ → │   Vision    │ → │  Language   │ → │   Action    │
│   Input     │   │   Encoder   │   │   Model     │   │   Decoder   │
└─────────────┘   └─────────────┘   └─────────────┘   └─────────────┘
       ↑                                   ↑
       │                                   │
   RGB images                    "Pick up the red cup"
```

**Cross-References**: Module 1 (perception-action loop), Module 4 (Isaac perception)

**Estimated Reading Time**: 5-6 minutes

---

### 02-multimodal-perception.md - Vision Encoders and Multimodal Fusion

**Learning Objectives**:
- Describe how vision encoders convert images to embeddings
- Explain cross-modal fusion strategies
- Compare early vs late fusion approaches

**Content Outline**:
1. Vision Encoders for Robotics
   - Vision Transformers (ViT) overview
   - CLIP-style pre-training
   - Image patches as tokens
2. Language Encoders
   - Tokenization review
   - Transformer language models
3. Cross-Modal Fusion
   - Early fusion (concatenate before processing)
   - Late fusion (process separately, combine at end)
   - Cross-attention (text attends to image regions)
4. Design Considerations
   - Computational cost
   - Pre-training data
   - Real-time constraints

**Key Diagram**: Multimodal Fusion Architecture
```
        Early Fusion                    Late Fusion
┌──────────┐                    ┌──────────┐  ┌──────────┐
│  Image   │                    │  Image   │  │   Text   │
│ Patches  │                    │ Encoder  │  │ Encoder  │
└────┬─────┘                    └────┬─────┘  └────┬─────┘
     │     ┌──────────┐              │             │
     ├────▶│Concatenate├─────▶       └──────┬──────┘
     │     └────┬─────┘                     │
┌────┴─────┐    │                    ┌──────▼──────┐
│   Text   │    │                    │   Combine   │
│  Tokens  │    ▼                    │   Layer     │
└──────────┘  [Joint                 └─────────────┘
              Processing]
```

**Cross-References**: Module 4 (Isaac ROS perception)

**Estimated Reading Time**: 6-7 minutes

---

### 03-language-grounding.md - Understanding Instructions for Action

**Learning Objectives**:
- Define language grounding in robotics context
- Explain object, spatial, and action grounding
- Describe how VLAs ground language to physical state

**Content Outline**:
1. What is Language Grounding?
   - Definition: linking language to physical reality
   - Why grounding is hard for robots
2. Types of Grounding
   - Object grounding ("the red cup")
   - Spatial grounding ("on the table")
   - Action grounding ("pick up", "place")
   - Temporal grounding ("first", "then")
3. Grounding Mechanisms in VLAs
   - Attention-based grounding
   - Learned object-word associations
4. Challenges and Limitations
   - Ambiguity resolution
   - Novel objects and instructions

**Key Diagram**: Grounding Types
```
┌───────────────────────────────────────────────────────────────┐
│                    Language Grounding                          │
├────────────────┬────────────────┬────────────────┬────────────┤
│    Object      │    Spatial     │     Action     │  Temporal  │
│   Grounding    │   Grounding    │   Grounding    │  Grounding │
├────────────────┼────────────────┼────────────────┼────────────┤
│ "red cup" →    │ "on table" →   │ "pick up" →    │ "first" →  │
│ specific       │ location       │ grasp          │ sequence   │
│ object         │ coordinates    │ primitive      │ ordering   │
└────────────────┴────────────────┴────────────────┴────────────┘
```

**Cross-References**: None (new concept)

**Estimated Reading Time**: 5-6 minutes

---

### 04-action-generation.md - From Language to Robot Movement

**Learning Objectives**:
- Explain action tokenization strategies
- Compare discrete vs continuous action representations
- Describe action chunking for smoother control

**Content Outline**:
1. The Action Prediction Problem
   - How do language models output robot actions?
   - Bridging discrete tokens and continuous control
2. Action Tokenization
   - Discretizing continuous actions into bins
   - Actions as "language tokens"
   - RT-1/RT-2 approach
3. Continuous Action Outputs
   - Direct regression to joint positions
   - Diffusion-based action generation
4. Action Chunking
   - Predicting sequences, not single steps
   - ACT and OpenVLA approaches
5. From Actions to Robot Commands
   - End-effector vs joint-level actions
   - Action frequency considerations

**Key Diagram**: Action Representation Strategies
```
┌─────────────────────────────────────────────────────────────────┐
│                    Action Representation                         │
├──────────────────────────┬──────────────────────────────────────┤
│   Discrete Tokens        │   Continuous Values                   │
├──────────────────────────┼──────────────────────────────────────┤
│ x: [-1, 1] → 256 bins    │ x: 0.342                             │
│ y: [-1, 1] → 256 bins    │ y: -0.178                            │
│ z: [-1, 1] → 256 bins    │ z: 0.521                             │
│ gripper: [open, close]   │ gripper: 0.85                        │
├──────────────────────────┼──────────────────────────────────────┤
│ Token sequence: [142,    │ Action vector: [0.342, -0.178,       │
│  89, 201, 1]             │  0.521, 0.85]                        │
└──────────────────────────┴──────────────────────────────────────┘
```

**Pseudocode Example**:
```python
# Illustrative: Action tokenization (non-production)
def tokenize_action(continuous_action, num_bins=256):
    """Convert continuous action to discrete tokens"""
    # Assume action in [-1, 1] range
    bin_index = int((continuous_action + 1) / 2 * (num_bins - 1))
    return bin_index

# VLA inference loop concept
def vla_step(image, instruction, model):
    """Single VLA inference step"""
    visual_tokens = model.encode_image(image)
    language_tokens = model.encode_text(instruction)
    action_tokens = model.predict_actions(visual_tokens, language_tokens)
    return detokenize_actions(action_tokens)
```

**Cross-References**: Module 2 (ROS 2 actions), Module 4 (control interfaces)

**Estimated Reading Time**: 6-7 minutes

---

### 05-architectures.md - VLA Design Patterns

**Learning Objectives**:
- Compare end-to-end vs modular VLA architectures
- Describe RT-2, OpenVLA, and SayCan approaches
- Evaluate tradeoffs for different use cases

**Content Outline**:
1. End-to-End VLAs
   - Definition: Single model from pixels to actions
   - RT-2 architecture overview
   - PaLM-E embodied language model
   - OpenVLA open-source approach
2. Modular VLAs
   - Definition: Separate components with interfaces
   - SayCan: LLM planning + skill policies
   - Code as Policies: LLM generates code
3. Architecture Comparison
   - Training requirements
   - Generalization capabilities
   - Debugging and interpretability
   - Computational requirements
4. When to Use Each Approach
   - End-to-end for emergent behaviors
   - Modular for interpretability and control

**Key Diagram**: Architecture Comparison
```
┌───────────────────────────────────────────────────────────────┐
│                      End-to-End VLA                            │
│  ┌─────────┐  ┌─────────────────────────────┐  ┌──────────┐  │
│  │ Camera  │→ │   Single VLA Model          │→ │ Actions  │  │
│  │ + Text  │  │ (RT-2, OpenVLA, PaLM-E)     │  │          │  │
│  └─────────┘  └─────────────────────────────┘  └──────────┘  │
└───────────────────────────────────────────────────────────────┘

┌───────────────────────────────────────────────────────────────┐
│                       Modular VLA                              │
│  ┌─────────┐  ┌──────────┐  ┌──────────┐  ┌──────────────┐   │
│  │ Camera  │→ │ Object   │→ │   LLM    │→ │ Skill Policy │   │
│  │         │  │ Detector │  │ Planner  │  │ (per action) │   │
│  └─────────┘  └──────────┘  └──────────┘  └──────────────┘   │
│       │            ↑             │               ↓            │
│       │            └─────────────┘          [Actions]         │
│   "Pick up                  "pick(red_cup)"                   │
│    red cup"                                                   │
└───────────────────────────────────────────────────────────────┘
```

**Tradeoff Table**:
| Aspect | End-to-End | Modular |
|--------|------------|---------|
| Training data | Very large (millions) | Moderate per component |
| Generalization | Emergent from scale | Engineered per interface |
| Debugging | Black box | Component inspection |
| Compute (inference) | Single forward pass | Multiple model calls |
| Flexibility | Fixed architecture | Swap components |

**Cross-References**: Module 4 (Isaac ROS modular perception)

**Estimated Reading Time**: 7-8 minutes

---

### 06-integration.md - Connecting VLA to Robot Systems

**Learning Objectives**:
- Describe how VLA connects to ROS 2 systems
- Explain input and output interfaces
- Identify integration patterns for production

**Content Outline**:
1. VLA Input Interfaces
   - Camera topics in ROS 2
   - Robot state feedback
   - Language command input
2. VLA Output Interfaces
   - Velocity commands
   - Action server goals
   - Joint trajectory commands
3. Control Loop Patterns
   - Open-loop execution
   - Closed-loop replanning
   - Hierarchical control
4. Real-Time Considerations
   - VLA inference latency
   - Action execution frequency
   - Buffering and prediction
5. Integration with Isaac
   - Isaac ROS perception feeding VLA
   - Simulation for VLA training

**Key Diagram**: VLA-ROS 2 Integration
```
┌─────────────────────────────────────────────────────────────────┐
│                    VLA Integration Architecture                  │
│                                                                  │
│  ┌──────────────────────┐        ┌──────────────────────┐       │
│  │   ROS 2 Camera       │───────▶│                      │       │
│  │   /camera/image      │        │                      │       │
│  └──────────────────────┘        │                      │       │
│                                  │      VLA Model       │       │
│  ┌──────────────────────┐        │                      │       │
│  │   Language Input     │───────▶│   (Inference at     │       │
│  │   /command/text      │        │    5-10 Hz)         │       │
│  └──────────────────────┘        │                      │       │
│                                  │                      │       │
│  ┌──────────────────────┐        │                      │       │
│  │   Robot State        │───────▶│                      │       │
│  │   /joint_states      │        └──────────┬───────────┘       │
│  └──────────────────────┘                   │                   │
│                                             ▼                   │
│                               ┌──────────────────────┐          │
│                               │   Action Executor    │          │
│                               │   (100+ Hz control)  │          │
│                               └──────────┬───────────┘          │
│                                          │                      │
│              ┌───────────────────────────┼────────────────┐     │
│              ▼                           ▼                ▼     │
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────┐  │
│  │  /cmd_vel        │  │ /move_group      │  │ /gripper     │  │
│  │  (base motion)   │  │ (arm planning)   │  │ (grasp)      │  │
│  └──────────────────┘  └──────────────────┘  └──────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

**Cross-References**: Module 2 (ROS 2 topics, actions), Module 4 (Isaac ROS)

**Estimated Reading Time**: 6-7 minutes

---

## Module 6: Capstone - End-to-End Humanoid AI Architecture

### Module-Level Structure

| Attribute | Value |
|-----------|-------|
| **Estimated Reading Time** | 35 minutes |
| **Prerequisites** | Modules 1-5 (all previous modules) |
| **Section Count** | 6 (index + 5 content sections) |
| **Target Word Count** | 5,000-7,000 words |
| **Diagrams** | 4+ text-described diagrams |
| **Code Examples** | None (pure narrative synthesis) |

---

### index.md - Module Overview

**Purpose**: Synthesize all modules into cohesive understanding

**Content Outline**:
1. Learning Objectives (5 bullet points)
2. Prerequisites (all modules)
3. What This Module Is (synthesis, not new concepts)
4. What This Module Is NOT (hands-on implementation)
5. Module Overview
6. How to Use This Module

**Key Diagram**: Textbook Arc
```
Module 1          Module 2          Module 3          Module 4          Module 5          Module 6
Foundations   →   ROS 2       →    Digital Twin  →    Isaac       →      VLA        →    Capstone
"What"            "Connect"         "Simulate"        "Perceive"        "Reason"         "Integrate"
```

**Cross-References**: All modules

---

### 01-system-overview.md - The Complete Humanoid AI Stack

**Learning Objectives**:
- Describe the full humanoid AI software stack
- Identify each layer's role and responsibility
- Understand data flow from sensors to actuators

**Content Outline**:
1. The Layered Architecture
   - Application layer
   - Intelligence layer (VLA)
   - Perception layer
   - Communication layer
   - Simulation layer
   - Control layer
   - Hardware layer
2. Layer Responsibilities
   - What each layer does
   - What it depends on
   - What depends on it
3. Why Layers Matter
   - Separation of concerns
   - Independent development
   - Testing and validation

**Key Diagram**: Full Stack
```
┌─────────────────────────────────────────────────────────────────┐
│                      APPLICATION LAYER                           │
│              Task-specific behaviors and goals                   │
│         "Pick up the red cup" → "Navigate to kitchen"           │
├─────────────────────────────────────────────────────────────────┤
│                    INTELLIGENCE LAYER (VLA)                      │
│              Vision-Language-Action reasoning                    │
│            Module 5: Perception→Language→Action                  │
├─────────────────────────────────────────────────────────────────┤
│                      PERCEPTION LAYER                            │
│          Object detection, SLAM, pose estimation                 │
│              Module 4: Isaac ROS perception                      │
├─────────────────────────────────────────────────────────────────┤
│                    COMMUNICATION LAYER                           │
│             ROS 2 middleware: topics, services                   │
│          Module 2: Nodes, messages, QoS policies                 │
├─────────────────────────────────────────────────────────────────┤
│                     SIMULATION LAYER                             │
│            Digital twin, sim-to-real transfer                    │
│        Module 3: Gazebo, Unity, synthetic data                   │
├─────────────────────────────────────────────────────────────────┤
│                       CONTROL LAYER                              │
│          Motion planning, trajectory execution                   │
│              Joint controllers, safety limits                    │
├─────────────────────────────────────────────────────────────────┤
│                       HARDWARE LAYER                             │
│            Sensors, actuators, compute platform                  │
│          Module 1: Physical embodiment concepts                  │
└─────────────────────────────────────────────────────────────────┘
```

**Cross-References**: All modules (each layer references specific module)

**Estimated Reading Time**: 6-7 minutes

---

### 02-module-connections.md - How the Pieces Fit Together

**Learning Objectives**:
- Trace connections between textbook modules
- Identify where each module's concepts apply
- Understand dependency relationships

**Content Outline**:
1. Foundation → Everything
   - Perception-action loop enables all higher layers
   - Embodiment shapes design decisions
2. ROS 2 → All Communication
   - Every component communicates via ROS 2
   - Topics for streaming, services for queries
3. Simulation → Development Lifecycle
   - Train in simulation, deploy to real
   - Synthetic data for perception training
4. Isaac → Perception Pipeline
   - GPU-accelerated computer vision
   - Real-time inference for VLA input
5. VLA → Decision Making
   - Consumes perception, produces actions
   - Language interface to robot capabilities
6. Connection Map
   - Visual representation of dependencies

**Key Diagram**: Module Connection Map
```
                    ┌────────────────┐
                    │   Application  │
                    │    (Goals)     │
                    └───────┬────────┘
                            │ uses
                            ▼
                    ┌────────────────┐
                    │   Module 5:    │
                    │     VLA        │◄──── Language commands
                    └───────┬────────┘
                            │ consumes
            ┌───────────────┼───────────────┐
            ▼               ▼               ▼
    ┌───────────────┐ ┌───────────────┐ ┌───────────────┐
    │   Module 4:   │ │   Module 2:   │ │   Module 3:   │
    │    Isaac      │ │    ROS 2      │ │  Digital Twin │
    │ (perception)  │ │(communication)│ │ (simulation)  │
    └───────┬───────┘ └───────┬───────┘ └───────┬───────┘
            │                 │                 │
            └────────────┬────┴────────────┬────┘
                         ▼                 ▼
                 ┌───────────────┐  ┌───────────────┐
                 │   Module 1:   │  │   Control &   │
                 │  Foundations  │  │   Hardware    │
                 │  (concepts)   │  │   (robots)    │
                 └───────────────┘  └───────────────┘
```

**Cross-References**: All modules with specific section links

**Estimated Reading Time**: 6-7 minutes

---

### 03-reference-architecture.md - Complete Data Flow

**Learning Objectives**:
- Trace data flow from sensor to actuator
- Identify processing at each stage
- Understand timing and synchronization

**Content Outline**:
1. The Data Flow Narrative
   - Step-by-step walkthrough
   - What happens at each stage
2. Sensor Data Path
   - Camera → image topic → perception
   - Encoder → joint state topic → control
3. Perception Processing Path
   - Raw image → object detection → scene understanding
4. Intelligence Processing Path
   - Perception features + language → VLA → action commands
5. Control Execution Path
   - Action commands → trajectory planning → motor commands
6. Feedback Loop
   - New sensor readings reflect changed state
   - Continuous loop at varying frequencies

**Key Diagram**: Detailed Data Flow
```
┌─────────────────────────────────────────────────────────────────────────┐
│                           DATA FLOW PATH                                 │
│                                                                          │
│  PHYSICAL WORLD                                                          │
│       │                                                                  │
│       ▼                                                                  │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐          │
│  │ Camera   │───▶│ /camera/ │───▶│ Isaac    │───▶│ Object   │          │
│  │ (30 Hz)  │    │ image    │    │ ROS      │    │ List     │          │
│  └──────────┘    └──────────┘    └──────────┘    └────┬─────┘          │
│                                                       │                  │
│  ┌──────────┐                                         │                  │
│  │ Language │─────────────────────────────────────────┤                  │
│  │ Command  │                                         │                  │
│  └──────────┘                                         │                  │
│                                                       ▼                  │
│                                               ┌──────────────┐           │
│                                               │     VLA      │           │
│                                               │  (5-10 Hz)   │           │
│                                               └──────┬───────┘           │
│                                                      │                   │
│                                                      ▼                   │
│                                               ┌──────────────┐           │
│                                               │   Motion     │           │
│                                               │  Planner     │           │
│                                               └──────┬───────┘           │
│                                                      │                   │
│                                                      ▼                   │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐          │
│  │ Joint    │◀───│ /joint   │◀───│ Control  │◀───│Trajectory│          │
│  │ Motors   │    │ command  │    │ (100 Hz) │    │          │          │
│  └────┬─────┘    └──────────┘    └──────────┘    └──────────┘          │
│       │                                                                  │
│       ▼                                                                  │
│  PHYSICAL WORLD (changed state)                                          │
│       │                                                                  │
│       └──────────────────── [Feedback to cameras] ──────────────────────┘
└─────────────────────────────────────────────────────────────────────────┘
```

**Cross-References**: Module 2 (ROS 2 topics), Module 4 (Isaac perception), Module 5 (VLA)

**Estimated Reading Time**: 7-8 minutes

---

### 04-system-concerns.md - Real-Time, Safety, and Modularity

**Learning Objectives**:
- Identify real-time constraints in humanoid systems
- Describe safety mechanisms at each layer
- Explain benefits of modular architecture

**Content Outline**:
1. Real-Time Constraints
   - Perception: 30+ Hz for visual feedback
   - VLA: 5-10 Hz (current limitation)
   - Control: 100+ Hz for stability
   - Scheduling and priority management
2. Safety Mechanisms
   - Hardware safety (e-stop, force limits)
   - Software safety (workspace limits, collision avoidance)
   - VLA safety (action filtering, uncertainty thresholds)
3. Modularity Benefits
   - Independent development and testing
   - Component replacement
   - Debugging and monitoring
   - ROS 2 node architecture enables modularity
4. Trade-offs
   - Modularity vs. end-to-end optimization
   - Safety vs. capability
   - Real-time vs. intelligence

**Key Diagram**: Safety Layers
```
┌─────────────────────────────────────────────────────────────────┐
│                       SAFETY ARCHITECTURE                        │
│                                                                  │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │                APPLICATION SAFETY                        │    │
│  │        Task-level constraints, mission abort             │    │
│  └─────────────────────────────────────────────────────────┘    │
│                              │                                   │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │                   VLA SAFETY                             │    │
│  │     Action filtering, confidence thresholds, OOD detect  │    │
│  └─────────────────────────────────────────────────────────┘    │
│                              │                                   │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │                 CONTROL SAFETY                           │    │
│  │    Joint limits, velocity limits, collision avoidance    │    │
│  └─────────────────────────────────────────────────────────┘    │
│                              │                                   │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │                 HARDWARE SAFETY                          │    │
│  │    Emergency stop, force/torque limits, thermal limits   │    │
│  └─────────────────────────────────────────────────────────┘    │
│                                                                  │
│  Each layer can independently halt operation                     │
└─────────────────────────────────────────────────────────────────┘
```

**Cross-References**: Module 2 (ROS 2 QoS), Module 4 (Isaac real-time)

**Estimated Reading Time**: 6-7 minutes

---

### 05-synthesis.md - Putting It All Together

**Learning Objectives**:
- Trace a complete task from command to execution
- Connect all module concepts in a single narrative
- Build a complete mental model of humanoid AI

**Content Outline**:
1. The Task: "Pick Up the Red Cup"
   - Full walkthrough from human command to successful grasp
2. Step-by-Step Execution
   - Language understanding
   - Visual perception and object localization
   - VLA reasoning and action prediction
   - Motion planning
   - Execution and feedback
3. What We've Learned
   - Module 1: Why embodiment matters
   - Module 2: How components communicate
   - Module 3: How we develop safely
   - Module 4: How we perceive
   - Module 5: How we reason
   - Module 6: How it all connects
4. Looking Forward
   - What the textbook prepared you for
   - Next steps for hands-on learning
   - The evolving field of Physical AI

**Key Diagram**: Task Execution Trace
```
┌─────────────────────────────────────────────────────────────────────────┐
│               "Pick up the red cup" → Successful Grasp                   │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  1. LANGUAGE INPUT                                                       │
│     Human: "Pick up the red cup"                                         │
│     └── Text tokenized, sent to VLA                                      │
│                                                                          │
│  2. VISUAL PERCEPTION (Module 4)                                         │
│     Camera → Isaac ROS → Objects detected                                │
│     └── Red cup identified at (x=0.3, y=0.2, z=0.1)                     │
│                                                                          │
│  3. VLA REASONING (Module 5)                                             │
│     Visual tokens + "pick up red cup" → Action sequence                  │
│     └── Predicted: [approach, grasp, lift]                               │
│                                                                          │
│  4. MOTION PLANNING (Module 2)                                           │
│     Action goal → MoveIt trajectory planning                             │
│     └── Collision-free path computed                                     │
│                                                                          │
│  5. CONTROL EXECUTION                                                    │
│     Trajectory → Joint commands @ 100 Hz                                 │
│     └── Arm moves toward cup                                             │
│                                                                          │
│  6. GRASP EXECUTION                                                      │
│     Gripper command → Force feedback                                     │
│     └── Cup grasped, force sensor confirms                               │
│                                                                          │
│  7. FEEDBACK LOOP                                                        │
│     New camera frame → VLA continues with lift action                    │
│     └── Task complete when cup at goal height                            │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

**Cross-References**: All modules (explicit references to each)

**Estimated Reading Time**: 8-10 minutes

---

## Cross-Reference Index

| Concept | Module | Section |
|---------|--------|---------|
| Perception-action loop | Module 1 | 02-embodied-intelligence.md |
| ROS 2 topics | Module 2 | 02-communication-patterns.md |
| ROS 2 actions | Module 2 | 03-distributed-control.md |
| Simulation | Module 3 | 01-simulation-purpose.md |
| Sim-to-real | Module 3 | 05-sim-to-real.md |
| Isaac ROS | Module 4 | 03-isaac-ros.md |
| AI inference | Module 4 | 04-ai-inference.md |
| VLA fundamentals | Module 5 | 01-vla-fundamentals.md |
| Action generation | Module 5 | 04-action-generation.md |

---

**Content Architecture Status**: Complete. Ready for Phase 2 content writing.
