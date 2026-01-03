# Content Structure Model: Core Module Content Expansion

**Feature**: 002-module-content-expansion
**Phase**: 1 (Design & Structure)
**Purpose**: Define entities, attributes, and relationships for educational content

---

## Overview

This document defines the **content architecture** for the Physical AI & Humanoid Robotics textbook. Unlike traditional software data models, this model describes educational content entities and their composition rules.

**Content Type**: Educational/Instructional (not application code)
**Storage Format**: Markdown files in `docs/` directory hierarchy
**Validation**: Spec-driven requirements (FR-001 through FR-030)

---

## Core Entities

### Entity: Module

A self-contained educational unit covering a major topic area.

**Attributes**:
- `title` (string, required): Full module name (e.g., "Foundations of Physical AI & Embodied Intelligence")
- `module_number` (integer, required): Sequential order (1-6)
- `directory_name` (string, required): Filesystem directory (e.g., "01-foundations")
- `learning_objectives` (list[string], required): 3-5 measurable learning outcomes
- `prerequisite_modules` (list[string], optional): References to prior modules or "None"
- `estimated_reading_time` (integer, required): Minutes for focused reading (20-40 range)
- `iteration_scope` (string, required): "iteration-2" for current scope, "iteration-3" for deferred

**Relationships**:
- **contains** → ConceptSection (1 to many, ordered)
- **contains** → IterationPlaceholder (1 to many)

**Validation Rules**:
- FR-001: Must have 3-5 learning objectives
- FR-002: Must contain 3-5 major ConceptSections (minimum)
- FR-006: Must declare prerequisites explicitly
- SC-003: Reading time must be 20-40 minutes
- Must have `index.md` as entry point

**Example Instance**:
```yaml
title: "Foundations of Physical AI & Embodied Intelligence"
module_number: 1
directory_name: "01-foundations"
learning_objectives:
  - "Explain the difference between traditional AI and Physical AI"
  - "Describe the perception-action loop in embodied systems"
  - "Justify the relevance of humanoid form factors"
prerequisite_modules: ["None"]
estimated_reading_time: 30
iteration_scope: "iteration-2"
```

---

### Entity: ConceptSection

A focused explanation of a specific technical concept within a module.

**Attributes**:
- `title` (string, required): Section heading (e.g., "What is Physical AI?")
- `file_name` (string, required): Markdown filename (e.g., "01-what-is-physical-ai.md")
- `content_markdown` (string, required): Educational content in Markdown format
- `order` (integer, required): Sequence within module
- `word_count_target` (integer, guideline): 500-1500 words typical

**Relationships**:
- **belongs_to** → Module (many to 1)
- **may_contain** → CodeSnippet (0 to many)
- **may_contain** → DiagramDescription (0 to many)

**Validation Rules**:
- Content must be technically accurate (Phase 0 research verified)
- Must be appropriate for CS/robotics undergrad/grad audience (FR-026)
- Must be original content (FR-027)
- Must follow consistent structure (FR-029)
- Must be Docusaurus-compatible Markdown (FR-030)

**Example Instance**:
```yaml
title: "What is Physical AI?"
file_name: "01-what-is-physical-ai.md"
order: 1
word_count_target: 800
contains:
  - CodeSnippet: "perception_action_loop_example"
  - DiagramDescription: "physical_ai_vs_traditional_ai"
```

---

### Entity: CodeSnippet

Illustrative (non-production) code demonstrating a concept.

**Attributes**:
- `language` (string, required): Programming language (default: "python")
- `code_text` (string, required): Actual code content
- `purpose_comment` (string, required): Explanation of what this demonstrates
- `illustrative_flag` (boolean, required): Always `true` (marks as non-production)
- `syntax_valid` (boolean, required): Must be syntactically correct

**Relationships**:
- **belongs_to** → ConceptSection (many to 1)

**Validation Rules**:
- FR-003: Module must have at least 3 CodeSnippets
- FR-028: Must include comments explaining conceptual purpose
- SC-006: Must be syntactically correct
- Must be marked as "illustrative, non-production" in comments

**Example Instance**:
```python
# PURPOSE: Illustrate the perception-action loop in a simple robot
# NOTE: This is conceptual code, not production-ready

class SimpleRobot:
    def perception(self):
        """Sense the environment"""
        sensor_data = self.read_sensors()
        return sensor_data

    def decision(self, sensor_data):
        """Process and decide action"""
        action = self.compute_action(sensor_data)
        return action

    def action(self, action):
        """Execute the action"""
        self.actuate(action)

    def perception_action_loop(self):
        """Continuous sensing and acting"""
        while True:
            data = self.perception()
            action = self.decision(data)
            self.action(action)
```

**Metadata**:
```yaml
language: "python"
purpose_comment: "Demonstrates the perception-action loop concept"
illustrative_flag: true
syntax_valid: true
```

---

### Entity: DiagramDescription

Text-based description of a system architecture or concept diagram.

**Attributes**:
- `title` (string, required): Diagram name (e.g., "End-to-End Humanoid AI Pipeline")
- `description_text` (string, required): Prose description of visual diagram
- `components` (list[string], required): Key elements in the diagram
- `relationships` (list[string], required): Connections between components
- `data_flow` (list[string], optional): Sequential flow of data/control

**Relationships**:
- **belongs_to** → ConceptSection (many to 1)

**Validation Rules**:
- FR-004: Module must have at least 2 DiagramDescriptions
- Must enable reader visualization without image assets
- Must describe architecture clearly (components, interfaces, flow)

**Example Instance**:
```yaml
title: "Physical AI vs Traditional AI Architecture"
description_text: |
  This diagram contrasts two architectural approaches:

  **Traditional AI (Left Side)**:
  - Input: Digital data (text, images, structured datasets)
  - Processing: Neural networks, statistical models (cloud/GPU)
  - Output: Predictions, classifications, generated content
  - Environment interaction: Indirect (via digital interfaces)

  **Physical AI (Right Side)**:
  - Input: Physical sensors (cameras, LiDAR, IMU, tactile)
  - Processing: Perception → World model → Action planning (edge/robot)
  - Output: Motor commands, actuator control
  - Environment interaction: Direct (embodied in physical world)

  **Key Difference (Center Arrow)**:
  Physical AI closes the perception-action loop in real-time with
  the physical world, while traditional AI operates on abstracted
  digital representations.

components:
  - "Traditional AI: Digital Input → Model → Digital Output"
  - "Physical AI: Sensors → Perception → Action → Actuators"
  - "Feedback Loop: Environment affects next sensor input"

relationships:
  - "Traditional AI: One-way data flow"
  - "Physical AI: Closed-loop sensorimotor coupling"

data_flow:
  - "Environment → Sensors → Perception Module"
  - "Perception → World Model → Action Planner"
  - "Action Planner → Motor Controller → Actuators"
  - "Actuators → Environment (loop closes)"
```

---

### Entity: IterationPlaceholder

Standardized notice for advanced topics deferred to future iterations.

**Attributes**:
- `deferred_topic` (string, required): Name of advanced topic (e.g., "Deep RL for Robot Control")
- `preview_text` (string, required): Brief description of what will be covered
- `target_iteration` (string, required): When this will be addressed (e.g., "Iteration 3")
- `rationale` (string, optional): Why deferred (complexity, dependencies, etc.)

**Relationships**:
- **belongs_to** → Module (many to 1, typically in "advanced-topics" section)

**Validation Rules**:
- FR-005: Module must end with placeholders for advanced topics
- SC-005: Each module has at least 1 placeholder section
- Must clearly indicate out-of-scope content

**Example Instance**:
```markdown
## Advanced Topics (Iteration 3)

The following topics will be covered in hands-on tutorials in Iteration 3:

### Deep Reinforcement Learning for Robot Control
**Preview**: Learn how to train robot policies using deep RL algorithms
(PPO, SAC, TD3) in Isaac Sim. Includes sim-to-real transfer techniques and
policy deployment on physical robots.

**Why Deferred**: Requires Isaac Sim environment setup, GPU infrastructure,
and hands-on coding exercises beyond the scope of conceptual introduction.

### Production-Grade ROS 2 Deployment
**Preview**: Build robust, production-ready ROS 2 systems with proper error
handling, monitoring, and deployment pipelines. Includes containerization,
CI/CD, and cloud integration.

**Why Deferred**: Focuses on engineering practices best learned through
hands-on implementation after conceptual understanding is established.
```

**Metadata**:
```yaml
placeholders:
  - deferred_topic: "Deep RL for Robot Control"
    preview_text: "Train robot policies using PPO, SAC, TD3 in Isaac Sim"
    target_iteration: "Iteration 3"
    rationale: "Requires hands-on environment setup"

  - deferred_topic: "Production-Grade ROS 2 Deployment"
    preview_text: "CI/CD, monitoring, containerization for robot systems"
    target_iteration: "Iteration 3"
    rationale: "Engineering best practices learned through implementation"
```

---

## Content Composition Rules

### Module Composition

A valid Module in Iteration 2 must:

1. **Structure**:
   - `index.md` (module overview + learning objectives)
   - 3-5 ConceptSection files (`01-*.md`, `02-*.md`, etc.)
   - 1 IterationPlaceholder file (`0X-advanced-topics.md`)

2. **Content Requirements**:
   - 3-5 learning objectives (FR-001)
   - 3-5 major sections (FR-002)
   - ≥3 code snippets across all sections (FR-003)
   - ≥2 diagram descriptions across all sections (FR-004)
   - 1 advanced topics placeholder section (FR-005)
   - Prerequisites declaration (FR-006)

3. **Quality Standards**:
   - All content technically accurate (verified in research.md)
   - Target audience: CS/robotics undergrad/grad (FR-026)
   - Original content only (FR-027)
   - Code snippets with explanatory comments (FR-028)
   - Consistent structure across modules (FR-029)
   - Docusaurus-compatible Markdown (FR-030)

### ConceptSection Composition

A valid ConceptSection must:

1. **Structure**:
   - Clear heading (level 2: `##`)
   - Introduction paragraph (what this section covers)
   - Core explanation (concepts, definitions, context)
   - Optional: CodeSnippet embedded
   - Optional: DiagramDescription embedded
   - Concluding summary or transition

2. **Length**: 500-1500 words typical (adjustable for concept complexity)

3. **Progression**: Concepts build on prior sections within the module

### CodeSnippet Guidelines

1. **Language**: Python (default for robotics education)
2. **Length**: 10-50 lines typical (focus on clarity over comprehensiveness)
3. **Comments**:
   - `# PURPOSE: [What this demonstrates]` (required first line)
   - `# NOTE: This is conceptual code, not production-ready` (required second line)
   - Inline comments explaining key concepts
4. **Syntax**: Must be valid Python (verifiable via linter)

### DiagramDescription Guidelines

1. **Components**: List all major elements (boxes, nodes, modules)
2. **Relationships**: Describe connections and interactions
3. **Data Flow**: Sequential steps if applicable (numbered list)
4. **Interfaces**: APIs, messages, protocols connecting components
5. **Length**: 100-300 words typical

---

## Validation Matrix

| Entity | FR Requirements | Success Criteria | Validation Method |
|--------|----------------|------------------|-------------------|
| Module | FR-001 to FR-006 | SC-001 to SC-009 | Checklist + Human Review |
| ConceptSection | FR-026, FR-027, FR-029, FR-030 | SC-003, SC-004, SC-007 | Content review + Build test |
| CodeSnippet | FR-003, FR-028 | SC-006 | Syntax linter + Manual review |
| DiagramDescription | FR-004 | SC-002, SC-007 | Clarity review |
| IterationPlaceholder | FR-005 | SC-005 | Count validation |

---

## Example: Module 1 Structure

```text
docs/01-foundations/
├── index.md                        # Module overview + learning objectives
├── 01-what-is-physical-ai.md       # ConceptSection 1
│   ├── DiagramDescription: "Physical AI vs Traditional AI"
│   └── CodeSnippet: "Simple perception-action loop"
├── 02-embodied-intelligence.md     # ConceptSection 2
│   ├── DiagramDescription: "Sensorimotor coupling"
│   └── CodeSnippet: "Embodied agent example"
├── 03-humanoid-form-factor.md      # ConceptSection 3
│   └── CodeSnippet: "Humanoid kinematics concept"
├── 04-system-overview.md           # ConceptSection 4
│   └── DiagramDescription: "End-to-end humanoid AI pipeline"
└── 05-advanced-topics.md           # IterationPlaceholder
```

**Entity Count**:
- Module: 1
- ConceptSections: 5
- CodeSnippets: 3
- DiagramDescriptions: 3
- IterationPlaceholders: 1

**Validation**: ✅ Meets all FR and SC requirements for Module 1

---

## Directory Naming Convention

| Module Number | Directory Name | Module Title |
|--------------|----------------|--------------|
| 1 | `01-foundations` | Foundations of Physical AI & Embodied Intelligence |
| 2 | `02-ros2` | ROS 2 – The Robotic Nervous System |
| 3 | `03-digital-twin` | Digital Twin – Gazebo & Unity Simulation |
| 4 | `04-isaac` | NVIDIA Isaac – AI Robot Brain |
| 5 | `05-vla` | Vision-Language-Action Integration (Iteration 3) |
| 6 | `06-capstone` | Capstone: Building a Humanoid AI System (Iteration 3) |

**File Naming**: `{order}-{slug}.md` (e.g., `01-what-is-physical-ai.md`)

---

## Content Writing State Machine

```text
[Research Complete]
    ↓
[Draft Learning Objectives]
    ↓
[Write ConceptSections] → [Embed CodeSnippets] → [Embed DiagramDescriptions]
    ↓
[Add IterationPlaceholders]
    ↓
[Validate Against FR/SC]
    ↓
[Docusaurus Build Test]
    ↓
[Human Review] → [Revise if needed] → [Mark Complete]
```

**State Persistence**: Track completion state in tasks.md (generated by `/sp.tasks`)

---

## Summary

This data model defines:
- **5 core entities**: Module, ConceptSection, CodeSnippet, DiagramDescription, IterationPlaceholder
- **Composition rules**: How entities combine to form valid educational content
- **Validation criteria**: FR and SC requirements mapped to entities
- **Example structures**: Concrete instance for Module 1

**Next Step**: Use this model to create content templates (contracts/) and writing workflow (quickstart.md).
