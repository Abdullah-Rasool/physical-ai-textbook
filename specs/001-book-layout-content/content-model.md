# Content Model: Book Layout & High-Level Content

**Branch**: `001-book-layout-content` | **Date**: 2025-12-28

## Overview

This document defines the content structure and entities for the Physical AI Textbook. Since this is a documentation project (not a traditional application), the "data model" consists of content types, their attributes, and relationships.

## Content Entities

### Entity: Module

A major section of the textbook covering a specific topic area.

| Attribute | Type | Required | Description |
|-----------|------|----------|-------------|
| title | string | Yes | Display title (e.g., "Foundations of Physical AI") |
| slug | string | Yes | URL-safe identifier (e.g., "foundations") |
| sidebar_position | number | Yes | Order in navigation (1-7) |
| introduction | markdown | Yes | 1-2 paragraph overview |
| key_concepts | list[string] | Yes | 3-5 bullet point concepts |
| system_role | markdown | Yes | Connection to overall humanoid AI system |
| learning_outcomes | list[string] | Yes | What students will learn |
| placeholder_topics | list[string] | Yes | Topics for Iteration 2 |
| further_reading | list[link] | No | External resources |

**Relationships**:
- Module → Chapter (1:many, future)
- Module → Module (references for system flow)

### Entity: Chapter (Iteration 2)

A subdivision within a module covering specific technical content.

| Attribute | Type | Required | Description |
|-----------|------|----------|-------------|
| title | string | Yes | Chapter title |
| parent_module | reference | Yes | Parent module slug |
| sidebar_position | number | Yes | Order within module |
| content | markdown | Yes | Full chapter content |
| code_examples | list[code_block] | No | Runnable code |
| exercises | list[exercise] | No | Hands-on activities |

**Status**: Placeholder only in Iteration 1

### Entity: Placeholder Notice

Standardized indicator for future content.

| Attribute | Type | Required | Description |
|-----------|------|----------|-------------|
| type | enum | Yes | "info" (Docusaurus admonition) |
| title | string | Yes | "Coming in Iteration 2" |
| topics | list[string] | Yes | Planned content topics |

**Format**:
```markdown
:::info Coming in Iteration 2
Detailed chapters, code examples, and hands-on tutorials will be added
in future iterations. Topics to be covered include:
- Topic 1
- Topic 2
- Topic 3
:::
```

## Module Instances

### Module 1: Introduction/Welcome

| Field | Value |
|-------|-------|
| title | Welcome to Physical AI |
| slug | intro |
| sidebar_position | 1 |
| purpose | Book overview, learning path, prerequisites |

### Module 2: Foundations of Physical AI

| Field | Value |
|-------|-------|
| title | Foundations of Physical AI |
| slug | foundations |
| sidebar_position | 2 |
| key_concepts | Physical AI definition, embodiment, perception-action loop, humanoid form factor |
| system_role | Introduces core concepts that underpin all subsequent modules |

### Module 3: The Robotic Nervous System (ROS 2)

| Field | Value |
|-------|-------|
| title | The Robotic Nervous System |
| slug | ros2 |
| sidebar_position | 3 |
| key_concepts | Nodes, topics, services, actions, middleware, distributed systems |
| system_role | Communication backbone connecting all robot components |

### Module 4: The Digital Twin (Gazebo & Unity)

| Field | Value |
|-------|-------|
| title | The Digital Twin |
| slug | digital-twin |
| sidebar_position | 4 |
| key_concepts | Simulation, physics engines, sensor modeling, sim-to-real gap |
| system_role | Safe development environment before real hardware deployment |

### Module 5: The AI–Robot Brain (NVIDIA Isaac)

| Field | Value |
|-------|-------|
| title | The AI–Robot Brain |
| slug | isaac |
| sidebar_position | 5 |
| key_concepts | Isaac Sim, Isaac ROS, perception pipelines, GPU acceleration |
| system_role | AI integration layer connecting learning to action |

### Module 6: Vision–Language–Action (VLA)

| Field | Value |
|-------|-------|
| title | Vision–Language–Action Models |
| slug | vla |
| sidebar_position | 6 |
| key_concepts | Multimodal models, vision transformers, language grounding, action generation |
| system_role | High-level intelligence enabling natural interaction and complex tasks |

### Module 7: Capstone

| Field | Value |
|-------|-------|
| title | Autonomous Humanoid System Overview |
| slug | capstone |
| sidebar_position | 7 |
| key_concepts | System integration, end-to-end pipeline, real-world deployment |
| system_role | Ties all modules together into coherent system understanding |

## Navigation Structure

```
sidebar:
  - type: doc
    id: intro
    label: Welcome
  - type: category
    label: Foundations
    items:
      - foundations/index
  - type: category
    label: "Module 1: ROS 2"
    items:
      - ros2/index
  - type: category
    label: "Module 2: Digital Twin"
    items:
      - digital-twin/index
  - type: category
    label: "Module 3: Isaac"
    items:
      - isaac/index
  - type: category
    label: "Module 4: VLA"
    items:
      - vla/index
  - type: category
    label: Capstone
    items:
      - capstone/index
```

## Content Validation Rules

1. **Every module MUST have**:
   - Title and slug
   - Introduction paragraph (100-200 words)
   - At least 3 key concepts
   - System role paragraph
   - At least 3 learning outcomes
   - Placeholder notice with 3+ future topics

2. **Content MUST NOT contain**:
   - Code blocks (except placeholder examples)
   - Shell commands
   - Configuration files
   - Implementation details

3. **Cross-references**:
   - Each module should reference at least one other module
   - Capstone must reference all 4 core modules
