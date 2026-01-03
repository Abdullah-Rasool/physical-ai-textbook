# Module Writing Workflow: Quick Start Guide

**Feature**: 002-module-content-expansion
**Phase**: 1 (Design & Structure)
**Purpose**: Step-by-step process for creating high-quality module content

---

## Overview

This guide provides a systematic workflow for writing each module in the Physical AI & Humanoid Robotics textbook. Following this process ensures:

- **Technical Accuracy**: All claims verified against authoritative sources
- **Consistency**: All modules follow the same structure and quality standards
- **Spec Compliance**: All FR and SC requirements satisfied
- **Human Review**: Quality gates enforced before marking tasks complete

**Expected Time per Module**: 4-8 hours of focused writing + 1-2 hours review

---

## Prerequisites

Before starting ANY module, ensure:

- [ ] **Research complete**: `research.md` filled for this module (all findings verified)
- [ ] **Data model understood**: Read `data-model.md` to understand content entities
- [ ] **Templates ready**: Review `contracts/` templates
- [ ] **Spec reviewed**: Refresh on FR requirements for this specific module (FR-007-010 for Module 1, etc.)
- [ ] **Directory exists**: Create `docs/{module-directory}/` if not present

---

## Workflow Steps

### Step 1: Verify Research

**Objective**: Confirm all technical claims for this module are verified.

**Actions**:
1. Open `specs/002-module-content-expansion/research.md`
2. Navigate to your module section (e.g., "Module 1: Foundations")
3. Verify all research tasks marked complete (no "NEEDS RESEARCH" items)
4. Note key findings, definitions, and source citations
5. Identify content guidance for depth and examples

**Validation**:
- [ ] All research tasks for this module have "Findings" filled
- [ ] Sources are authoritative (official docs, peer-reviewed papers)
- [ ] Content guidance is clear and actionable

**If research incomplete**: STOP. Complete research tasks before proceeding.

---

### Step 2: Draft Learning Objectives

**Objective**: Write 3-5 measurable learning objectives for the module.

**Actions**:
1. Open `docs/{module-directory}/index.md` (create if needed)
2. Use template from `contracts/module-template.md`
3. Write learning objectives using format: "After completing this module, you will be able to..."
4. Ensure objectives are:
   - **Measurable**: Use verbs like "explain," "describe," "implement," "analyze"
   - **Aligned with FR**: Cover module-specific requirements (e.g., FR-007-010 for Module 1)
   - **Progressive**: Build from basic to advanced understanding

**Example (Module 1)**:
```markdown
## Learning Objectives

After completing this module, you will be able to:

1. **Explain** the difference between traditional AI and Physical AI, including key distinguishing characteristics
2. **Describe** the perception-action loop and its role in embodied intelligence systems
3. **Justify** why humanoid form factors are relevant for physical AI applications
4. **Visualize** the end-to-end humanoid AI system architecture, from sensing to action
```

**Validation**:
- [ ] 3-5 objectives written (FR-001)
- [ ] Each objective is measurable (action verb + specific outcome)
- [ ] Objectives align with acceptance scenarios in spec.md

---

### Step 3: Write Concept Sections

**Objective**: Create 3-5 focused ConceptSection files covering core topics.

**Actions**:
1. Identify 3-5 core concepts from FR requirements (e.g., Module 1: Physical AI definition, embodiment, humanoid rationale, system overview)
2. For each concept:
   a. Create file: `docs/{module-directory}/{order}-{slug}.md`
   b. Use `contracts/concept-section-template.md` as starting point
   c. Write 500-1500 words explaining the concept
   d. Target audience: CS/robotics undergrad/grad (assume programming background)
   e. Use precise technical terminology with definitions where needed
3. Embed CodeSnippets (see Step 3a)
4. Embed DiagramDescriptions (see Step 3b)

**Content Structure** (per section):
```markdown
# {Concept Title}

## Introduction
[1-2 sentences: What this section covers]

## Core Explanation
[Main content: definitions, context, examples]
[Technical depth appropriate for target audience]

## {Optional: Code Example}
[CodeSnippet using template from contracts/]

## {Optional: Architecture Overview}
[DiagramDescription using template from contracts/]

## Summary
[Key takeaways, transition to next section]
```

**Writing Guidelines**:
- **Clarity First**: Prioritize explanation clarity over academic formality
- **No Jargon Overload**: Define domain-specific terms on first use
- **Progressive Complexity**: Simple → Advanced within each section
- **Original Content**: Write from verified research, never copy-paste
- **Technical Accuracy**: Cross-reference with research.md findings

**Validation**:
- [ ] 3-5 ConceptSections created (FR-002)
- [ ] Each section 500-1500 words
- [ ] Content appropriate for target audience (FR-026)
- [ ] All content original (FR-027)
- [ ] Consistent structure across sections (FR-029)

---

### Step 3a: Embed Code Snippets

**Objective**: Add at least 3 illustrative code examples across all sections.

**Actions**:
1. Identify concepts that benefit from code illustration
2. Use `contracts/code-snippet-template.md`
3. Write Python code demonstrating the concept (10-50 lines typical)
4. **REQUIRED**: Include purpose comment and non-production note
5. Add explanatory comments for key lines
6. Verify syntax correctness (run through linter if possible)

**Example**:
````markdown
```python
# PURPOSE: Illustrate the perception-action loop in a simple robot
# NOTE: This is conceptual code, not production-ready

class EmbodiedAgent:
    def __init__(self):
        self.world_state = {}

    def perceive(self):
        """Gather sensor data from environment"""
        # In real robots: camera, LiDAR, IMU, etc.
        sensor_data = self.read_sensors()
        return sensor_data

    def act(self, sensor_data):
        """Decide and execute action based on perception"""
        # Process sensor data
        action = self.compute_action(sensor_data)
        # Execute motor commands
        self.execute(action)

    def run(self):
        """Main perception-action loop"""
        while True:
            data = self.perceive()  # Sense
            self.act(data)          # Act
            # Loop continues: action changes world, affects next perception
```

**Explanation**: This code demonstrates the core perception-action loop concept in embodied AI. The agent continuously senses the environment (`perceive()`), processes that information, and takes actions (`act()`) that modify the world, which in turn affects future perceptions—creating a closed feedback loop.
````

**Validation**:
- [ ] At least 3 CodeSnippets across module (FR-003)
- [ ] All code includes purpose comment (FR-028)
- [ ] All code syntactically correct (SC-006)
- [ ] Code marked as "illustrative, non-production"

---

### Step 3b: Embed Diagram Descriptions

**Objective**: Add at least 2 text-described architecture diagrams.

**Actions**:
1. Identify system architectures or concept flows that need visualization
2. Use `contracts/diagram-description-template.md`
3. Describe diagram in prose (components, relationships, data flow)
4. Enable reader to visualize the architecture without image assets

**Example**:
```markdown
### System Architecture: End-to-End Humanoid AI Pipeline

**Architecture Overview**:

The humanoid AI system consists of four major layers, connected in a pipeline:

1. **Perception Layer** (Bottom):
   - Components: Cameras, LiDAR, IMU, tactile sensors, joint encoders
   - Output: Raw sensor data (images, point clouds, acceleration, force)
   - Interface: Sensor data published to ROS 2 topics

2. **Processing Layer** (Middle-Left):
   - Components: Isaac ROS perception modules, sensor fusion, SLAM
   - Input: Raw sensor data from Perception Layer
   - Output: Structured environment representation (object poses, obstacles, odometry)
   - Interface: Perception results on ROS 2 topics

3. **Decision Layer** (Middle-Right):
   - Components: Task planner, motion planner, VLA model (Iteration 3)
   - Input: Environment representation from Processing Layer
   - Output: Action commands (target poses, trajectories)
   - Interface: Action commands on ROS 2 action servers

4. **Actuation Layer** (Top):
   - Components: Motor controllers, inverse kinematics, trajectory execution
   - Input: Action commands from Decision Layer
   - Output: Joint torque/velocity commands to physical motors
   - Interface: Hardware driver nodes (ROS 2 control)

**Data Flow**:
1. Sensors capture environment → Perception Layer
2. Perception Layer publishes raw data → Processing Layer
3. Processing Layer extracts structured information → Decision Layer
4. Decision Layer plans actions → Actuation Layer
5. Actuation Layer moves robot → Environment changes
6. Changed environment affects sensors (loop closes)

**Key Interfaces**:
- ROS 2 topics for sensor data streaming (perception → processing)
- ROS 2 topics for perception results (processing → decision)
- ROS 2 action servers for trajectory execution (decision → actuation)
- Hardware interfaces for motor control (actuation → physical robot)
```

**Validation**:
- [ ] At least 2 DiagramDescriptions across module (FR-004)
- [ ] Each description includes: components, relationships, data flow
- [ ] Reader can visualize architecture from text alone

---

### Step 4: Add Iteration Placeholders

**Objective**: Create "Advanced Topics" section with deferred content.

**Actions**:
1. Create file: `docs/{module-directory}/{last-order}-advanced-topics.md`
2. List 2-4 advanced topics deferred to Iteration 3
3. For each topic:
   - **Preview**: 1-2 sentences on what will be covered
   - **Why Deferred**: Brief rationale (complexity, hands-on requirements, etc.)
4. Use template from `contracts/iteration-placeholder-template.md`

**Example**:
```markdown
# Advanced Topics (Iteration 3)

The following topics will be covered in hands-on tutorials in Iteration 3:

## Deep Reinforcement Learning for Humanoid Control

**Preview**: Learn to train humanoid locomotion and manipulation policies using deep RL algorithms (PPO, SAC) in Isaac Sim. Includes reward engineering, sim-to-real transfer, and policy deployment.

**Why Deferred**: Requires Isaac Sim environment setup, GPU infrastructure, and extensive hands-on practice beyond the scope of conceptual introduction.

---

## Production-Grade Sensor Fusion

**Preview**: Implement robust multi-sensor fusion (camera + LiDAR + IMU) using Extended Kalman Filters and sensor calibration techniques. Build production-ready SLAM pipelines.

**Why Deferred**: Requires mathematical depth (Bayesian estimation) and hands-on sensor calibration best learned through implementation projects.

---

## Real-Hardware Deployment

**Preview**: Deploy AI models on physical humanoid robots, handling real-world challenges like latency, sensor noise, and safety constraints.

**Why Deferred**: Requires access to physical hardware and builds on all prior modules.
```

**Validation**:
- [ ] "Advanced Topics" section created (FR-005)
- [ ] At least 1 placeholder (SC-005)
- [ ] Each placeholder has preview + rationale
- [ ] Clear indication topics are out of scope for Iteration 2

---

### Step 5: Validate Against Spec

**Objective**: Ensure module satisfies all FR and SC requirements.

**Actions**:
1. Open `specs/002-module-content-expansion/spec.md`
2. Review FR requirements for this module:
   - General FRs: FR-001 through FR-006
   - Module-specific FRs: FR-007-010 (Module 1), FR-011-015 (Module 2), etc.
   - Content quality FRs: FR-026 through FR-030
3. Use the **Per-Module Validation Checklist** from `plan.md`
4. Check all items systematically

**Validation Checklist** (copy from plan.md):
- [ ] FR-001: 3-5 learning objectives at module start
- [ ] FR-002: 3-5 major sections
- [ ] FR-003: ≥3 code snippets (marked non-production)
- [ ] FR-004: ≥2 diagram descriptions
- [ ] FR-005: Advanced topics placeholder
- [ ] FR-006: Prerequisites declared
- [ ] Module-specific FRs (e.g., FR-007-010 for Module 1)
- [ ] FR-026: Appropriate for CS/robotics audience
- [ ] FR-027: All content original
- [ ] FR-028: Code snippets have explanatory comments
- [ ] FR-029: Consistent structure
- [ ] FR-030: Docusaurus-compatible Markdown

**If any item fails**: Revise content before proceeding.

---

### Step 6: Docusaurus Build Test

**Objective**: Verify module renders correctly in Docusaurus.

**Actions**:
1. Run Docusaurus build command:
   ```bash
   npm run build
   ```
2. Check for Markdown syntax errors
3. If errors occur:
   - Fix formatting issues (headings, code blocks, links)
   - Re-run build
4. Verify navigation works (module appears in sidebar)

**Common Issues**:
- Unclosed code blocks (missing closing ```)
- Invalid Markdown heading levels
- Broken internal links
- Special characters not escaped

**Validation**:
- [ ] Build completes without errors (SC-009)
- [ ] Module renders correctly in browser
- [ ] Navigation functional

**Build Success**: Green light to proceed to human review.

---

### Step 7: Human Review

**Objective**: Quality gate before marking task complete.

**Actions**:
1. **Technical Accuracy Review**:
   - Verify all claims match research findings
   - Cross-check code examples for correctness
   - Confirm diagram descriptions are architecturally sound

2. **Clarity and Readability Review**:
   - Read module end-to-end (20-40 minutes)
   - Check flow and logical progression
   - Ensure explanations are clear for target audience

3. **Consistency Check**:
   - Compare structure to other completed modules
   - Verify terminology consistency across modules
   - Check that format matches templates

4. **Acceptance Scenario Verification**:
   - Review acceptance scenarios from spec.md for this module
   - Confirm content enables readers to satisfy scenarios

**Review Outcomes**:
- **Pass**: Mark task as complete
- **Revisions Needed**: Document feedback, revise (Steps 3-5), re-review

**Validation**:
- [ ] Technical accuracy verified
- [ ] Clarity and flow acceptable
- [ ] Consistent with other modules
- [ ] Acceptance scenarios satisfied

**ONLY mark task complete after human review passes.**

---

## Module Writing Checklist Summary

Use this as a quick reference:

### Pre-Writing
- [ ] Research complete for this module
- [ ] Data model and templates reviewed
- [ ] Module directory created

### Writing
- [ ] Learning objectives drafted (3-5)
- [ ] Concept sections written (3-5)
- [ ] Code snippets embedded (≥3)
- [ ] Diagram descriptions embedded (≥2)
- [ ] Advanced topics placeholders added (≥1)

### Validation
- [ ] All FR requirements satisfied
- [ ] Docusaurus build succeeds
- [ ] Human review passed

### Completion
- [ ] Task marked complete in tasks.md
- [ ] Module ready for reader consumption

---

## Tips for Efficient Writing

### Research Integration
- Keep `research.md` open while writing
- Copy verified definitions and citations directly
- Don't re-verify what's already researched

### Template Usage
- Start from templates (`contracts/`)
- Customize for specific content, don't fight the structure
- Templates ensure consistency across modules

### Batch Similar Work
- Write all learning objectives first (across sections)
- Write all ConceptSections, then embed code/diagrams
- Reduces context switching

### Incremental Validation
- Validate after each ConceptSection (don't wait until end)
- Run Docusaurus build frequently (catch errors early)
- Self-review as you go

### Time Management
- Module 1: 6-8 hours (establishing patterns)
- Modules 2-4: 4-6 hours (templates established)
- Human review: 1-2 hours per module

---

## Troubleshooting

### Issue: Research incomplete
**Solution**: STOP writing. Complete research tasks first. Writing without verified sources risks inaccuracy.

### Issue: Code snippet syntax errors
**Solution**: Use Python linter (e.g., `python -m py_compile snippet.py`). Fix errors before embedding.

### Issue: Docusaurus build fails
**Solution**: Check error message for line number. Common fixes:
- Close all code blocks with ```
- Fix heading levels (# then ##, not # then ###)
- Escape special characters in Markdown

### Issue: Module too long (>40 min reading)
**Solution**: Move advanced content to placeholders. Focus on core concepts only.

### Issue: Module too short (<20 min reading)
**Solution**: Expand explanations, add more examples, or split dense sections.

---

## Success Criteria

A module is **complete** when:

1. ✅ All 7 workflow steps passed
2. ✅ Human review approved
3. ✅ Task marked complete in `tasks.md`
4. ✅ Reader can achieve all learning objectives from content

**Do NOT mark complete** until human review passes. Quality over speed.

---

## Next Steps After Module Completion

1. Update `tasks.md` to mark module task complete
2. Run `.specify/scripts/bash/update-agent-context.sh claude` (updates CLAUDE.md)
3. Proceed to next module (in dependency order: Foundations → ROS 2 → Digital Twin → Isaac)
4. Celebrate progress! 🎉

---

## Questions or Issues?

- Review `data-model.md` for entity definitions
- Check `plan.md` for architectural decisions
- Consult `contracts/` templates for format examples
- Refer to `spec.md` for FR/SC requirements

**Remember**: Technical accuracy and human review are non-negotiable. Never skip these steps.
