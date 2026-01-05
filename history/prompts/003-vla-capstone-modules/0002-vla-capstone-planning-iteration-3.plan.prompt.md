---
id: 0002
title: VLA Capstone Planning Iteration 3
stage: plan
date: 2026-01-04
surface: agent
model: claude-opus-4-5-20251101
feature: 003-vla-capstone-modules
branch: 003-vla-capstone-modules
user: abd_dev9
command: /sp.plan
labels: ["vla", "capstone", "iteration-3", "planning", "textbook"]
links:
  spec: specs/003-vla-capstone-modules/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/003-vla-capstone-modules/plan.md
  - specs/003-vla-capstone-modules/research.md
  - specs/003-vla-capstone-modules/data-model.md
  - specs/003-vla-capstone-modules/quickstart.md
tests:
  - null
---

## Prompt

Iteration 3 – Conceptual Completion
Part A: Primary Execution Plan

Objective:
Complete the textbook conceptually by adding the missing intelligence layer (VLA) and an end-to-end Capstone that connects all prior modules into a single coherent story.

Iteration Scope:

Add Vision-Language-Action (VLA) module

Add Capstone module (full system synthesis)

No frontend polish, no deployment, no auth

Phase Breakdown
Phase 0: Context & Validation

Confirm Iteration 2 modules are complete and consistent

Verify conceptual gaps:

Intelligence layer missing (VLA)

No single end-to-end system narrative (Capstone)

Deliverable:

Iteration 3 scope confirmation (this plan)

Phase 1: Architecture Definition

Define conceptual architecture for:

VLA Module

Vision → Language → Action pipeline

Role of foundation models (VLMs)

World models and planning

How VLA interfaces with ROS 2 and Isaac

Why VLA is the "brain" layer

Capstone Module

Full humanoid AI stack: Sensors → Perception → VLA → Planning → Control → Actuators

Data flow across all modules

Sim-to-real loop

Failure handling and feedback loops

Deliverable:

Clear architecture narratives (no code dependency)

Phase 2: Content Design

Module 05: VLA (Vision-Language-Action)

index.md (objectives, prerequisites)

3–5 concept sections:

Multimodal perception

Language grounding

Action planning

Policy learning vs reasoning

2+ diagram descriptions

Minimal illustrative pseudocode

Iteration 4 placeholders (training, datasets, scaling)

Module 06: Capstone

index.md (system overview)

3–4 concept sections:

End-to-end humanoid pipeline

System integration narrative

Example task walkthrough ("Pick & place in real world")

Design trade-offs and constraints

2+ diagram descriptions

No code required

Clear mapping to all previous modules

Deliverable:

Fully written Markdown modules

Phase 3: Validation & Quality Gates

Validation Checklist:

Consistent terminology with Iteration 1–2

No unexplained jumps in abstraction

Reader can explain full system verbally

No dependency on code to understand architecture

Checks:

FR alignment

Diagram clarity

Conceptual completeness

Human review for correctness

Phase 4: Integration Readiness

Ensure:

Sidebar ordering updated

Cross-links between modules added

Intro page reflects full textbook scope

Explicitly Deferred:

Frontend animations

UI polish

Dark/light toggle

Auth

Urdu localization

Deployment

Completion Definition (DoD)

Iteration 3 is complete when:

VLA module is written and validated

Capstone module is written and validated

End-to-end humanoid AI story is clear

Textbook feels complete conceptually

No deployment performed

Ready for:

Iteration 4 (Frontend, UX, Auth, Hackathon features)

Part B: Design Rationale & Writing Strategy

Create:

Conceptual architecture for VLA and Capstone

Section structure for both modules

Research-concurrent writing approach

Validation checklist for conceptual completeness

Architecture Sketch

VLA: Vision → Language → Action reasoning loop

Capstone: End-to-end humanoid AI stack (Sensors → Perception → VLA → Planning → Control → Actuation)

Sections

VLA:

Multimodal perception

Language grounding

Action planning & policies

Integration with ROS 2 / Isaac

Capstone:

Full system overview

Data & control flow

Example task walkthrough

Design tradeoffs

Research Approach

Research while writing (no upfront research phase)

Use authoritative sources (papers, official docs)

APA citation style

Decisions to Document

Why VLA is the intelligence layer

Why Capstone is narrative, not code

Why training & scaling deferred

Why frontend/auth excluded

Testing / Validation

Reader can explain full humanoid AI pipeline without code

Architecture understandable end-to-end

Modules align with Iteration 1–2 concepts

No missing conceptual links

Phases

Research → Concept Writing → Synthesis → Validation

Not Included

Frontend / UI / animations

Auth or localization

Deployment or production code

## Response snapshot

Created complete implementation plan for VLA & Capstone modules (Iteration 3). Key artifacts generated:

1. **plan.md**: Full implementation plan with:
   - Technical context (Markdown/Docusaurus, no code)
   - Constitution check (all gates passed)
   - 4-phase implementation breakdown
   - Content architecture for both modules
   - Success criteria and risk mitigation

2. **research.md**: Consolidated VLA research including:
   - VLA fundamentals (perception → language → action)
   - Multimodal perception (vision encoders, fusion)
   - Language grounding definitions
   - Action representation strategies
   - Architecture comparisons (end-to-end vs modular)
   - ROS 2 integration patterns
   - System-level concerns

3. **data-model.md**: Detailed content architecture:
   - Module 5: 7 files (index + 6 sections)
   - Module 6: 6 files (index + 5 sections)
   - Learning objectives for each section
   - Diagram specifications
   - Cross-reference requirements

4. **quickstart.md**: Writing guidelines:
   - Style and terminology consistency
   - Section template
   - Diagram format
   - Quality checklist

Architecture decision surfaced: VLA positioned as distinct intelligence layer. Suggested documenting via `/sp.adr vla-intelligence-layer`.

## Outcome

- ✅ Impact: Complete planning artifacts for Iteration 3 conceptual completion
- 🧪 Tests: N/A (planning phase)
- 📁 Files: 4 files created (plan.md, research.md, data-model.md, quickstart.md)
- 🔁 Next prompts: /sp.tasks to generate task breakdown, then content writing
- 🧠 Reflection: Research-concurrent approach enables faster iteration while maintaining accuracy

## Evaluation notes (flywheel)

- Failure modes observed: None - all phases completed successfully
- Graders run and results (PASS/FAIL): Constitution check PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Validate content architecture by writing one section
