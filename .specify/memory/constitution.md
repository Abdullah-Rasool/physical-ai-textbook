<!--
  SYNC IMPACT REPORT
  ==================
  Version change: N/A (template) → 1.0.0 (initial release)

  Modified principles: N/A (first constitution)

  Added sections:
  - Core Principles (4): Technical Accuracy, Clarity, Spec-Driven Development, Reproducibility
  - Key Standards: Content standards and AI usage guidelines
  - Project Constraints: Scope, tooling, and timeline constraints
  - Governance: Amendment procedure, versioning policy, compliance review

  Removed sections:
  - Principle 5 and 6 (user specified 4 principles only)

  Templates requiring updates:
  - ✅ .specify/templates/plan-template.md (no changes required - uses generic "Constitution Check")
  - ✅ .specify/templates/spec-template.md (no changes required - technology-agnostic)
  - ✅ .specify/templates/tasks-template.md (no changes required - generic structure)

  Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Technical Accuracy

All technical content MUST be conceptually accurate and verified:
- Every explanation MUST be technically correct and conceptually verifiable
- Claims MUST be supported by established principles or cited sources
- No speculative or unverified technical content is permitted
- Content MUST undergo human review before publication

**Rationale**: Educational material on Physical AI and Humanoid Robotics must maintain rigorous accuracy to build correct mental models in learners. Inaccurate content undermines the textbook's credibility and harms student learning.

### II. Clarity for Technical Audience

Content MUST be written for readers with computer science or engineering backgrounds:
- Assume familiarity with programming, mathematics, and engineering fundamentals
- Use precise technical terminology with definitions when introducing domain-specific concepts
- Structure content with clear headings, progressive complexity, and logical flow
- Prioritize explanation clarity over academic formality

**Rationale**: The target audience consists of technical professionals and students. Content must respect their existing knowledge while effectively communicating new Physical AI concepts.

### III. Spec-Driven Development

All development decisions MUST be guided by written specifications:
- Feature work begins only after specification is approved
- Specifications are managed using Spec-Kit Plus workflow
- Changes to specifications require explicit version updates
- Implementation MUST trace back to specification requirements

**Rationale**: Spec-driven development ensures traceability, reduces scope creep, and enables effective collaboration during the hackathon timeline.

### IV. Reproducibility

The project MUST be buildable and deployable from the repository alone:
- All dependencies MUST be explicitly declared
- Build and deployment instructions MUST be documented in the repository
- No undocumented manual steps are permitted
- The deployed textbook MUST be accessible via GitHub Pages

**Rationale**: Reproducibility ensures project sustainability and enables others to build, modify, and deploy the textbook independently.

## Key Standards

### Content Standards

- **Originality**: All content MUST be original or properly attributed. Zero tolerance for plagiarism.
- **Human Review**: AI-generated content MUST be reviewed and verified by humans before inclusion.
- **Structured Writing**: Content MUST follow clear, structured educational writing patterns.
- **Verifiability**: All technical claims MUST be conceptually verifiable through first principles or citations.

### AI Usage Guidelines

- AI is used as a **reasoning assistant**, not a copy-paste generator
- AI outputs MUST be critically evaluated and refined by humans
- AI MUST NOT be the sole source of technical accuracy verification
- All AI-assisted content MUST be marked for human review

## Project Constraints

### Scope

- Limited to **Hackathon 1 – Phase 1** deliverables only
- Deliverable is a **web-based interactive textbook**, not a research paper
- Focus on foundational Physical AI and Humanoid Robotics concepts

### Tooling

- **Documentation Framework**: Docusaurus
- **Hosting**: GitHub Pages
- **Specification Management**: Spec-Kit Plus
- **Version Control**: Git with GitHub

### Timeline

- Time-bound to hackathon timeline
- Prioritize MVP (Minimum Viable Product) over feature completeness
- Defer non-essential features to post-hackathon phases

## Governance

### Amendment Procedure

1. Proposed amendments MUST be documented with rationale
2. Amendments require explicit approval from project maintainers
3. Each amendment MUST include a migration plan for affected artifacts
4. Version number MUST be updated according to semantic versioning

### Versioning Policy

This constitution follows semantic versioning (MAJOR.MINOR.PATCH):
- **MAJOR**: Backward-incompatible principle removals or redefinitions
- **MINOR**: New principle/section added or materially expanded guidance
- **PATCH**: Clarifications, wording fixes, non-semantic refinements

### Compliance Review

- All pull requests MUST verify compliance with this constitution
- The `/sp.plan` and `/sp.tasks` commands MUST check constitution alignment
- Complexity and deviations from principles MUST be explicitly justified

### Success Criteria

The project succeeds when:
1. Book is successfully deployed on GitHub Pages
2. Repository clearly demonstrates spec-driven workflow
3. All content aligns with this constitution
4. Claude Code follows all governing rules defined herein

**Version**: 1.0.0 | **Ratified**: 2025-12-28 | **Last Amended**: 2025-12-28
