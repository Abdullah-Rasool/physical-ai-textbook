# Writing Guidelines: VLA & Capstone Modules

**Feature**: `003-vla-capstone-modules` | **Date**: 2026-01-04 | **Plan**: [plan.md](./plan.md)

---

## Quick Reference

This document provides writing guidelines for content authors implementing Module 5 (VLA) and Module 6 (Capstone).

---

## Style Guidelines

### Voice and Tone

- **Direct and clear**: Write in second person ("you will learn", "you can see")
- **Technical but accessible**: Assume CS/engineering background, define domain-specific terms
- **Educational**: Focus on understanding, not implementation details
- **Neutral**: Avoid marketing language or hype about specific technologies

### Terminology Consistency

Use these exact terms consistently (from Modules 1-4):

| Term | Definition | Avoid |
|------|------------|-------|
| Perception-action loop | Continuous cycle of sensing and acting | "Sense-act cycle", "observe-act" |
| ROS 2 | Robot Operating System 2 (middleware) | "ROS" alone (ambiguous with ROS 1) |
| Digital twin | Virtual replica of physical system | "Simulation" alone (less specific) |
| Node | Independent ROS 2 process | "Component", "module" (overloaded) |
| Topic | Named pub/sub channel | "Stream", "channel" |
| VLA | Vision-Language-Action model | "Multimodal robot model" |
| VLM | Vision-Language Model (perception only) | Avoid conflating with VLA |

### New Terms for Modules 5-6

| Term | Definition | First Use |
|------|------------|-----------|
| Language grounding | Connecting language to physical state | 03-language-grounding.md |
| Action tokenization | Converting continuous actions to discrete tokens | 04-action-generation.md |
| Action chunking | Predicting sequences of actions | 04-action-generation.md |
| End-to-end VLA | Single model from pixels to actions | 05-architectures.md |
| Modular VLA | Separate components with interfaces | 05-architectures.md |

---

## Section Template

Each section file should follow this structure:

```markdown
# Section Title

**Estimated Reading Time**: X minutes

---

## Learning Objectives

After completing this section, you will be able to:

1. **Verb** specific outcome 1
2. **Verb** specific outcome 2
3. **Verb** specific outcome 3

---

## [Main Content Heading]

[Content paragraphs]

### [Subheading]

[Content paragraphs]

### Key Concept: [Name]

[Definition and explanation]

---

## [Architecture Diagram Title]

[Text-described diagram using ASCII/box drawing]

**Diagram Explanation**: [2-3 sentences explaining what the diagram shows]

---

## [Optional: Illustrative Code]

```python
# Purpose: [What this code demonstrates]
# Note: Illustrative only, not production code

[Pseudocode or minimal example]
```

---

## Connection to Previous Modules

- **Module X**: [How this section relates]
- **Module Y**: [How this section builds on prior concepts]

---

## Key Takeaways

- Point 1
- Point 2
- Point 3

---

**Next**: [Link to next section]
```

---

## Diagram Guidelines

### Format

Use ASCII box drawing for all diagrams. Supported characters:

```
Boxes:     ┌ ─ ┐ │ └ ┘ ├ ┤ ┬ ┴ ┼
Arrows:    → ← ↑ ↓ ▶ ◀ ▲ ▼
Other:     │ ─ • ◦ ○ ●
```

### Diagram Types

1. **Pipeline diagrams**: Show data flow left-to-right or top-to-bottom
2. **Architecture diagrams**: Show layers or components
3. **Comparison tables**: Side-by-side with clear differences
4. **Process flows**: Numbered steps with arrows

### Example

```
┌─────────────────────────────────────────────────────────────────┐
│                      Diagram Title                               │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────┐       ┌──────────┐       ┌──────────┐            │
│  │ Component│──────▶│ Component│──────▶│ Component│            │
│  │    A     │       │    B     │       │    C     │            │
│  └──────────┘       └──────────┘       └──────────┘            │
│       │                                      │                   │
│       └──────────── Label ──────────────────┘                   │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## Cross-Reference Guidelines

### Internal Links (within same module)

```markdown
[action generation](./04-action-generation.md)
```

### Cross-Module Links

```markdown
[ROS 2 actions](../02-ros2/03-distributed-control.md)
```

### Required Cross-References per Section

- Minimum 3 cross-references to prior module concepts
- At least 1 reference to Module 1 foundations where applicable
- Explicit "Connection to Previous Modules" section

---

## Pseudocode Guidelines

### Purpose

- Illustrate concepts, NOT provide working code
- Show patterns and structures
- Bridge from concept to implementation idea

### Style

```python
# Purpose: [Clear statement of what this demonstrates]
# Context: [When this would be used]
# Note: Illustrative pseudocode, not production code

def concept_example(input_param):
    """Brief docstring explaining the concept."""
    # Step 1: [What this does]
    intermediate = process(input_param)

    # Step 2: [What this does]
    result = transform(intermediate)

    return result
```

### Rules

- Include clear purpose comment
- Use Python-like syntax (familiar to audience)
- Add inline comments explaining each step
- Keep under 20 lines
- No dependencies or imports needed to understand

---

## Word Count Targets

| Section Type | Target Words | Min | Max |
|--------------|--------------|-----|-----|
| index.md | 800-1000 | 600 | 1200 |
| Content section | 1000-1200 | 800 | 1500 |
| Advanced topics | 800-1000 | 600 | 1200 |

### Reading Time Calculation

- Assume 200 words per minute
- Add 1 minute per diagram
- Add 30 seconds per code block

---

## Quality Checklist

Before submitting a section, verify:

### Content
- [ ] Learning objectives are specific and measurable
- [ ] Key terms are defined on first use
- [ ] Concepts build on prior knowledge
- [ ] No unexplained jargon or acronyms

### Structure
- [ ] Follows section template
- [ ] Includes at least one diagram
- [ ] Has "Connection to Previous Modules" section
- [ ] Has "Key Takeaways" summary

### Style
- [ ] Uses consistent terminology
- [ ] Second person voice
- [ ] Technical but accessible
- [ ] No marketing language

### Links
- [ ] All internal links are valid
- [ ] Cross-references to prior modules included
- [ ] Next section link at bottom

### Technical
- [ ] Diagrams render correctly in markdown
- [ ] Pseudocode is syntactically reasonable
- [ ] No placeholder text remains

---

## File Naming Convention

```
[module-number]-[module-name]/[section-number]-[section-slug].md
```

Examples:
- `05-vla/index.md`
- `05-vla/01-vla-fundamentals.md`
- `06-capstone/03-reference-architecture.md`

---

## Revision Workflow

1. Write section following template
2. Self-review against checklist
3. Verify links work
4. Test diagram rendering
5. Check word count
6. Submit for human review

---

**Writing Guidelines Status**: Complete. Ready for content authoring.
