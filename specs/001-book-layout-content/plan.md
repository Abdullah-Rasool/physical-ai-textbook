# Implementation Plan: Book Layout & High-Level Content

**Branch**: `001-book-layout-content` | **Date**: 2025-12-28 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-book-layout-content/spec.md`

## Summary

Establish the complete Docusaurus-based textbook structure for Physical AI & Humanoid Robotics with 6 navigable modules (Foundations + 4 core modules + Capstone). Each module contains high-level introductions, key concepts, and placeholder notices for future detailed content. The book builds and deploys to GitHub Pages as the Iteration 1 deliverable.

## Technical Context

**Language/Version**: JavaScript/TypeScript (Node.js 18+) for Docusaurus build tooling; Markdown for content
**Primary Dependencies**: Docusaurus 3.x, React 18.x (bundled with Docusaurus)
**Storage**: N/A (static site, no database)
**Testing**: Manual validation (navigation, build success, link checking)
**Target Platform**: Web browsers (modern Chrome, Firefox, Safari, Edge); GitHub Pages hosting
**Project Type**: Static documentation site (Docusaurus docs template)
**Performance Goals**: Page load <3 seconds on broadband; all content static and cacheable
**Constraints**: Hackathon Phase 1 timeline; high-level content only; no deep tutorials
**Scale/Scope**: 6 modules, ~15-20 total pages in Iteration 1

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Evidence |
|-----------|--------|----------|
| I. Technical Accuracy | ✅ PASS | High-level conceptual content only; marked for human review before publication |
| II. Clarity for Technical Audience | ✅ PASS | Target audience: CS/engineering students with foundational knowledge |
| III. Spec-Driven Development | ✅ PASS | Feature spec approved; using Spec-Kit Plus workflow |
| IV. Reproducibility | ✅ PASS | Docusaurus + npm dependencies; GitHub Pages deployment documented |

**Content Standards Check:**
- Originality: ✅ All content will be original
- Human Review: ✅ Required before deployment
- Structured Writing: ✅ Template-based module structure
- Verifiability: ✅ High-level concepts; detailed technical claims deferred

**AI Usage Compliance:**
- AI as reasoning assistant: ✅ Claude Code for planning and scaffolding
- Human review required: ✅ All AI-generated content marked for review
- Not sole verification source: ✅ Human review gate before deployment

## Project Structure

### Documentation (this feature)

```text
specs/001-book-layout-content/
├── spec.md              # Feature specification (complete)
├── plan.md              # This file
├── research.md          # Phase 0: Technology decisions
├── content-model.md     # Phase 1: Book structure model
├── quickstart.md        # Phase 1: Setup and deployment guide
└── checklists/
    └── requirements.md  # Specification quality checklist
```

### Source Code (repository root)

```text
textbook/                    # Docusaurus project root
├── docusaurus.config.js     # Site configuration
├── sidebars.js              # Navigation structure
├── package.json             # Dependencies
├── docs/                    # Content directory
│   ├── intro.md             # Homepage/landing
│   ├── foundations/         # Module: Foundations of Physical AI
│   │   └── index.md
│   ├── ros2/                # Module 1: ROS 2
│   │   └── index.md
│   ├── digital-twin/        # Module 2: Digital Twin
│   │   └── index.md
│   ├── isaac/               # Module 3: NVIDIA Isaac
│   │   └── index.md
│   ├── vla/                 # Module 4: VLA
│   │   └── index.md
│   └── capstone/            # Capstone module
│       └── index.md
├── static/                  # Static assets (images, diagrams)
└── src/                     # Custom components (if needed)
    └── css/
        └── custom.css       # Theme customizations
```

**Structure Decision**: Docusaurus documentation template with flat module structure. Each module is a top-level directory under `docs/` containing an `index.md` landing page. This enables clean URLs (`/foundations`, `/ros2`, etc.) and straightforward navigation.

## Architecture Decisions

### Decision 1: Docusaurus Docs Template (Not Blog)

**Context**: Docusaurus supports multiple site types (docs, blog, custom pages).

**Decision**: Use the "docs" template as the primary site structure.

**Rationale**:
- Docs template provides built-in sidebar navigation essential for textbook UX
- Supports hierarchical content organization (modules → chapters → sections)
- Version control integration aligns with spec-driven workflow
- Blog template inappropriate for structured educational content

**Alternatives Rejected**:
- Blog template: Linear/chronological, doesn't support hierarchical navigation
- Custom pages only: Requires manual navigation implementation

### Decision 2: Flat Module Structure (Iteration 1)

**Context**: Content hierarchy could be deep (module → chapter → section → subsection) or shallow.

**Decision**: Flat module structure with single `index.md` per module for Iteration 1.

**Rationale**:
- Matches Iteration 1 scope (high-level content only)
- Simpler to scaffold and validate
- Easy to expand to deeper hierarchy in Iteration 2
- Avoids premature structuring before content exists

**Alternatives Rejected**:
- Deep hierarchy now: Creates empty placeholder files, harder to validate
- Single page per module: Doesn't demonstrate navigation capability

### Decision 3: GitHub Actions for Deployment

**Context**: GitHub Pages can be deployed via Actions or manual push to `gh-pages` branch.

**Decision**: Use GitHub Actions workflow for automated deployment on push to main.

**Rationale**:
- Reproducible deployments (constitution requirement)
- Enables CI checks before deploy (build success, link validation)
- Standard pattern for Docusaurus + GitHub Pages
- No manual deployment steps

**Alternatives Rejected**:
- Manual deploy: Violates reproducibility principle
- Third-party CI: Unnecessary complexity for hackathon scope

### Decision 4: Content-First Placeholders

**Context**: Iteration 1 includes "placeholder notices" for future content.

**Decision**: Use a standardized admonition block format for placeholders.

**Rationale**:
- Docusaurus admonitions provide visual distinction
- Consistent format across all modules
- Easy to find/replace when adding real content in Iteration 2
- Clearly communicates "coming soon" to readers

**Format**:
```markdown
:::info Coming in Iteration 2
Detailed technical content, code examples, and hands-on tutorials
will be added in future iterations.
:::
```

## Information Architecture

### Navigation Hierarchy

```
Physical AI Textbook (root)
├── Welcome / Introduction
├── Foundations of Physical AI
│   └── [Iteration 2: Chapters]
├── Module 1: The Robotic Nervous System (ROS 2)
│   └── [Iteration 2: Chapters]
├── Module 2: The Digital Twin (Gazebo & Unity)
│   └── [Iteration 2: Chapters]
├── Module 3: The AI–Robot Brain (NVIDIA Isaac)
│   └── [Iteration 2: Chapters]
├── Module 4: Vision–Language–Action (VLA)
│   └── [Iteration 2: Chapters]
└── Capstone: Autonomous Humanoid System Overview
    └── [Iteration 2: Integration project]
```

### Page Taxonomy

| Page Type | Purpose | Iteration 1 Scope |
|-----------|---------|-------------------|
| Landing (intro.md) | Welcome, book overview, learning path | Full content |
| Module Index | Module introduction, key concepts, connections | Full content |
| Chapter | Deep technical content on specific topic | Placeholder only |
| Lab/Tutorial | Hands-on exercises with code | Out of scope |

### Module Content Template

Each module `index.md` follows this structure:

```markdown
---
sidebar_position: N
---

# [Module Title]

[1-2 paragraph introduction: What is this? Why does it matter?]

## Key Concepts

- Concept 1: Brief explanation
- Concept 2: Brief explanation
- Concept 3: Brief explanation
- [Additional concepts as needed]

## Role in the Humanoid AI System

[1 paragraph: How this module connects to others in the pipeline]

## What You'll Learn

[Bullet list of learning outcomes for this module]

:::info Coming in Iteration 2
Detailed chapters, code examples, and hands-on tutorials will be added
in future iterations. Topics to be covered include:
- [Topic 1]
- [Topic 2]
- [Topic 3]
:::

## Further Reading

[Optional: Links to official documentation, papers, or resources]
```

## Iteration Boundaries

### Iteration 1 (This Plan)

| Deliverable | Status |
|-------------|--------|
| Docusaurus project setup | To implement |
| 6 module landing pages with introductions | To implement |
| Key concepts for each module | To implement |
| Navigation sidebar configuration | To implement |
| Placeholder notices for future content | To implement |
| GitHub Pages deployment | To implement |
| Mobile-responsive reading | Built-in with Docusaurus |

### Iteration 2 (Future)

| Deliverable | Notes |
|-------------|-------|
| Detailed chapter breakdowns | Each module expands to 3-5 chapters |
| Step-by-step tutorials | Hands-on labs with prerequisites |
| ROS 2 code examples | Actual runnable code |
| Gazebo/Isaac configuration | Environment setup guides |
| Interactive diagrams | Mermaid or custom components |
| Search enhancement | Algolia DocSearch or similar |

## Validation Strategy

### Build Validation

- [ ] `npm run build` completes without errors
- [ ] No broken internal links (Docusaurus built-in check)
- [ ] All module pages render correctly

### Content Validation

- [ ] All 6 modules present in navigation
- [ ] Each module has: introduction, key concepts, placeholder notice
- [ ] No implementation details (code, commands) in content
- [ ] Technical accuracy review by human reviewer

### Deployment Validation

- [ ] GitHub Actions workflow executes successfully
- [ ] Site accessible at GitHub Pages URL
- [ ] All pages load within 3 seconds
- [ ] Mobile layout functional

### Constitution Compliance

- [ ] Content marked for human review before publication
- [ ] All content original (no plagiarism)
- [ ] Spec-driven workflow documented in repository
- [ ] Build/deploy process reproducible from repository

## Complexity Tracking

> No constitution violations requiring justification. All decisions align with principles.

| Decision | Constitution Alignment |
|----------|----------------------|
| Docusaurus framework | Matches tooling constraint |
| GitHub Pages deployment | Matches reproducibility requirement |
| Placeholder content approach | Aligns with Hackathon Phase 1 scope |
| Human review gate | Satisfies accuracy principle |

## Risk Analysis

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Docusaurus version incompatibility | Low | Medium | Pin exact versions in package.json |
| GitHub Pages deployment failure | Low | High | Test deployment early; document rollback |
| Content exceeds high-level scope | Medium | Low | Clear placeholder template; review gate |
| Navigation complexity in Iteration 2 | Medium | Medium | Design extensible sidebar structure now |

## Next Steps

After `/sp.plan` completes:

1. Run `/sp.tasks` to generate implementation task list
2. Execute tasks in order (setup → content → deployment)
3. Human review of all content before deployment
4. Deploy to GitHub Pages
5. Validate against success criteria
