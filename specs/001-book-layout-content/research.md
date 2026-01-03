# Research: Book Layout & High-Level Content

**Branch**: `001-book-layout-content` | **Date**: 2025-12-28

## Research Questions

This document captures technology decisions and best practices research for Iteration 1.

### Q1: Docusaurus Version Selection

**Decision**: Docusaurus 3.x (latest stable)

**Rationale**:
- Docusaurus 3.x is the current stable release with active maintenance
- Built on React 18 with improved performance
- Native TypeScript support for configuration
- Enhanced MDX 3 support for content authoring
- GitHub Pages deployment well-documented

**Alternatives Considered**:
- Docusaurus 2.x: Deprecated; no security updates
- Other static site generators (Hugo, Jekyll, MkDocs): Less React ecosystem integration; fewer documentation-focused features

**Sources**:
- Docusaurus official documentation: https://docusaurus.io/docs
- Docusaurus 3.0 release notes

### Q2: GitHub Pages Deployment Strategy

**Decision**: GitHub Actions with `peaceiris/actions-gh-pages`

**Rationale**:
- Standard community action for Docusaurus → GitHub Pages
- Handles build + deploy in single workflow
- Supports custom domains if needed later
- Well-maintained with regular updates

**Workflow Structure**:
```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4
        with:
          node-version: 18
      - run: npm ci
        working-directory: textbook
      - run: npm run build
        working-directory: textbook
      - uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./textbook/build
```

**Alternatives Considered**:
- Manual gh-pages branch push: Violates reproducibility
- Vercel/Netlify: Adds external dependency; GitHub Pages sufficient for hackathon

### Q3: Content Organization Pattern

**Decision**: Flat module structure with category directories

**Rationale**:
- Each module is a directory under `docs/`
- `index.md` serves as module landing page
- Sidebar auto-generation uses directory structure
- Easy to expand to chapters by adding files to each directory

**Structure**:
```
docs/
├── intro.md           (sidebar_position: 1)
├── foundations/
│   └── index.md       (sidebar_position: 2)
├── ros2/
│   └── index.md       (sidebar_position: 3)
├── digital-twin/
│   └── index.md       (sidebar_position: 4)
├── isaac/
│   └── index.md       (sidebar_position: 5)
├── vla/
│   └── index.md       (sidebar_position: 6)
└── capstone/
    └── index.md       (sidebar_position: 7)
```

**Alternatives Considered**:
- Single docs folder with all files: Poor organization for future expansion
- Deep nesting now: Premature optimization; creates empty placeholders

### Q4: Docusaurus Configuration Approach

**Decision**: Minimal configuration; rely on defaults

**Rationale**:
- Docusaurus defaults are well-suited for documentation sites
- Minimal config reduces maintenance burden
- Custom theming deferred to Iteration 2 if needed
- Focus on content over presentation for hackathon

**Key Configuration Points**:
- `title`: "Physical AI Textbook"
- `url`: GitHub Pages URL
- `baseUrl`: Repository-based path
- `docs.routeBasePath`: "/" (docs as homepage)
- `sidebar`: Auto-generated from file structure

### Q5: Node.js and npm Version Requirements

**Decision**: Node.js 18.x LTS, npm 9+

**Rationale**:
- Node 18 LTS is current long-term support version
- Docusaurus 3.x requires Node 18+
- npm 9+ bundled with Node 18
- Consistent with GitHub Actions ubuntu-latest

**Version Pinning**:
```json
{
  "engines": {
    "node": ">=18.0.0"
  }
}
```

## Technical Constraints Resolved

| Constraint | Resolution |
|------------|------------|
| Docusaurus version | 3.x (latest stable) |
| Node.js version | 18.x LTS |
| Deployment target | GitHub Pages via Actions |
| Content format | MDX (Markdown + JSX) |
| Sidebar navigation | Auto-generated from file structure |
| URL structure | Clean URLs based on directory names |

## Dependencies List

### Production Dependencies

```json
{
  "@docusaurus/core": "^3.0.0",
  "@docusaurus/preset-classic": "^3.0.0",
  "prism-react-renderer": "^2.0.0",
  "react": "^18.0.0",
  "react-dom": "^18.0.0"
}
```

### Development Dependencies

```json
{
  "@docusaurus/module-type-aliases": "^3.0.0",
  "@docusaurus/types": "^3.0.0"
}
```

## Open Questions for Iteration 2

These items are intentionally deferred:

1. **Search Integration**: Algolia DocSearch vs local search
2. **Custom Theming**: Brand colors, typography, custom components
3. **Code Execution**: Live code blocks with runnable examples
4. **Diagrams**: Mermaid integration for architecture diagrams
5. **Versioning**: Multi-version documentation for course updates
