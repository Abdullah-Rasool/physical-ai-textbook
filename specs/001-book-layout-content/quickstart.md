# Quickstart: Book Layout & High-Level Content

**Branch**: `001-book-layout-content` | **Date**: 2025-12-28

## Prerequisites

- Node.js 18+ installed (`node --version`)
- npm 9+ installed (`npm --version`)
- Git installed and configured
- GitHub account with repository access

## Local Development Setup

### 1. Clone and Navigate

```bash
git clone https://github.com/[username]/physical-ai-textbook.git
cd physical-ai-textbook
git checkout 001-book-layout-content
```

### 2. Initialize Docusaurus Project

```bash
# From repository root
npx create-docusaurus@latest textbook classic --typescript
cd textbook
```

### 3. Install Dependencies

```bash
npm install
```

### 4. Start Development Server

```bash
npm run start
```

The site will be available at `http://localhost:3000`.

## Project Structure After Setup

```
physical-ai-textbook/
├── .specify/                  # Spec-Kit Plus configuration
├── specs/                     # Feature specifications
│   └── 001-book-layout-content/
├── history/                   # Prompt history records
└── textbook/                  # Docusaurus project
    ├── docs/                  # Content (Markdown/MDX)
    ├── src/                   # React components
    ├── static/                # Static assets
    ├── docusaurus.config.ts   # Site configuration
    ├── sidebars.ts            # Navigation structure
    └── package.json           # Dependencies
```

## Adding Module Content

### Create Module Directory

```bash
cd textbook/docs
mkdir foundations ros2 digital-twin isaac vla capstone
```

### Create Module Landing Page

Each module needs an `index.md`:

```bash
touch foundations/index.md
touch ros2/index.md
touch digital-twin/index.md
touch isaac/index.md
touch vla/index.md
touch capstone/index.md
```

### Module Template

Use this template for each `index.md`:

```markdown
---
sidebar_position: N
---

# [Module Title]

[Introduction paragraph explaining what this module covers and why it matters]

## Key Concepts

- **Concept 1**: Brief explanation
- **Concept 2**: Brief explanation
- **Concept 3**: Brief explanation

## Role in the Humanoid AI System

[Paragraph explaining how this module connects to others]

## What You'll Learn

- Learning outcome 1
- Learning outcome 2
- Learning outcome 3

:::info Coming in Iteration 2
Detailed chapters, code examples, and hands-on tutorials will be added
in future iterations. Topics to be covered include:
- Topic 1
- Topic 2
- Topic 3
:::
```

## Building for Production

```bash
cd textbook
npm run build
```

Build output is in `textbook/build/`.

## Deployment to GitHub Pages

### Option 1: GitHub Actions (Recommended)

Create `.github/workflows/deploy.yml`:

```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]

permissions:
  contents: read
  pages: write
  id-token: write

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4
        with:
          node-version: 18
          cache: npm
          cache-dependency-path: textbook/package-lock.json
      - run: npm ci
        working-directory: textbook
      - run: npm run build
        working-directory: textbook
      - uses: actions/upload-pages-artifact@v3
        with:
          path: textbook/build

  deploy:
    needs: build
    runs-on: ubuntu-latest
    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}
    steps:
      - uses: actions/deploy-pages@v4
        id: deployment
```

### Option 2: Manual Deploy

```bash
cd textbook
npm run build
npx gh-pages -d build
```

## Validation Checklist

After setup, verify:

- [ ] `npm run start` launches dev server
- [ ] All 6 modules visible in sidebar
- [ ] Each module page loads without errors
- [ ] `npm run build` completes without errors
- [ ] No broken links reported during build

## Common Issues

### "Cannot find module" errors

```bash
rm -rf node_modules package-lock.json
npm install
```

### Port 3000 already in use

```bash
npm run start -- --port 3001
```

### Build fails with MDX errors

Check for:
- Unclosed JSX tags
- Invalid frontmatter YAML
- Broken internal links

## Next Steps

1. Create content for each module following the template
2. Configure sidebar navigation in `sidebars.ts`
3. Update `docusaurus.config.ts` with project metadata
4. Test local build
5. Deploy to GitHub Pages
6. Validate deployment
