<!--
Sync Impact Report:
Version change: N/A (initial version) → 1.0.0
List of modified principles: N/A (new project)
Added sections: All principles and governance sections
Removed sections: N/A
Templates requiring updates:
  - .specify/templates/plan-template.md (constitution check section) ✅ updated
  - .specify/templates/spec-template.md (scope/requirements alignment) ✅ updated
  - .specify/templates/tasks-template.md (task categorization) ✅ updated
  - .qwen/commands/sp.constitution.toml (command alignment) ✅ updated
Follow-up TODOs: None
-->

# AI/Spec-Driven Book Creation Constitution

## Core Principles

### I. Spec-Driven Development
All book content and structure must be defined through clear, testable specifications before implementation. Every chapter, section, and feature begins with a detailed specification that outlines requirements, acceptance criteria, and validation methods.

### II. AI-Assisted Content Creation
Leverage AI tools (Qwen) as primary content creation and editing assistants. All content generation, editing, and review processes must incorporate AI assistance to accelerate development while maintaining quality standards.

### III. Docusaurus-First Architecture
Every book project starts with a Docusaurus foundation. The documentation site must be self-contained, properly configured with appropriate themes, navigation, and search capabilities. All content must be structured to work seamlessly within the Docusaurus framework.

### IV. Spec-Kit Plus Integration
Integrate Spec-Kit Plus methodologies throughout the book creation process. Use Spec-Kit templates, workflows, and tools to ensure consistent, high-quality deliverables that follow best practices for specification-driven development.

### V. GitHub Pages Deployment (NON-NEGOTIABLE)
Every book project must be deployable to GitHub Pages. The build process, static assets, and site configuration must be optimized for GitHub Pages hosting with proper routing, asset loading, and performance considerations.

### VI. Version Control & Documentation
Maintain comprehensive version control for all content, specifications, and configurations. Document all major changes, feature additions, and architectural decisions using Prompt History Records (PHRs) and Architecture Decision Records (ADRs).

## Technology Stack Requirements

All projects must utilize:
- Docusaurus as the primary static site generator
- GitHub Pages for hosting and deployment
- Spec-Kit Plus for specification and workflow management
- Qwen or compatible AI tools for content creation and editing
- Markdown format for all content files
- Git for version control with appropriate branching strategies

## Development Workflow

All book creation follows the Spec-Driven Development lifecycle:
1. Create detailed feature specifications with acceptance criteria
2. Generate content using AI assistance
3. Implement content within Docusaurus framework
4. Test locally and validate against specifications
5. Deploy to GitHub Pages
6. Document changes with PHRs and ADRs as appropriate

Code reviews must verify compliance with all constitutional principles. All major changes require explicit approval and documentation.

## Governance

This constitution supersedes all other practices within the project. Amendments require formal documentation, team approval, and migration planning. All pull requests and reviews must verify constitutional compliance. Complexity must be justified with clear benefits to the overall project goals.

**Version**: 1.0.0 | **Ratified**: 2026-01-05 | **Last Amended**: 2026-01-05
