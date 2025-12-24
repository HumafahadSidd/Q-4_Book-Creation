---
id: 3
title: Create Physical AI Book Plan
stage: plan
date_iso: 2025-12-24
surface: agent
model: Qwen
feature: physical-ai-book
branch: 001-physical-ai-book
user: user
command: /sp.plan
labels:
  - planning
  - physical-ai
  - education
  - robotics
  - curriculum
links:
  spec: specs/001-physical-ai-book/spec.md
  plan: specs/001-physical-ai-book/plan.md
  ticket: null
  adr: null
  pr: null
files_yaml:
  - specs/001-physical-ai-book/plan.md
  - specs/001-physical-ai-book/research.md
  - specs/001-physical-ai-book/data-model.md
  - specs/001-physical-ai-book/quickstart.md
  - specs/001-physical-ai-book/contracts/api-contract.yaml
tests_yaml: []
prompt_text: |
  ---
  description: Execute the implementation planning workflow using the plan template to generate design artifacts.
  handoffs: 
    - label: Create Tasks
      agent: sp.tasks
      prompt: Break the plan into tasks
      send: true
    - label: Create Checklist
      agent: sp.checklist
      prompt: Create a checklist for the following domain...
  ---

  ## User Input

  ```text
  $ARGUMENTS
  ```

  You **MUST** consider the user input before proceeding (if not empty).

  ## Outline

  1. **Setup**: Run `.specify/scripts/powershell/setup-plan.ps1 -Json` from repo root and parse JSON for FEATURE_SPEC, IMPL_PLAN, SPECS_DIR, BRANCH. For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\''m Groot' (or double-quote if possible: "I'm Groot").

  2. **Load context**: Read FEATURE_SPEC and `.specify/memory/constitution.md`. Load IMPL_PLAN template (already copied).

  3. **Execute plan workflow**: Follow the structure in IMPL_PLAN template to:
     - Fill Technical Context (mark unknowns as "NEEDS CLARIFICATION")
     - Fill Constitution Check section from constitution
     - Evaluate gates (ERROR if violations unjustified)
     - Phase 0: Generate research.md (resolve all NEEDS CLARIFICATION)
     - Phase 1: Generate data-model.md, contracts/, quickstart.md
     - Phase 1: Update agent context by running the agent script
     - Re-evaluate Constitution Check post-design

  4. **Stop and report**: Command ends after Phase 2 planning. Report branch, IMPL_PLAN path, and generated artifacts.

  ## Phases

  ### Phase 0: Outline & Research

  1. **Extract unknowns from Technical Context** above:
     - For each NEEDS CLARIFICATION → research task
     - For each dependency → best practices task
     - For each integration → patterns task

  2. **Generate and dispatch research agents**:

     ```text
     For each unknown in Technical Context:
       Task: "Research {unknown} for {feature context}"
     For each technology choice:
       Task: "Find best practices for {tech} in {domain}"
     ```

  3. **Consolidate findings** in `research.md` using format:
     - Decision: [what was chosen]
     - Rationale: [why chosen]
     - Alternatives considered: [what else evaluated]

  **Output**: research.md with all NEEDS CLARIFICATION resolved

  ### Phase 1: Design & Contracts

  **Prerequisites:** `research.md` complete

  1. **Extract entities from feature spec** → `data-model.md`:
     - Entity name, fields, relationships
     - Validation rules from requirements
     - State transitions if applicable

  2. **Generate API contracts** from functional requirements:
     - For each user action → endpoint
     - Use standard REST/GraphQL patterns
     - Output OpenAPI/GraphQL schema to `/contracts/`

  3. **Agent context update**:
     - Run `.specify/scripts/powershell/update-agent-context.ps1 -AgentType qwen`
     - These scripts detect which AI agent is in use
     - Update the appropriate agent-specific context file
     - Add only new technology from current plan
     - Preserve manual additions between markers

  **Output**: data-model.md, /contracts/*, quickstart.md, agent-specific file

  ## Key rules

  - Use absolute paths
  - ERROR on gate failures or unresolved clarifications

  ---

  /sp.plan Create a detailed, end-to-end writing and production plan for the book
  "Physical AI & Humanoid Robotics: From Digital Intelligence to Embodied Intelligence".

  The plan must transform the specification into a structured execution roadmap
  covering content creation, technical depth progression, and learning outcomes.

  The plan should include:
  - High-level book architecture (Parts, Modules, Chapters)
  - Chapter-by-chapter writing sequence with dependencies
  - Weekly breakdown aligned with a 13-week capstone-style course
  - Mapping of each week to chapters, tools, and core concepts
  - Identification of prerequisite knowledge for each module
  - Placement of hands-on examples, simulations, and case studies
  - Dedicated milestones for ROS 2, Gazebo, NVIDIA Isaac, and VLA integration
  - A clearly defined Capstone Project plan (Autonomous Humanoid Robot)
  - Guidance for Sim-to-Real transition phases
  - Review, refinement, and consistency checkpoints

  The output should be:
  - Structured in clear sections and bullet points
  - Optimized for Spec-Kit Plus execution
  - Aligned with the project constitution
  - Written from the perspective of a senior AI & Robotics architect

  Do not generate the actual book content.
  Only produce the strategic writing and development plan.

response_text: |
  I've successfully created the implementation plan for the "Physical AI & Humanoid Robotics" book. Here's what was completed:

  1. Created the implementation plan file (plan.md) with:
     - Detailed technical context for the educational book
     - Constitution check alignment
     - Project structure with content organization
     - Phase-by-phase implementation plan
     - Risk mitigation strategies
     - Success metrics

  2. Created the research.md file with:
     - Technology decisions for ROS 2, simulation platforms, and NVIDIA Isaac
     - Rationales for each decision
     - Alternatives considered

  3. Created the data-model.md file with:
     - Entities for educational modules, chapters, exercises, and resources
     - Relationships between entities
     - Validation rules
     - State transitions

  4. Created the quickstart.md guide with:
     - Software setup instructions
     - Course content navigation
     - Exercise execution guidance
     - Capstone project setup

  5. Created the API contract in contracts/api-contract.yaml with:
     - Endpoints for student management
     - Module content access
     - Exercise submission and evaluation
     - Simulation environment interaction

  All required artifacts have been created according to the implementation plan workflow. The plan is comprehensive and aligned with the project constitution, focusing on embodied intelligence, educational accessibility, safety and ethics, Sim2Real approach, VLA integration, and technical foundation.
outcome: Successfully created comprehensive implementation plan for the Physical AI & Humanoid Robotics educational book
evaluation: The plan includes all required components (research, data model, quickstart guide, contracts) and aligns with the project constitution. It provides a structured roadmap for developing the educational content.
---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage:
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.