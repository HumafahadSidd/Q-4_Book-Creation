---
id: 1
title: physical-ai-book-spec
stage: spec
date: 2026-01-05
surface: agent
model: Qwen
feature: physical-ai-book
branch: 003-physical-ai-book
user: kk
command: sp.specify
labels: ["spec", "book", "ai", "robotics"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - C:\Users\kk\Desktop\q-4\spec-kit\BookBySpecKit\specs\003-physical-ai-book\spec.md
  - C:\Users\kk\Desktop\q-4\spec-kit\BookBySpecKit\specs\003-physical-ai-book\checklists\requirements.md
tests:
  - specs\003-physical-ai-book\spec.md validation
  - checklists\requirements.md creation
---

## Prompt

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

The text the user typed after `/sp.specify` in the triggering message **is** the feature description. Assume you always have it available in this conversation even if `# 📜 Project Constitution ## Project Title **AI / Spec‑Driven Book Creation with Docusaurus** ## Purpose & Vision Is project ka maqsad ek **high‑quality, professional technical book** create karna hai jo **AI + Spec‑Driven Development** approach par mabni ho. Book **Docusaurus** par likhi jayegi aur **GitHub Pages** par deploy hogi. Yeh project **Spec‑Kit Plus** methodology follow karega taake: * Clear specifications likhi ja saken * AI ko structured tareeqay se guide kiya ja sake * Reusable intelligence (knowledge) generate ho > ⚠️ Note: Is project mein **Claude Code tool use nahi hoga**. Sirf general AI models (jaise Qwen ya Claude chat) as an assistant use honge. --- ## Core Principles ### 1. Spec‑First Development * Code ya content likhne se pehle **complete specification** zaroori hogi * Har chapter aur section ka clear goal define hoga ### 2. AI as a Co‑Author (Not Owner) * AI sirf **assistant / co‑writer** hoga * Final decision, structure aur approval **human author** ka hoga ### 3. Accuracy & Authenticity * Har technical claim **verifiable** hona chahiye * Hallucinated ya unverified content strictly mana hai ### 4. Beginner‑Friendly but Expert‑Grade * Language simple, clear aur structured hogi * Examples practical aur real‑world based honge --- ## Quality Standards ### Writing Quality * Clear headings (H1 → H4) * Short paragraphs * Bullet points where possible * Code blocks properly formatted ### Technical Quality * Docusaurus compatible Markdown * Correct folder structure * Working examples only ### AI Usage Rules * AI se likha gaya content **review & refine** kiya jayega * Spec ke baghair koi content generate nahi hoga --- ## Documentation Structure ``` /book ├── docs/ │ ├── intro.md │ ├── chapter-1.md │ ├── chapter-2.md │ └── ... ├── docusaurus.config.js ├── sidebars.js └── README.md ``` --- ## Deployment Rules * Source control: **GitHub** * Hosting: **GitHub Pages** * Every major change ke baad deployment update hoga --- ## Ethical & Professional Guidelines * Plagiarism strictly forbidden * Content original aur author‑owned hoga * External references properly credited hongi --- ## Success Criteria Project tab successful mana jayega jab: * Book GitHub Pages par live ho * Har chapter spec‑driven ho * Content reusable aur extendable ho * Reader bina confusion ke follow kar sakay --- ## Final Authority Is project mein **final authority human author** ke paas hogi. AI sirf guidance aur drafting ke liye use hoga. --- ✍️ *This constitution defines the non‑negotiable rules and standards of this project.* --- # 📘 /sp.specify — Book Specification ## Book Title **Physical AI & Humanoid Robotics: From Digital Intelligence to Embodied Systems** ## Book Goal Is book ka goal students ko **digital AI se Physical AI** ki taraf le jana hai — jahan AI models sirf screen tak mehdood nahi rehte balkay **real aur simulated physical environments** mein kaam karte hain. Students is book ke end tak: * Humanoid robotics ke core concepts samajh sakenge * ROS 2 ke zariye robots control kar sakenge * Gazebo, Unity aur NVIDIA Isaac Sim mein robots simulate kar sakenge * Vision‑Language‑Action (VLA) systems build kar sakenge * AI ko physical duniya ke laws ke saath integrate kar sakenge --- ## Target Audience * AI / CS students (Beginner → Intermediate) * Developers jo Robotics ya Physical AI mein entry chahte hain * Professionals jo Embodied Intelligence seekhna chahte hain **Assumed Knowledge:** * Basic Python * Basic AI/ML concepts --- ## Scope (In‑Scope) ### Core Topics Covered * Physical AI & Embodied Intelligence * ROS 2 (Nodes, Topics, Services, Actions) * Python Agents → ROS 2 (rclpy) * URDF & Robot Description * Gazebo Physics Simulation * Unity for Visualization * NVIDIA Isaac Sim & Isaac ROS * VSLAM, Navigation (Nav2) * Vision‑Language‑Action (VLA) * Conversational Robotics (Whisper + LLMs) * Sim‑to‑Real transfer --- ## Out of Scope * Manufacturing of humanoid robots * Low‑level motor driver electronics * Proprietary robot firmware internals * Advanced mechanical engineering math --- ## Book Structure (High‑Level) ### Part I — Foundations of Physical AI * What is Physical AI? * Embodied Intelligence * From Digital Agents to Physical Systems ### Part II — The Robotic Nervous System (ROS 2) * ROS 2 Architecture * Nodes, Topics, Services, Actions * Python‑based ROS 2 packages * URDF for humanoids ### Part III — Digital Twins & Simulation * Gazebo fundamentals * Physics, gravity, collisions * Sensor simulation (LiDAR, Cameras, IMU) * Unity for HRI visualization ### Part IV — The AI Robot Brain (NVIDIA Isaac) * Isaac Sim overview * Synthetic data generation * Isaac ROS pipelines * VSLAM & Navigation * Reinforcement Learning basics ### Part V — Vision‑Language‑Action (VLA) * Voice‑to‑Action using Whisper * LLM‑based task planning * Translating language to ROS actions ### Part VI — Humanoid Systems * Kinematics & dynamics * Bipedal locomotion * Manipulation & grasping * Human‑Robot Interaction ### Part VII — Capstone Project * Autonomous Humanoid System * End‑to‑end pipeline * Simulation → Real deployment --- ## Learning Outcomes Students will be able to: * Design Physical AI systems * Build and deploy ROS 2 packages * Simulate humanoid robots * Use NVIDIA Isaac for perception & navigation * Implement Vision‑Language‑Action pipelines * Build a conversational humanoid robot --- ## Constraints & Assumptions * Primary OS: Ubuntu 22.04 * Simulation‑first approach * Hardware optional but recommended * Cloud‑based alternatives allowed --- ## Success Criteria * Book is fully Spec‑Driven * Each chapter includes: * Clear objectives * Diagrams / examples * Practical exercises * Capstone is reproducible in simulation --- ✍️ *This specification defines WHAT the book will cover and HOW success is measured.*` appears literally below. Do not ask the user to repeat it unless they provided an empty command.

Given that feature description, do this:

1. **Generate a concise short name** (2-4 words) for the branch:
   - Analyze the feature description and extract the most meaningful keywords
   - Create a 2-4 word short name that captures the essence of the feature
   - Use action-noun format when possible (e.g., "add-user-auth", "fix-payment-bug")
   - Preserve technical terms and acronyms (OAuth2, API, JWT, etc.)
   - Keep it concise but descriptive enough to understand the feature at a glance
   - Examples:
     - "I want to add user authentication" → "user-auth"
     - "Implement OAuth2 integration for the API" → "oauth2-api-integration"
     - "Create a dashboard for analytics" → "analytics-dashboard"
     - "Fix payment processing timeout bug" → "fix-payment-timeout"

2. **Check for existing branches before creating new one**:

   a. First, fetch all remote branches to ensure we have the latest information:

      ```bash
      git fetch --all --prune
      ```

   b. Find the highest feature number across all sources for the short-name:
      - Remote branches: `git ls-remote --heads origin | grep -E 'refs/heads/[0-9]+-<short-name>$'`
      - Local branches: `git branch | grep -E '^[* ]*[0-9]+-<short-name>$'`
      - Specs directories: Check for directories matching `specs/[0-9]+-<short-name>`

   c. Determine the next available number:
      - Extract all numbers from all three sources
      - Find the highest number N
      - Use N+1 for the new branch number

   d. Run the script `.specify/scripts/powershell/create-new-feature.ps1 -Json "# 📜 Project Constitution ## Project Title **AI / Spec‑Driven Book Creation with Docusaurus** ## Purpose & Vision Is project ka maqsad ek **high‑quality, professional technical book** create karna hai jo **AI + Spec‑Driven Development** approach par mabni ho. Book **Docusaurus** par likhi jayegi aur **GitHub Pages** par deploy hogi. Yeh project **Spec‑Kit Plus** methodology follow karega taake: * Clear specifications likhi ja saken * AI ko structured tareeqay se guide kiya ja sake * Reusable intelligence (knowledge) generate ho > ⚠️ Note: Is project mein **Claude Code tool use nahi hoga**. Sirf general AI models (jaise Qwen ya Claude chat) as an assistant use honge. --- ## Core Principles ### 1. Spec‑First Development * Code ya content likhne se pehle **complete specification** zaroori hogi * Har chapter aur section ka clear goal define hoga ### 2. AI as a Co‑Author (Not Owner) * AI sirf **assistant / co‑writer** hoga * Final decision, structure aur approval **human author** ka hoga ### 3. Accuracy & Authenticity * Har technical claim **verifiable** hona chahiye * Hallucinated ya unverified content strictly mana hai ### 4. Beginner‑Friendly but Expert‑Grade * Language simple, clear aur structured hogi * Examples practical aur real‑world based honge --- ## Quality Standards ### Writing Quality * Clear headings (H1 → H4) * Short paragraphs * Bullet points where possible * Code blocks properly formatted ### Technical Quality * Docusaurus compatible Markdown * Correct folder structure * Working examples only ### AI Usage Rules * AI se likha gaya content **review & refine** kiya jayega * Spec ke baghair koi content generate nahi hoga --- ## Documentation Structure ``` /book ├── docs/ │ ├── intro.md │ ├── chapter-1.md │ ├── chapter-2.md │ └── ... ├── docusaurus.config.js ├── sidebars.js └── README.md ``` --- ## Deployment Rules * Source control: **GitHub** * Hosting: **GitHub Pages** * Every major change ke baad deployment update hoga --- ## Ethical & Professional Guidelines * Plagiarism strictly forbidden * Content original aur author‑owned hoga * External references properly credited hongi --- ## Success Criteria Project tab successful mana jayega jab: * Book GitHub Pages par live ho * Har chapter spec‑driven ho * Content reusable aur extendable ho * Reader bina confusion ke follow kar sakay --- ## Final Authority Is project mein **final authority human author** ke paas hogi. AI sirf guidance aur drafting ke liye use hoga. --- ✍️ *This constitution defines the non‑negotiable rules and standards of this project.* --- # 📘 /sp.specify — Book Specification ## Book Title **Physical AI & Humanoid Robotics: From Digital Intelligence to Embodied Systems** ## Book Goal Is book ka goal students ko **digital AI se Physical AI** ki taraf le jana hai — jahan AI models sirf screen tak mehdood nahi rehte balkay **real aur simulated physical environments** mein kaam karte hain. Students is book ke end tak: * Humanoid robotics ke core concepts samajh sakenge * ROS 2 ke zariye robots control kar sakenge * Gazebo, Unity aur NVIDIA Isaac Sim mein robots simulate kar sakenge * Vision‑Language‑Action (VLA) systems build kar sakenge * AI ko physical duniya ke laws ke saath integrate kar sakenge --- ## Target Audience * AI / CS students (Beginner → Intermediate) * Developers jo Robotics ya Physical AI mein entry chahte hain * Professionals jo Embodied Intelligence seekhna chahte hain **Assumed Knowledge:** * Basic Python * Basic AI/ML concepts --- ## Scope (In‑Scope) ### Core Topics Covered * Physical AI & Embodied Intelligence * ROS 2 (Nodes, Topics, Services, Actions) * Python Agents → ROS 2 (rclpy) * URDF & Robot Description * Gazebo Physics Simulation * Unity for Visualization * NVIDIA Isaac Sim & Isaac ROS * VSLAM, Navigation (Nav2) * Vision‑Language‑Action (VLA) * Conversational Robotics (Whisper + LLMs) * Sim‑to‑Real transfer --- ## Out of Scope * Manufacturing of humanoid robots * Low‑level motor driver electronics * Proprietary robot firmware internals * Advanced mechanical engineering math --- ## Book Structure (High‑Level) ### Part I — Foundations of Physical AI * What is Physical AI? * Embodied Intelligence * From Digital Agents to Physical Systems ### Part II — The Robotic Nervous System (ROS 2) * ROS 2 Architecture * Nodes, Topics, Services, Actions * Python‑based ROS 2 packages * URDF for humanoids ### Part III — Digital Twins & Simulation * Gazebo fundamentals * Physics, gravity, collisions * Sensor simulation (LiDAR, Cameras, IMU) * Unity for HRI visualization ### Part IV — The AI Robot Brain (NVIDIA Isaac) * Isaac Sim overview * Synthetic data generation * Isaac ROS pipelines * VSLAM & Navigation * Reinforcement Learning basics ### Part V — Vision‑Language‑Action (VLA) * Voice‑to‑Action using Whisper * LLM‑based task planning * Translating language to ROS actions ### Part VI — Humanoid Systems * Kinematics & dynamics * Bipedal locomotion * Manipulation & grasping * Human‑Robot Interaction ### Part VII — Capstone Project * Autonomous Humanoid System * End‑to‑end pipeline * Simulation → Real deployment --- ## Learning Outcomes Students will be able to: * Design Physical AI systems * Build and deploy ROS 2 packages * Simulate humanoid robots * Use NVIDIA Isaac for perception & navigation * Implement Vision‑Language‑Action pipelines * Build a conversational humanoid robot --- ## Constraints & Assumptions * Primary OS: Ubuntu 22.04 * Simulation‑first approach * Hardware optional but recommended * Cloud‑based alternatives allowed --- ## Success Criteria * Book is fully Spec‑Driven * Each chapter includes: * Clear objectives * Diagrams / examples * Practical exercises * Capstone is reproducible in simulation --- ✍️ *This specification defines WHAT the book will cover and HOW success is measured.*"` with the calculated number and short-name:
      - Pass `--number N+1` and `--short-name "your-short-name"` along with the feature description
      - Bash example: `.specify/scripts/powershell/create-new-feature.ps1 -Json "# 📜 Project Constitution ## Project Title **AI / Spec‑Driven Book Creation with Docusaurus** ## Purpose & Vision Is project ka maqsad ek **high‑quality, professional technical book** create karna hai jo **AI + Spec‑Driven Development** approach par mabni ho. Book **Docusaurus** par likhi jayegi aur **GitHub Pages** par deploy hogi. Yeh project **Spec‑Kit Plus** methodology follow karega taake: * Clear specifications likhi ja saken * AI ko structured tareeqay se guide kiya ja sake * Reusable intelligence (knowledge) generate ho > ⚠️ Note: Is project mein **Claude Code tool use nahi hoga**. Sirf general AI models (jaise Qwen ya Claude chat) as an assistant use honge. --- ## Core Principles ### 1. Spec‑First Development * Code ya content likhne se pehle **complete specification** zaroori hogi * Har chapter aur section ka clear goal define hoga ### 2. AI as a Co‑Author (Not Owner) * AI sirf **assistant / co‑writer** hoga * Final decision, structure aur approval **human author** ka hoga ### 3. Accuracy & Authenticity * Har technical claim **verifiable** hona chahiye * Hallucinated ya unverified content strictly mana hai ### 4. Beginner‑Friendly but Expert‑Grade * Language simple, clear aur structured hogi * Examples practical aur real‑world based honge --- ## Quality Standards ### Writing Quality * Clear headings (H1 → H4) * Short paragraphs * Bullet points where possible * Code blocks properly formatted ### Technical Quality * Docusaurus compatible Markdown * Correct folder structure * Working examples only ### AI Usage Rules * AI se likha gaya content **review & refine** kiya jayega * Spec ke baghair koi content generate nahi hoga --- ## Documentation Structure ``` /book ├── docs/ │ ├── intro.md │ ├── chapter-1.md │ ├── chapter-2.md │ └── ... ├── docusaurus.config.js ├── sidebars.js └── README.md ``` --- ## Deployment Rules * Source control: **GitHub** * Hosting: **GitHub Pages** * Every major change ke baad deployment update hoga --- ## Ethical & Professional Guidelines * Plagiarism strictly forbidden * Content original aur author‑owned hoga * External references properly credited hongi --- ## Success Criteria Project tab successful mana jayega jab: * Book GitHub Pages par live ho * Har chapter spec‑driven ho * Content reusable aur extendable ho * Reader bina confusion ke follow kar sakay --- ## Final Authority Is project mein **final authority human author** ke paas hogi. AI sirf guidance aur drafting ke liye use hoga. --- ✍️ *This constitution defines the non‑negotiable rules and standards of this project.* --- # 📘 /sp.specify — Book Specification ## Book Title **Physical AI & Humanoid Robotics: From Digital Intelligence to Embodied Systems** ## Book Goal Is book ka goal students ko **digital AI se Physical AI** ki taraf le jana hai — jahan AI models sirf screen tak mehdood nahi rehte balkay **real aur simulated physical environments** mein kaam karte hain. Students is book ke end tak: * Humanoid robotics ke core concepts samajh sakenge * ROS 2 ke zariye robots control kar sakenge * Gazebo, Unity aur NVIDIA Isaac Sim mein robots simulate kar sakenge * Vision‑Language‑Action (VLA) systems build kar sakenge * AI ko physical duniya ke laws ke saath integrate kar sakenge --- ## Target Audience * AI / CS students (Beginner → Intermediate) * Developers jo Robotics ya Physical AI mein entry chahte hain * Professionals jo Embodied Intelligence seekhna chahte hain **Assumed Knowledge:** * Basic Python * Basic AI/ML concepts --- ## Scope (In‑Scope) ### Core Topics Covered * Physical AI & Embodied Intelligence * ROS 2 (Nodes, Topics, Services, Actions) * Python Agents → ROS 2 (rclpy) * URDF & Robot Description * Gazebo Physics Simulation * Unity for Visualization * NVIDIA Isaac Sim & Isaac ROS * VSLAM, Navigation (Nav2) * Vision‑Language‑Action (VLA) * Conversational Robotics (Whisper + LLMs) * Sim‑to‑Real transfer --- ## Out of Scope * Manufacturing of humanoid robots * Low‑level motor driver electronics * Proprietary robot firmware internals * Advanced mechanical engineering math --- ## Book Structure (High‑Level) ### Part I — Foundations of Physical AI * What is Physical AI? * Embodied Intelligence * From Digital Agents to Physical Systems ### Part II — The Robotic Nervous System (ROS 2) * ROS 2 Architecture * Nodes, Topics, Services, Actions * Python‑based ROS 2 packages * URDF for humanoids ### Part III — Digital Twins & Simulation * Gazebo fundamentals * Physics, gravity, collisions * Sensor simulation (LiDAR, Cameras, IMU) * Unity for HRI visualization ### Part IV — The AI Robot Brain (NVIDIA Isaac) * Isaac Sim overview * Synthetic data generation * Isaac ROS pipelines * VSLAM & Navigation * Reinforcement Learning basics ### Part V — Vision‑Language‑Action (VLA) * Voice‑to‑Action using Whisper * LLM‑based task planning * Translating language to ROS actions ### Part VI — Humanoid Systems * Kinematics & dynamics * Bipedal locomotion * Manipulation & grasping * Human‑Robot Interaction ### Part VII — Capstone Project * Autonomous Humanoid System * End‑to‑end pipeline * Simulation → Real deployment --- ## Learning Outcomes Students will be able to: * Design Physical AI systems * Build and deploy ROS 2 packages * Simulate humanoid robots * Use NVIDIA Isaac for perception & navigation * Implement Vision‑Language‑Action pipelines * Build a conversational humanoid robot --- ## Constraints & Assumptions * Primary OS: Ubuntu 22.04 * Simulation‑first approach * Hardware optional but recommended * Cloud‑based alternatives allowed --- ## Success Criteria * Book is fully Spec‑Driven * Each chapter includes: * Clear objectives * Diagrams / examples * Practical exercises * Capstone is reproducible in simulation --- ✍️ *This specification defines WHAT the book will cover and HOW success is measured.*" --json --number 5 --short-name "user-auth" "Add user authentication"`
      - PowerShell example: `.specify/scripts/powershell/create-new-feature.ps1 -Json "# 📜 Project Constitution ## Project Title **AI / Spec‑Driven Book Creation with Docusaurus** ## Purpose & Vision Is project ka maqsad ek **high‑quality, professional technical book** create karna hai jo **AI + Spec‑Driven Development** approach par mabni ho. Book **Docusaurus** par likhi jayegi aur **GitHub Pages** par deploy hogi. Yeh project **Spec‑Kit Plus** methodology follow karega taake: * Clear specifications likhi ja saken * AI ko structured tareeqay se guide kiya ja sake * Reusable intelligence (knowledge) generate ho > ⚠️ Note: Is project mein **Claude Code tool use nahi hoga**. Sirf general AI models (jaise Qwen ya Claude chat) as an assistant use honge. --- ## Core Principles ### 1. Spec‑First Development * Code ya content likhne se pehle **complete specification** zaroori hogi * Har chapter aur section ka clear goal define hoga ### 2. AI as a Co‑Author (Not Owner) * AI sirf **assistant / co‑writer** hoga * Final decision, structure aur approval **human author** ka hoga ### 3. Accuracy & Authenticity * Har technical claim **verifiable** hona chahiye * Hallucinated ya unverified content strictly mana hai ### 4. Beginner‑Friendly but Expert‑Grade * Language simple, clear aur structured hogi * Examples practical aur real‑world based honge --- ## Quality Standards ### Writing Quality * Clear headings (H1 → H4) * Short paragraphs * Bullet points where possible * Code blocks properly formatted ### Technical Quality * Docusaurus compatible Markdown * Correct folder structure * Working examples only ### AI Usage Rules * AI se likha gaya content **review & refine** kiya jayega * Spec ke baghair koi content generate nahi hoga --- ## Documentation Structure ``` /book ├── docs/ │ ├── intro.md │ ├── chapter-1.md │ ├── chapter-2.md │ └── ... ├── docusaurus.config.js ├── sidebars.js └── README.md ``` --- ## Deployment Rules * Source control: **GitHub** * Hosting: **GitHub Pages** * Every major change ke baad deployment update hoga --- ## Ethical & Professional Guidelines * Plagiarism strictly forbidden * Content original aur author‑owned hoga * External references properly credited hongi --- ## Success Criteria Project tab successful mana jayega jab: * Book GitHub Pages par live ho * Har chapter spec‑driven ho * Content reusable aur extendable ho * Reader bina confusion ke follow kar sakay --- ## Final Authority Is project mein **final authority human author** ke paas hogi. AI sirf guidance aur drafting ke liye use hoga. --- ✍️ *This constitution defines the non‑negotiable rules and standards of this project.* --- # 📘 /sp.specify — Book Specification ## Book Title **Physical AI & Humanoid Robotics: From Digital Intelligence to Embodied Systems** ## Book Goal Is book ka goal students ko **digital AI se Physical AI** ki taraf le jana hai — jahan AI models sirf screen tak mehdood nahi rehte balkay **real aur simulated physical environments** mein kaam karte hain. Students is book ke end tak: * Humanoid robotics ke core concepts samajh sakenge * ROS 2 ke zariye robots control kar sakenge * Gazebo, Unity aur NVIDIA Isaac Sim mein robots simulate kar sakenge * Vision‑Language‑Action (VLA) systems build kar sakenge * AI ko physical duniya ke laws ke saath integrate kar sakenge --- ## Target Audience * AI / CS students (Beginner → Intermediate) * Developers jo Robotics ya Physical AI mein entry chahte hain * Professionals jo Embodied Intelligence seekhna chahte hain **Assumed Knowledge:** * Basic Python * Basic AI/ML concepts --- ## Scope (In‑Scope) ### Core Topics Covered * Physical AI & Embodied Intelligence * ROS 2 (Nodes, Topics, Services, Actions) * Python Agents → ROS 2 (rclpy) * URDF & Robot Description * Gazebo Physics Simulation * Unity for Visualization * NVIDIA Isaac Sim & Isaac ROS * VSLAM, Navigation (Nav2) * Vision‑Language‑Action (VLA) * Conversational Robotics (Whisper + LLMs) * Sim‑to‑Real transfer --- ## Out of Scope * Manufacturing of humanoid robots * Low‑level motor driver electronics * Proprietary robot firmware internals * Advanced mechanical engineering math --- ## Book Structure (High‑Level) ### Part I — Foundations of Physical AI * What is Physical AI? * Embodied Intelligence * From Digital Agents to Physical Systems ### Part II — The Robotic Nervous System (ROS 2) * ROS 2 Architecture * Nodes, Topics, Services, Actions * Python‑based ROS 2 packages * URDF for humanoids ### Part III — Digital Twins & Simulation * Gazebo fundamentals * Physics, gravity, collisions * Sensor simulation (LiDAR, Cameras, IMU) * Unity for HRI visualization ### Part IV — The AI Robot Brain (NVIDIA Isaac) * Isaac Sim overview * Synthetic data generation * Isaac ROS pipelines * VSLAM & Navigation * Reinforcement Learning basics ### Part V — Vision‑Language‑Action (VLA) * Voice‑to‑Action using Whisper * LLM‑based task planning * Translating language to ROS actions ### Part VI — Humanoid Systems * Kinematics & dynamics * Bipedal locomotion * Manipulation & grasping * Human‑Robot Interaction ### Part VII — Capstone Project * Autonomous Humanoid System * End‑to‑end pipeline * Simulation → Real deployment --- ## Learning Outcomes Students will be able to: * Design Physical AI systems * Build and deploy ROS 2 packages * Simulate humanoid robots * Use NVIDIA Isaac for perception & navigation * Implement Vision‑Language‑Action pipelines * Build a conversational humanoid robot --- ## Constraints & Assumptions * Primary OS: Ubuntu 22.04 * Simulation‑first approach * Hardware optional but recommended * Cloud‑based alternatives allowed --- ## Success Criteria * Book is fully Spec‑Driven * Each chapter includes: * Clear objectives * Diagrams / examples * Practical exercises * Capstone is reproducible in simulation --- ✍️ *This specification defines WHAT the book will cover and HOW success is measured.*" -Json -Number 5 -ShortName "user-auth" "Add user authentication"`

   **IMPORTANT**:
   - Check all three sources (remote branches, local branches, specs directories) to find the highest number
   - Only match branches/directories with the exact short-name pattern
   - If no existing branches/directories found with this short-name, start with number 1
   - You must only ever run this script once per feature
   - The JSON is provided in the terminal as output - always refer to it to get the actual content you're looking for
   - The JSON output will contain BRANCH_NAME and SPEC_FILE paths
   - For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\''m Groot' (or double-quote if possible: "I'm Groot")

3. Load `.specify/templates/spec-template.md` to understand required sections.

4. Follow this execution flow:

    1. Parse user description from Input
       If empty: ERROR "No feature description provided"
    2. Extract key concepts from description
       Identify: actors, actions, data, constraints
    3. For unclear aspects:
       - Make informed guesses based on context and industry standards
       - Only mark with [NEEDS CLARIFICATION: specific question] if:
         - The choice significantly impacts feature scope or user experience
         - Multiple reasonable interpretations exist with different implications
         - No reasonable default exists
       - **LIMIT: Maximum 3 [NEEDS CLARIFICATION] markers total**
       - Prioritize clarifications by impact: scope > security/privacy > user experience > technical details
    4. Fill User Scenarios & Testing section
       If no clear user flow: ERROR "Cannot determine user scenarios"
    5. Generate Functional Requirements
       Each requirement must be testable
       Use reasonable defaults for unspecified details (document assumptions in Assumptions section)
    6. Define Success Criteria
       Create measurable, technology-agnostic outcomes
       Include both quantitative metrics (time, performance, volume) and qualitative measures (user satisfaction, task completion)
       Each criterion must be verifiable without implementation details
    7. Identify Key Entities (if data involved)
    8. Return: SUCCESS (spec ready for planning)

5. Write the specification to SPEC_FILE using the template structure, replacing placeholders with concrete details derived from the feature description (arguments) while preserving section order and headings.

6. **Specification Quality Validation**: After writing the initial spec, validate it against quality criteria:

   a. **Create Spec Quality Checklist**: Generate a checklist file at `FEATURE_DIR/checklists/requirements.md` using the checklist template structure with these validation items:

      ```markdown
      # Specification Quality Checklist: [FEATURE NAME]
      
      **Purpose**: Validate specification completeness and quality before proceeding to planning
      **Created**: [DATE]
      **Feature**: [Link to spec.md]
      
      ## Content Quality
      
      - [ ] No implementation details (languages, frameworks, APIs)
      - [ ] Focused on user value and business needs
      - [ ] Written for non-technical stakeholders
      - [ ] All mandatory sections completed
      
      ## Requirement Completeness
      
      - [ ] No [NEEDS CLARIFICATION] markers remain
      - [ ] Requirements are testable and unambiguous
      - [ ] Success criteria are measurable
      - [ ] Success criteria are technology-agnostic (no implementation details)
      - [ ] All acceptance scenarios are defined
      - [ ] Edge cases are identified
      - [ ] Scope is clearly bounded
      - [ ] Dependencies and assumptions identified
      
      ## Feature Readiness
      
      - [ ] All functional requirements have clear acceptance criteria
      - [ ] User scenarios cover primary flows
      - [ ] Feature meets measurable outcomes defined in Success Criteria
      - [ ] No implementation details leak into specification
      
      ## Notes
      
      - Items marked incomplete require spec updates before `/sp.clarify` or `/sp.plan`
      ```

   b. **Run Validation Check**: Review the spec against each checklist item:
      - For each item, determine if it passes or fails
      - Document specific issues found (quote relevant spec sections)

   c. **Handle Validation Results**:

      - **If all items pass**: Mark checklist complete and proceed to step 6

      - **If items fail (excluding [NEEDS CLARIFICATION])**:
        1. List the failing items and specific issues
        2. Update the spec to address each issue
        3. Re-run validation until all items pass (max 3 iterations)
        4. If still failing after 3 iterations, document remaining issues in checklist notes and warn user

      - **If [NEEDS CLARIFICATION] markers remain**:
        1. Extract all [NEEDS CLARIFICATION: ...] markers from the spec
        2. **LIMIT CHECK**: If more than 3 markers exist, keep only the 3 most critical (by scope/security/UX impact) and make informed guesses for the rest
        3. For each clarification needed (max 3), present options to user in this format:

           ```markdown
           ## Question [N]: [Topic]
           
           **Context**: [Quote relevant spec section]
           
           **What we need to know**: [Specific question from NEEDS CLARIFICATION marker]
           
           **Suggested Answers**:
           
           | Option | Answer | Implications |
           |--------|--------|--------------|
           | A      | [First suggested answer] | [What this means for the feature] |
           | B      | [Second suggested answer] | [What this means for the feature] |
           | C      | [Third suggested answer] | [What this means for the feature] |
           | Custom | Provide your own answer | [Explain how to provide custom input] |
           
           **Your choice**: _[Wait for user response]_
           ```

        4. **CRITICAL - Table Formatting**: Ensure markdown tables are properly formatted:
           - Use consistent spacing with pipes aligned
           - Each cell should have spaces around content: `| Content |` not `|Content|`
           - Header separator must have at least 3 dashes: `|--------|`
           - Test that the table renders correctly in markdown preview
        5. Number questions sequentially (Q1, Q2, Q3 - max 3 total)
        6. Present all questions together before waiting for responses
        7. Wait for user to respond with their choices for all questions (e.g., "Q1: A, Q2: Custom - [details], Q3: B")
        8. Update the spec by replacing each [NEEDS CLARIFICATION] marker with the user's selected or provided answer
        9. Re-run validation after all clarifications are resolved

   d. **Update Checklist**: After each validation iteration, update the checklist file with current pass/fail status

7. Report completion with branch name, spec file path, checklist results, and readiness for the next phase (`/sp.clarify` or `/sp.plan`).

**NOTE:** The script creates and checks out the new branch and initializes the spec file before writing.

## General Guidelines

## Quick Guidelines

- Focus on **WHAT** users need and **WHY**.
- Avoid HOW to implement (no tech stack, APIs, code structure).
- Written for business stakeholders, not developers.
- DO NOT create any checklists that are embedded in the spec. That will be a separate command.

### Section Requirements

- **Mandatory sections**: Must be completed for every feature
- **Optional sections**: Include only when relevant to the feature
- When a section doesn't apply, remove it entirely (don't leave as "N/A")

### For AI Generation

When creating this spec from a user prompt:

1. **Make informed guesses**: Use context, industry standards, and common patterns to fill gaps
2. **Document assumptions**: Record reasonable defaults in the Assumptions section
3. **Limit clarifications**: Maximum 3 [NEEDS CLARIFICATION] markers - use only for critical decisions that:
   - Significantly impact feature scope or user experience
   - Have multiple reasonable interpretations with different implications
   - Lack any reasonable default
4. **Prioritize clarifications**: scope > security/privacy > user experience > technical details
5. **Think like a tester**: Every vague requirement should fail the "testable and unambiguous" checklist item
6. **Common areas needing clarification** (only if no reasonable default exists):
   - Feature scope and boundaries (include/exclude specific use cases)
   - User types and permissions (if multiple conflicting interpretations possible)
   - Security/compliance requirements (when legally/financially significant)

**Examples of reasonable defaults** (don't ask about these):

- Data retention: Industry-standard practices for the domain
- Performance targets: Standard web/mobile app expectations unless specified
- Error handling: User-friendly messages with appropriate fallbacks
- Authentication method: Standard session-based or OAuth2 for web apps
- Integration patterns: RESTful APIs unless specified otherwise

### Success Criteria Guidelines

Success criteria must be:

1. **Measurable**: Include specific metrics (time, percentage, count, rate)
2. **Technology-agnostic**: No mention of frameworks, languages, databases, or tools
3. **User-focused**: Describe outcomes from user/business perspective, not system internals
4. **Verifiable**: Can be tested/validated without knowing implementation details

**Good examples**:

- "Users can complete checkout in under 3 minutes"
- "System supports 10,000 concurrent users"
- "95% of searches return results in under 1 second"
- "Task completion rate improves by 40%"

**Bad examples** (implementation-focused):

- "API response time is under 200ms" (too technical, use "Users see results instantly")
- "Database can handle 1000 TPS" (implementation detail, use user-facing metric)
- "React components render efficiently" (framework-specific)
- "Redis cache hit rate above 80%" (technology-specific)

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