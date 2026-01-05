# Data Model: Physical AI & Humanoid Robotics Book

## Overview

This document defines the data model for the Physical AI & Humanoid Robotics book project, based on the entities identified in the feature specification. The data model encompasses the core content structure, relationships between different components, and validation rules derived from the requirements.

## Core Entities

### 1. Book Content

**Description**: The primary entity representing the educational material covering Physical AI and Humanoid Robotics concepts.

**Fields**:
- `id`: Unique identifier for the book content (string, required)
- `title`: Title of the book content (string, required)
- `version`: Version of the content (string, required)
- `type`: Type of content (enum: "part", "chapter", "section", "exercise", "diagram", "code_example", required)
- `content`: The actual content in Markdown format (string, required)
- `objectives`: Learning objectives for the content (array of strings, required)
- `prerequisites`: Prerequisites needed to understand the content (array of strings, optional)
- `difficulty`: Difficulty level (enum: "beginner", "intermediate", "advanced", required)
- `estimated_time`: Estimated time to complete (integer, in minutes, required)
- `created_date`: Date when content was created (date, required)
- `last_updated`: Date when content was last updated (date, required)
- `authors`: List of authors who contributed to the content (array of strings, required)
- `tags`: Tags for categorizing content (array of strings, optional)

**Relationships**:
- One-to-many with Book Content (parent-child for parts/chapters/sections)
- One-to-many with Exercises (exercises related to this content)
- One-to-many with Diagrams (diagrams used in this content)
- One-to-many with Code Examples (code examples in this content)

**Validation Rules**:
- Title must be between 5 and 200 characters
- Content must not be empty
- Type must be one of the defined enum values
- Difficulty must be one of the defined enum values
- Estimated time must be between 5 and 480 minutes

### 2. Simulation Environments

**Description**: Digital platforms where students can practice robotics concepts, including Gazebo, Unity, and NVIDIA Isaac Sim.

**Fields**:
- `id`: Unique identifier for the simulation environment (string, required)
- `name`: Name of the simulation environment (string, required)
- `version`: Version of the simulation environment (string, required)
- `description`: Brief description of the simulation environment (string, required)
- `requirements`: System requirements for running the environment (string, required)
- `installation_guide`: Step-by-step installation guide (string, required)
- `tutorials`: List of available tutorials (array of strings, optional)
- `supported_robots`: List of robots supported in this environment (array of strings, optional)
- `features`: Key features of the simulation environment (array of strings, required)
- `limitations`: Known limitations of the environment (array of strings, optional)
- `integration_points`: How this environment integrates with other tools (array of strings, required)

**Relationships**:
- Many-to-many with Book Content (content that references this environment)
- One-to-many with Code Examples (examples specific to this environment)

**Validation Rules**:
- Name must be between 2 and 50 characters
- Description must not exceed 500 characters
- Requirements must not be empty
- At least one feature must be specified

### 3. ROS 2 Components

**Description**: The middleware architecture including nodes, topics, services, and actions that enable robot communication.

**Fields**:
- `id`: Unique identifier for the ROS 2 component (string, required)
- `name`: Name of the ROS 2 component (string, required)
- `type`: Type of ROS 2 component (enum: "node", "topic", "service", "action", required)
- `description`: Description of the component (string, required)
- `purpose`: Purpose of the component in ROS 2 architecture (string, required)
- `implementation_language`: Language used for implementation (enum: "Python", "C++", "both", required)
- `dependencies`: List of dependencies for this component (array of strings, optional)
- `parameters`: Configuration parameters for the component (array of objects, optional)
- `messages`: Message types used by the component (array of strings, optional)
- `examples`: Code examples demonstrating usage (array of strings, optional)
- `best_practices`: Best practices for using this component (array of strings, optional)

**Relationships**:
- Many-to-many with Book Content (content that covers this component)
- One-to-many with Code Examples (examples of this component)

**Validation Rules**:
- Name must be between 2 and 100 characters
- Type must be one of the defined enum values
- Description must not be empty
- Implementation language must be one of the defined enum values

### 4. Student Learning Path

**Description**: The structured journey from basic Physical AI concepts to advanced VLA systems and capstone project completion.

**Fields**:
- `id`: Unique identifier for the learning path (string, required)
- `name`: Name of the learning path (string, required)
- `description`: Description of the learning path (string, required)
- `target_audience`: Target audience for this path (enum: "beginner", "intermediate", "advanced", required)
- `duration`: Estimated duration of the path (integer, in hours, required)
- `prerequisites`: Prerequisites for starting this path (array of strings, required)
- `milestones`: Key milestones in the learning path (array of objects, required)
- `learning_outcomes`: Expected learning outcomes (array of strings, required)
- `assessment_methods`: Methods for assessing progress (array of strings, optional)
- `support_resources`: Additional resources for support (array of strings, optional)

**Relationships**:
- Many-to-many with Book Content (content that forms part of this path)
- One-to-many with Exercises (exercises that assess progress in this path)

**Validation Rules**:
- Name must be between 5 and 100 characters
- Target audience must be one of the defined enum values
- Duration must be between 10 and 200 hours
- At least one milestone must be specified
- At least one learning outcome must be specified

## State Transitions

### Book Content States
- `draft` → `review` → `approved` → `published`
- `published` → `deprecated` (when content becomes outdated)

### Learning Path States
- `planned` → `active` → `completed`
- `active` → `on_hold` (if student pauses progress)

## Relationships Summary

```
Book Content (1) ←→ (M) Book Content (parent-child hierarchy)
Book Content (M) ←→ (M) Simulation Environments (references)
Book Content (M) ←→ (M) ROS 2 Components (covers)
Book Content (M) ←→ (M) Student Learning Path (part of)

Simulation Environments (M) ←→ (M) Code Examples (specific to)
ROS 2 Components (M) ←→ (M) Code Examples (demonstrates)

Student Learning Path (M) ←→ (M) Exercises (assesses)
Book Content (M) ←→ (M) Exercises (includes)
```

## Validation Rules Summary

1. **Book Content Validation**:
   - All required fields must be present
   - Content must meet quality standards (reviewed and approved)
   - Estimated time must be realistic

2. **Simulation Environments Validation**:
   - Installation guides must be tested and verified
   - System requirements must be accurate
   - Integration points must be documented

3. **ROS 2 Components Validation**:
   - Examples must be functional and tested
   - Dependencies must be clearly specified
   - Best practices must align with current standards

4. **Student Learning Path Validation**:
   - Prerequisites must be properly defined
   - Milestones must be achievable
   - Learning outcomes must be measurable