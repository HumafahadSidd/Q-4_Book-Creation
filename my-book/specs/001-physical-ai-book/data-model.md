# Data Model: Physical AI & Humanoid Robotics Educational Book

## Educational Modules

**Entity**: Module
- **Fields**: 
  - module_id (string): Unique identifier (e.g., "01-introduction", "02-embodied-intelligence")
  - title (string): Module title
  - description (text): Brief description of module content
  - duration_weeks (number): Estimated duration in weeks
  - learning_objectives (array of strings): List of learning objectives
  - prerequisites (array of strings): Prerequisites from other modules
  - required_tools (array of strings): Tools/software needed for this module
  - content_path (string): Path to module content files

**Relationships**:
- Module contains multiple Chapters
- Module contains multiple Exercises
- Module references multiple Resources

## Chapters

**Entity**: Chapter
- **Fields**:
  - chapter_id (string): Unique identifier within module
  - title (string): Chapter title
  - module_id (string): Reference to parent module
  - content_type (string): Type of content (e.g., "conceptual", "tutorial", "application")
  - estimated_reading_time (number): In minutes
  - key_concepts (array of strings): List of key concepts covered
  - learning_outcomes (array of strings): Specific outcomes for this chapter
  - chapter_path (string): Path to chapter content file

**Relationships**:
- Chapter belongs to one Module
- Chapter contains multiple Exercises
- Chapter references multiple Resources

## Exercises

**Entity**: Exercise
- **Fields**:
  - exercise_id (string): Unique identifier
  - title (string): Exercise title
  - module_id (string): Reference to parent module
  - chapter_id (string): Reference to parent chapter (optional)
  - difficulty_level (string): "beginner", "intermediate", "advanced"
  - exercise_type (string): "simulation", "hardware", "theoretical", "analysis"
  - estimated_completion_time (number): In minutes
  - objectives (array of strings): Learning objectives for this exercise
  - instructions (text): Detailed instructions
  - evaluation_criteria (array of strings): How the exercise will be evaluated
  - exercise_path (string): Path to exercise file

**Relationships**:
- Exercise belongs to one Module
- Exercise optionally belongs to one Chapter
- Exercise references multiple Resources

## Resources

**Entity**: Resource
- **Fields**:
  - resource_id (string): Unique identifier
  - title (string): Resource title
  - resource_type (string): "documentation", "video", "code", "paper", "tool", "dataset"
  - url (string): URL or path to resource
  - description (text): Brief description of resource
  - module_id (string): Reference to relevant module
  - chapter_id (string): Reference to relevant chapter (optional)
  - tags (array of strings): Tags for categorization

**Relationships**:
- Resource belongs to one or more Modules
- Resource optionally belongs to one or more Chapters

## Capstone Project

**Entity**: CapstoneProject
- **Fields**:
  - project_id (string): Unique identifier
  - title (string): Project title
  - description (text): Detailed project description
  - objectives (array of strings): Learning objectives
  - milestones (array of objects): 
    - milestone_name (string)
    - milestone_description (text)
    - milestone_deadline (date)
    - deliverables (array of strings)
  - evaluation_criteria (array of strings): How the project will be evaluated
  - required_skills (array of strings): Skills needed to complete the project
  - tools_required (array of strings): Tools/software needed for the project
  - project_path (string): Path to project guidelines

**Relationships**:
- CapstoneProject integrates multiple Modules
- CapstoneProject references multiple Resources

## Assessment Items

**Entity**: Assessment
- **Fields**:
  - assessment_id (string): Unique identifier
  - title (string): Assessment title
  - module_id (string): Reference to relevant module
  - assessment_type (string): "quiz", "practical", "project", "presentation"
  - difficulty_level (string): "beginner", "intermediate", "advanced"
  - duration_minutes (number): Estimated time to complete
  - questions (array of objects): List of questions with answer options
  - evaluation_criteria (array of strings): How the assessment will be evaluated
  - assessment_path (string): Path to assessment file

**Relationships**:
- Assessment belongs to one Module
- Assessment optionally references one or more Chapters

## Validation Rules

### Module Validation
- module_id must follow format: "NN-module-name" where NN is a 2-digit number
- title must be 5-100 characters
- duration_weeks must be between 0.5 and 4
- learning_objectives must contain at least 2 items
- required_tools must be a non-empty array

### Chapter Validation
- chapter_id must be unique within its module
- content_type must be one of the predefined values
- estimated_reading_time must be between 15 and 180 minutes
- learning_outcomes must align with parent module's learning objectives

### Exercise Validation
- difficulty_level must be one of "beginner", "intermediate", "advanced"
- exercise_type must be one of the predefined values
- estimated_completion_time must be between 15 and 300 minutes
- evaluation_criteria must be specific and measurable

### Capstone Project Validation
- milestones must be in chronological order
- objectives must align with overall course objectives
- required_skills must align with covered modules
- evaluation_criteria must be comprehensive and measurable

## State Transitions

### Content Development States
- **Draft**: Initial content creation
- **Review**: Content under review by subject matter experts
- **Revised**: Content updated based on feedback
- **Approved**: Content approved for use
- **Published**: Content made available to students

### Exercise/Assessment States
- **Design**: Exercise/assessment being designed
- **Pilot**: Exercise/assessment being tested with small group
- **Validated**: Exercise/assessment validated and ready for use
- **Deployed**: Exercise/assessment in use with students
- **Evaluated**: Exercise/assessment effectiveness assessed