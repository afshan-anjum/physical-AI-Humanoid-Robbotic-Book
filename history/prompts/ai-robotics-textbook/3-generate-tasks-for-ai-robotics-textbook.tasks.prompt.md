---
id: 3
title: Generate tasks for AI Robotics Textbook
stage: tasks
date: 2025-12-15
surface: agent
model: gemini-cli-agent
feature: ai-robotics-textbook
branch: main
user: unspecified
command: /sp.tasks
labels: [tasks, planning]
links:
  spec: specs/ai-robotics-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/ai-robotics-textbook/tasks.md
tests:
  - "none"
---

## Prompt

---
description: Generate an actionable, dependency-ordered tasks.md for the feature based on available design artifacts.
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

1. **Setup**: Run `.specify/scripts/powershell/check-prerequisites.ps1 -Json` from repo root and parse FEATURE_DIR and AVAILABLE_DOCS list. All paths must be absolute. For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\''m Groot' (or double-quote if possible: "I'm Groot").

2. **Load design documents**: Read from FEATURE_DIR:
   - **Required**: plan.md (tech stack, libraries, structure), spec.md (user stories with priorities)
   - **Optional**: data-model.md (entities), contracts/ (API endpoints), research.md (decisions), quickstart.md (test scenarios)
   - Note: Not all projects have all documents. Generate tasks based on what's available.

3. **Execute task generation workflow**:
   - Load plan.md and extract tech stack, libraries, project structure
   - Load spec.md and extract user stories with their priorities (P1, P2, P3, etc.)
   - If data-model.md exists: Extract entities and map to user stories
   - If contracts/ exists: Map endpoints to user stories
   - If research.md exists: Extract decisions for setup tasks
   - Generate tasks organized by user story (see Task Generation Rules below)
   - Generate dependency graph showing user story completion order
   - Create parallel execution examples per user story
   - Validate task completeness (each user story has all needed tasks, independently testable)

4. **Generate tasks.md**: Use `.specify.specify/templates/tasks-template.md` as structure, fill with:
   - Correct feature name from plan.md
   - Phase 1: Setup tasks (project initialization)
   - Phase 2: Foundational tasks (blocking prerequisites for all user stories)
   - Phase 3+: One phase per user story (in priority order from spec.md)
   - Each phase includes: story goal, independent test criteria, tests (if requested), implementation tasks
   - Final Phase: Polish & cross-cutting concerns
   - All tasks must follow the strict checklist format (see Task Generation Rules below)
   - Clear file paths for each task
   - Dependencies section showing story completion order
   - Parallel execution examples per story
   - Implementation strategy section (MVP first, incremental delivery)

5. **Report**: Output path to generated tasks.md and summary:
   - Total task count
   - Task count per user story
   - Parallel opportunities identified
   - Independent test criteria for each story
   - Suggested MVP scope (typically just User Story 1)
   - Format validation: Confirm ALL tasks follow the checklist format (checkbox, ID, labels, file paths)

## Response snapshot

I have successfully generated the `tasks.md` file.

**Report:**

*   **File Created**: `specs/ai-robotics-textbook/tasks.md`
*   **Total Tasks**: 33
*   **Tasks per User Story**:
    *   US1 (Intro): 1
    *   US2 (Navigation): 6
    *   US3 (Content): 6
    *   US4 (Search): 1
    *   US5 (Hardware): 2
*   **Parallel Opportunities**: Parallelizable tasks are marked with `[P]` and can be executed concurrently within their phases. User stories can be implemented in parallel after the Foundational phase is complete.
*   **Suggested MVP Scope**: The first MVP can be achieved by completing all P1 user stories: US1 (View Introduction), US2 (Navigate Course Structure), and US3 (Read Module Content).
*   **Format Validation**: All tasks strictly follow the required checklist format.

## Outcome

- ✅ Impact: A detailed, actionable task list was created, enabling the implementation phase.
- 🧪 Tests: Test creation tasks were included for custom components.
- 📁 Files: `specs/ai-robotics-textbook/tasks.md` was created.
- 🔁 Next prompts: `/sp.implement`
- 🧠 Reflection: The agent successfully synthesized multiple design documents into a coherent and structured task plan, following the specified format precisely.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): n/a
- Next experiment: n/a
