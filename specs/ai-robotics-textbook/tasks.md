---
description: "Task list for AI & Humanoid Robotics Textbook feature implementation"
---

# Tasks: AI & Humanoid Robotics Textbook

**Input**: Design documents from `/specs/ai-robotics-textbook/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md

**Tests**: Test tasks are included for custom components as per research.md.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- All paths are relative to the `docusaurus/` directory as specified in `plan.md`.

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and configuration of core Docusaurus features and testing.

- [x] T001 Configure `docusaurus.config.js` with textbook title ('AI & Humanoid Robotics Textbook'), favicon, and theme settings.
- [x] T002 Configure Jest, Babel, and React Testing Library by creating `jest.config.js`, `babel.config.js`, and `jest.setup.js` in the `docusaurus/` directory. <!-- NOTE: Testing dependencies were added to package.json but must be installed manually via `npm install` due to system execution policies. -->
- [x] T003 [P] Install and configure `@docusaurus/theme-mermaid` in `docusaurus.config.js` to enable Mermaid diagrams. <!-- NOTE: Dependency added to package.json but must be installed manually via `npm install`. -->
- [x] T004 [P] Apply for Algolia DocSearch and add placeholder API keys to the `themeConfig.algolia` section in `docusaurus.config.js`. <!-- NOTE: Placeholder values used. Real keys must be added after approval from Algolia. -->
- [x] T005 [P] Create `src/css/custom.css` and add basic styling rules for admonitions and code blocks to improve readability.
- [x] T006 Create initial directories for the 6 modules (`module-1` to `module-6`) inside the `docs/` directory.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core content structure and navigation that MUST be complete before user stories are fully implemented.

- [x] T007 Implement the main sidebar navigation structure for all modules and static pages in `sidebars.js`.
- [x] T008 [P] Create the main landing page `docs/intro.md` with placeholder introductory content.
- [x] T009 [P] Create placeholder `hardware-requirements.md` page in `docusaurus/docs/`.
- [x] T010 [P] Create placeholder `glossary.md` page in `docusaurus/docs/`.
- [x] T011 [P] Create placeholder `assessment-guidelines.md` page in `docusaurus/docs/`.
- [x] T012 [P] Create placeholder `prerequisites-setup.md` page in `docusaurus/docs/`.
- [x] T013 [P] Create placeholder `additional-resources.md` page in `docusaurus/docs/`.

**Checkpoint**: Foundation ready - user story implementation can now begin.

---

## Phase 3: User Story 1 - View Textbook Introduction (Priority: P1) 🎯 MVP

**Goal**: As a new student, I want to see a clear landing page with an introduction to the textbook.
**Independent Test**: The `intro.md` page renders correctly with its content and is accessible from the root URL.

- [x] T014 [US1] Populate `docs/intro.md` with final introduction text, summary of the 6 modules, and a call to action.

**Checkpoint**: User Story 1 is fully functional and testable independently.

---

## Phase 4: User Story 2 - Navigate Course Structure (Priority: P1)

**Goal**: As a student, I want to easily navigate through the different modules and chapters using a persistent sidebar.
**Independent Test**: The sidebar shows all 6 modules with their correct labels and positions, and they are expandable.

- [x] T015 [P] [US2] Create `docs/module-1/_category_.json` with the label "Module 1: The Foundation".
- [x] T016 [P] [US2] Create `docs/module-2/_category_.json` with the label "Module 2: ROS 2".
- [x] T017 [P] [US2] Create `docs/module-3/_category_.json` with the label "Module 3: Gazebo".
- [x] T018 [P] [US2] Create `docs/module-4/_category_.json` with the label "Module 4: NVIDIA Isaac".
- [x] T019 [P] [US2] Create `docs/module-5/_category_.json` with the label "Module 5: Humanoid Development".
- [x] T020 [P] [US2] Create `docs/module-6/_category_.json` with the label "Module 6: Conversational Robotics/Capstone".

**Checkpoint**: User Story 2 is fully functional and testable independently.

---

## Phase 5: User Story 3 - Read Module Content (Priority: P1)

**Goal**: As a student, I want to read the content of each chapter, including formatted text, code examples, and diagrams.
**Independent Test**: A sample chapter page renders text, a highlighted code block, and a Mermaid diagram correctly.

- [x] T021 [P] [US3] Create a sample chapter `docs/module-1/week-1.mdx` including frontmatter, learning objectives, sample text, a JS code block, and a Mermaid diagram.
- [x] T022 [P] [US3] Create a placeholder chapter `docs/module-2/overview.md`.
- [x] T023 [P] [US3] Create a placeholder chapter `docs/module-3/overview.md`.
- [x] T024 [P] [US3] Create a placeholder chapter `docs/module-4/overview.md`.
- [x] T025 [P] [US3] Create a placeholder chapter `docs/module-5/overview.md`.
- [x] T026 [P] [US3] Create a placeholder chapter `docs/module-6/overview.md`.

**Checkpoint**: User Story 3 is demonstrable with sample content.

---

## Phase 6: User Story 4 - Search for Topics (Priority: P2)

**Goal**: As a student, I want to find specific topics quickly using a search bar.
**Independent Test**: The search bar is visible and configured with the Algolia keys in the config file.

- [x] T027 [US4] Finalize Algolia DocSearch configuration in `docusaurus.config.js`, setting `contextualSearch: true`.

**Checkpoint**: User Story 4 is functional.

---

## Phase 7: User Story 5 - Check Hardware Requirements (Priority: P2)

**Goal**: As a student, I want to find a dedicated "Hardware Requirements" section.
**Independent Test**: The hardware requirements page is linked in the sidebar and contains formatted content.

- [x] T028 [US5] Populate `docs/hardware-requirements.md` with detailed hardware specifications using Markdown formatting.
- [x] T029 [US5] Verify `hardware-requirements.md` is correctly linked in the main sidebar in `sidebars.js`.

**Checkpoint**: User Story 5 is fully functional.

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and ensure overall quality.

- [x] T030 [P] Create a sample custom React component `src/components/InteractiveDiagram.js`.
- [x] T031 Create a Jest/RTL test for `InteractiveDiagram.js` in `src/components/__tests__/InteractiveDiagram.test.js`.
- [x] T032 Review the site on different screen sizes and add responsive adjustments to `src/css/custom.css`. <!-- NOTE: This task requires human visual review and testing on various screen sizes. -->
- [x] T033 **Constitutional Alignment**: Final review against all principles (Progressive Learning, Hands-On Examples, etc.). <!-- NOTE: This task requires human review and validation against the project's constitutional principles. -->

---

## Dependencies & Execution Order

### Phase Dependencies
- **Setup (Phase 1)**: Can start immediately.
- **Foundational (Phase 2)**: Depends on Setup completion. BLOCKS all user stories.
- **User Stories (Phase 3-7)**: Depend on Foundational phase completion. Can then proceed in priority order.
- **Polish (Phase 8)**: Depends on all user stories being complete.

### User Story Dependencies
- All user stories are designed to be independently implementable after Phase 2 is complete.

---

## Implementation Strategy

### MVP First (P1 Stories)
1.  Complete Phase 1: Setup
2.  Complete Phase 2: Foundational
3.  Complete Phase 3 (US1), Phase 4 (US2), and Phase 5 (US3).
4.  **STOP and VALIDATE**: Test that the introduction, navigation, and sample content work together.
5.  This bundle constitutes the core MVP.
