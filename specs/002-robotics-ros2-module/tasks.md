# Tasks: Module 1: Robotic Nervous System (ROS 2)

**Input**: Design documents from `/specs/002-robotics-ros2-module/`
**Prerequisites**: plan.md, spec.md

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Create the basic directory structure for the new module.

- [X] T001 Create the directory for the new module at `book/docs/01-ros2-basics/`
- [X] T002 Create and configure the sidebar category file at `book/docs/01-ros2-basics/_category_.json`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Ensure the Docusaurus project is configured to support the required features.

- [X] T003 Check `book/docusaurus.config.ts` and ensure the `@docusaurus/theme-mermaid` is enabled. Add it if it's missing.

---

## Phase 3: User Story 1 - Understand ROS 2 Communication (Priority: P1) 🎯 MVP

**Goal**: Create the first chapter explaining the core communication concepts of ROS 2.

**Independent Test**: The content for this chapter renders correctly, the Mermaid diagram is visible, and the page builds successfully.

### Implementation for User Story 1

- [X] T004 [US1] Create the markdown file for the first chapter at `book/docs/01-ros2-basics/01-communication.md`
- [X] T005 [US1] Write the introductory content explaining ROS 2 Nodes, Topics, and Services in `book/docs/01-ros2-basics/01-communication.md`
- [X] T006 [US1] Add a Mermaid diagram to `book/docs/01-ros2-basics/01-communication.md` to illustrate the publisher/subscriber model.

**Checkpoint**: Chapter 1 is drafted and renders correctly on the local development server.

---

## Phase 4: User Story 2 - Control a Robot Actuator (Priority: P2)

**Goal**: Create the second chapter, which provides a practical Python example for controlling an actuator.

**Independent Test**: The content for this chapter renders correctly, and the Python code block is formatted properly.

### Implementation for User Story 2

- [X] T007 [US2] Create the markdown file for the second chapter at `book/docs/01-ros2-basics/02-rclpy-control.md`
- [X] T008 [US2] Write the content explaining `rclpy` and the publisher/subscriber pattern in `book/docs/01-ros2-basics/02-rclpy-control.md`
- [X] T009 [US2] Add the example Python `rclpy` publisher script as a code block in `book/docs/01-ros2-basics/02-rclpy-control.md`

**Checkpoint**: Chapter 2 is drafted and can be viewed alongside Chapter 1.

---

## Phase 5: User Story 3 - Define a Humanoid Model (Priority: P3)

**Goal**: Create the third chapter, explaining how to define a robot's structure using URDF.

**Independent Test**: The content for this chapter renders correctly, and the URDF XML code block is valid.

### Implementation for User Story 3

- [X] T010 [US3] Create the markdown file for the third chapter at `book/docs/01-ros2-basics/03-urdf-basics.md`
- [X] T011 [US3] Write the content explaining URDF, links, and joints in `book/docs/01-ros2-basics/03-urdf-basics.md`
- [X] T012 [US3] Add the example URDF file content as an XML code block in `book/docs/01-ros2-basics/03-urdf-basics.md`

**Checkpoint**: All three chapters are drafted.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Finalize the module by ensuring quality, consistency, and build integrity.

- [X] T013 [P] Review content in all three new markdown files for technical accuracy, spelling, and grammar.
- [X] T013a Install the @docusaurus/theme-mermaid package in the `book/` directory.
- [ ] T014 Run the production build command `npm run build` from the `book/` directory to ensure the new module integrates without errors.
- [X] T015 [P] Validate that all diagrams render correctly and code blocks are properly formatted in the final build.

---

## Dependencies & Execution Order

### Phase Dependencies
- **Setup (Phase 1)** is the starting point.
- **Foundational (Phase 2)** depends on Setup.
- **User Stories (Phases 3-5)** depend on Foundational.
- **Polish (Phase 6)** depends on all user stories being complete.

### User Story Dependencies
- **User Story 1 (P1)**: No dependencies on other stories.
- **User Story 2 (P2)**: Logically follows US1, but tasks can be worked on in parallel.
- **User Story 3 (P3)**: Logically follows US1/US2, but tasks can be worked on in parallel.

The user stories for this feature are chapters in a book, so while they have a logical reading order, their creation can happen in parallel.

### Parallel Opportunities
- After Phase 2, work on all three user stories (chapters) can happen in parallel.
- T013 (review) and T015 (validation) can be done in parallel with other polish tasks.

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1.  Complete Phase 1: Setup.
2.  Complete Phase 2: Foundational.
3.  Complete Phase 3: User Story 1.
4.  **STOP and VALIDATE**: Run `npm run start` in `book/` and verify Chapter 1 renders correctly. This is the MVP.

### Incremental Delivery

1.  Complete and validate the MVP (US1).
2.  Add User Story 2 (Chapter 2). Validate that both chapters render correctly.
3.  Add User Story 3 (Chapter 3). Validate that all three chapters render correctly.
4.  Complete the Polish phase to finalize the module.
