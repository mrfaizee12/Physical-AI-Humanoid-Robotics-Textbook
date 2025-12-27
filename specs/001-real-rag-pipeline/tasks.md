---
description: "Task list for Real RAG Pipeline implementation"
---

# Tasks: Real RAG Pipeline

**Input**: Design documents from `/specs/001-real-rag-pipeline/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `frontend/src/`
- **Paths shown below assume the web backend structure from plan.md**

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create backend project structure per implementation plan in backend/
- [x] T002 Initialize Python 3.11 project with FastAPI, Cohere SDK, Qdrant client, OpenAI-compatible SDK dependencies in backend/requirements.txt
- [x] T003 [P] Configure environment variables management with python-dotenv in backend/.env.example

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 Setup configuration management for API keys in backend/src/config.py
- [x] T005 [P] Implement Qdrant client connection in backend/src/services/qdrant_service.py
- [x] T006 [P] Setup Cohere client connection in backend/src/services/embedding_service.py
- [x] T007 Create Gemini client connection in backend/src/services/llm_service.py
- [x] T008 Configure error handling and logging infrastructure in backend/src/utils/
- [x] T009 Setup response models based on data model in backend/src/models/response_models.py

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Query textbook content with RAG (Priority: P1) 🎯 MVP

**Goal**: Enable users to ask questions about textbook content and receive accurate, contextually relevant answers based on actual textbook material with proper citations.

**Independent Test**: Can be fully tested by sending a query to the RAG endpoint and verifying that the response contains actual textbook content with citations rather than mock responses, delivering real educational value.

### Implementation for User Story 1

- [x] T010 [P] [US1] Create RAG models in backend/src/models/rag_models.py (Query, TextChunk, RAGResponse, ErrorResponse)
- [x] T011 [US1] Implement Qdrant service with text chunk retrieval in backend/src/services/qdrant_service.py
- [x] T012 [US1] Implement embedding service with Cohere integration in backend/src/services/embedding_service.py
- [x] T013 [US1] Implement RAG orchestration service in backend/src/services/rag_service.py
- [x] T014 [US1] Create RAG API endpoint at /api/rag/query in backend/src/api/rag_routes.py
- [x] T015 [US1] Add validation rules for Query model (text length, similarity threshold, max_chunks)
- [x] T016 [US1] Implement fallback response for no-match scenarios ("I don't know based on the textbook.")

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Maintain existing response format (Priority: P2)

**Goal**: Ensure the RAG endpoint maintains the same response schema so that existing client applications continue to work without modifications.

**Independent Test**: Can be fully tested by comparing the response structure of the new RAG implementation with the previous mock implementation to verify schema consistency.

### Implementation for User Story 2

- [x] T017 [P] [US2] Verify response schema matches existing format in backend/src/models/response_models.py
- [x] T018 [US2] Update RAG service to maintain exact response structure in backend/src/services/rag_service.py
- [x] T019 [US2] Test response schema compatibility with existing frontend expectations

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Secure access to external services (Priority: P3)

**Goal**: Ensure the RAG pipeline connects to external services (Qdrant, Cohere, Gemini) using environment-provided credentials without hardcoding secrets.

**Independent Test**: Can be fully tested by verifying that external service connections use environment variables for authentication and no credentials are hardcoded in the source code.

### Implementation for User Story 3

- [x] T020 [P] [US3] Verify all API keys loaded from environment variables in backend/src/config.py
- [x] T021 [US3] Implement secure credential handling in backend/src/services/
- [x] T022 [US3] Add validation for missing environment variables with appropriate error handling

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T023 [P] Documentation updates for RAG pipeline in backend/docs/
- [ ] T024 Code cleanup and refactoring across all services
- [ ] T025 Performance optimization for RAG query response time
- [ ] T026 [P] Add unit tests for all services in backend/tests/unit/
- [ ] T027 Security hardening for API endpoints
- [ ] T028 Run quickstart.md validation to ensure complete functionality

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each User Story

- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all models for User Story 1 together:
Task: "Create RAG models in backend/src/models/rag_models.py (Query, TextChunk, RAGResponse, ErrorResponse)"
Task: "Implement Qdrant service with text chunk retrieval in backend/src/services/qdrant_service.py"
Task: "Implement embedding service with Cohere integration in backend/src/services/embedding_service.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence