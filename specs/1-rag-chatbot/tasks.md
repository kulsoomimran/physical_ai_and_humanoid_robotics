---
description: "Task list for Embedded RAG Chatbot feature"
---

# Tasks: Embedded RAG Chatbot

**Input**: Design documents from `/specs/1-rag-chatbot/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `backend/src/`, `backend/tests/`
- **Frontend**: `frontend/src/`, `frontend/public/`
- Paths shown below follow the planned structure from plan.md

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create backend project structure with requirements.txt
- [X] T002 [P] Create frontend project structure for Docusaurus integration
- [X] T003 [P] Set up git repository with proper .gitignore for Python

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T004 Set up Neon Postgres database schema and migration framework
- [X] T005 [P] Configure Qdrant Cloud connection and collection setup
- [X] T006 [P] Set up FastAPI application structure with configuration management
- [X] T007 Create base models/entities that all stories depend on
- [X] T008 Configure error handling and logging infrastructure
- [X] T009 [P] Set up environment configuration for Gemini API access
- [X] T010 [P] Set up context7 MCP server integration for orchestration

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Query Book Content via RAG Chatbot (Priority: P1) 🎯 MVP

**Goal**: Enable users to ask questions about the Physical AI & Humanoid Robotics book content and receive accurate, contextually relevant answers through an integrated chatbot interface

**Independent Test**: Can be fully tested by asking questions about book content and verifying that the chatbot provides accurate answers based on the book's information

### Tests for User Story 1 (OPTIONAL - only if tests requested) ⚠️

- [X] T011 [P] [US1] Contract test for /chat/start endpoint in backend/tests/contract/test_chat.py
- [X] T012 [P] [US1] Contract test for /chat/{session_id}/query endpoint in backend/tests/contract/test_chat.py
- [X] T013 [P] [US1] Integration test for book content query flow in backend/tests/integration/test_rag.py

### Implementation for User Story 1

- [X] T014 [P] [US1] Create Conversation model in backend/src/models/conversation.py
- [X] T015 [P] [US1] Create Query model in backend/src/models/query.py
- [X] T016 [P] [US1] Create Response model in backend/src/models/response.py
- [X] T017 [P] [US1] Create SourceCitation model in backend/src/models/source_citation.py
- [X] T018 [US1] Implement ConversationService in backend/src/services/conversation_service.py
- [X] T019 [US1] Implement RAGService for book content retrieval in backend/src/services/rag_service.py
- [X] T020 [US1] Implement ChatAPI endpoints in backend/src/api/chat.py (depends on T018, T019)
- [X] T021 [US1] Set up document ingestion pipeline for book content
- [X] T022 [US1] Implement embedding generation for book content chunks
- [X] T023 [US1] Integrate with Qdrant for vector storage of book content
- [X] T024 [US1] Implement response generation using Gemini API
- [X] T025 [US1] Add source citation functionality to responses
- [X] T026 [US1] Add logging for user story 1 operations

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Query User-Selected Text via RAG Chatbot (Priority: P2)

**Goal**: Enable users to select text from the book or paste their own content and ask questions specifically about that text

**Independent Test**: Can be tested by selecting text in the book or pasting external content and asking questions about it

### Tests for User Story 2 (OPTIONAL - only if tests requested) ⚠️

- [ ] T027 [P] [US2] Contract test for document chunking endpoint in backend/tests/contract/test_documents.py
- [ ] T028 [P] [US2] Integration test for user-provided text query flow in backend/tests/integration/test_user_text.py

### Implementation for User Story 2

- [X] T029 [P] [US2] Create DocumentChunk model in backend/src/models/document_chunk.py
- [X] T030 [P] [US2] Create EmbeddingVector model in backend/src/models/embedding_vector.py
- [X] T031 [US2] Implement DocumentChunkService in backend/src/services/document_chunk_service.py
- [X] T032 [US2] Enhance RAGService to handle user-provided text in backend/src/services/rag_service.py
- [X] T033 [US2] Update ChatAPI to support context_mode parameter in backend/src/api/chat.py
- [X] T034 [US2] Implement document chunking functionality for user text
- [X] T035 [US2] Add Qdrant integration for temporary user text vectors
- [X] T036 [US2] Update frontend to support text selection and paste functionality
- [X] T037 [US2] Add validation for user-provided text inputs

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Interactive Learning with Contextual Explanations (Priority: P3)

**Goal**: Enable users to ask for explanations of complex concepts from the book in different ways (simplified, detailed, examples)

**Independent Test**: Can be tested by asking for explanations of complex concepts in different ways

### Tests for User Story 3 (OPTIONAL - only if tests requested) ⚠️

- [ ] T038 [P] [US3] Integration test for response customization in backend/tests/integration/test_explanations.py

### Implementation for User Story 3

- [X] T039 [P] [US3] Enhance Query model with user_preferences in backend/src/models/query.py
- [X] T040 [US3] Update RAGService to support different response styles in backend/src/services/rag_service.py
- [X] T041 [US3] Implement response customization based on user preferences
- [X] T042 [US3] Add example extraction functionality from book content
- [X] T043 [US3] Update Gemini API integration to generate different response types
- [X] T044 [US3] Update frontend UI to allow response style selection

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Chatbot UI Integration with Docusaurus Book

**Goal**: Embed the chatbot interface within the Docusaurus book pages as specified in functional requirement FR-001

- [X] T045 Create chatbot UI component using ChatKit SDKs in content/src/components/Chatbot.jsx
- [X] T046 Integrate chatbot component into Docusaurus theme
- [X] T047 Add CSS styling for chatbot interface to match book design
- [X] T048 Implement text selection functionality in book pages
- [X] T049 Connect frontend to backend API endpoints
- [X] T050 Add loading states and error handling to UI

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T051 [P] Documentation updates in docs/
- [ ] T052 Code cleanup and refactoring
- [ ] T053 Performance optimization across all stories
- [ ] T054 [P] Additional unit tests in backend/tests/unit/
- [ ] T055 Security hardening
- [ ] T056 Run quickstart.md validation
- [ ] T057 Set up monitoring and observability
- [ ] T058 Configure deployment pipeline

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **UI Integration (Phase 6)**: Can proceed in parallel with user stories once backend API is available
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tests for a user story marked [P] can run in parallel
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

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
5. Add UI Integration → Test → Deploy/Demo
6. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
   - Developer D: UI Integration
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