# Implementation Plan: Real RAG Pipeline

**Branch**: `001-real-rag-pipeline` | **Date**: 2025-12-19 | **Spec**: G:\textbook\specs\001-real-rag-pipeline\spec.md
**Input**: Feature specification from `/specs/001-real-rag-pipeline/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Enable the real RAG pipeline by implementing a complete end-to-end system that removes mock responses and activates Qdrant + Cohere + Gemini integration. The system will create embeddings with Cohere, retrieve relevant textbook chunks from Qdrant, send context to Gemini via OpenAI-compatible SDK, and return real answers with citations while maintaining the existing response schema.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: FastAPI, Cohere SDK, Qdrant client, OpenAI-compatible SDK for Gemini
**Storage**: Qdrant vector database for textbook chunks, environment variables for credentials
**Testing**: pytest for unit and integration tests
**Target Platform**: Linux server
**Project Type**: Web - backend API service
**Performance Goals**: <500ms response time for RAG queries, handle 100 concurrent users
**Constraints**: Must maintain existing response schema, no hardcoded secrets, secure API key handling
**Scale/Scope**: 10k textbook queries per day, 1M+ text chunks in vector store

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

1. **Content Accuracy**: RAG responses must be grounded in official textbook content with proper citations (PASSED - requirement in spec)
2. **Instructional Clarity**: System must provide clear, educational responses suitable for AI/robotics learners (PASSED - requirement in spec)
3. **Complete Reproducibility**: Implementation must be fully reproducible with proper documentation (PASSED - will document in quickstart.md)
4. **Content Consistency**: RAG system must maintain synchronization with textbook content (PASSED - using existing Qdrant collection)
5. **Source Code and Citations**: Responses must include IEEE-style citations to source material (PASSED - requirement in spec)
6. **Chatbot Requirements**: Responses must be strictly derived from textbook content (PASSED - requirement in spec)

## Project Structure

### Documentation (this feature)

```text
specs/001-real-rag-pipeline/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

backend/
├── src/
│   ├── models/
│   │   ├── rag_models.py
│   │   └── response_models.py
│   ├── services/
│   │   ├── rag_service.py
│   │   ├── embedding_service.py
│   │   └── qdrant_service.py
│   └── api/
│       └── rag_routes.py
└── tests/
    ├── unit/
    │   ├── test_embedding_service.py
    │   └── test_rag_service.py
    └── integration/
        └── test_rag_integration.py

**Structure Decision**: Backend API service structure selected to implement RAG functionality. The implementation will be added to the existing backend structure in the repository, with models for RAG data, services for embedding/Qdrant/Gemini integration, and API routes for the RAG endpoint.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| N/A | N/A | N/A |
