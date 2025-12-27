# Feature Specification: Real RAG Pipeline

**Feature Branch**: `001-real-rag-pipeline`
**Created**: 2025-12-19
**Status**: Draft
**Input**: User description: "Enable the **real RAG pipeline** so the chatbot returns **actual textbook-based answers**.

**Goal**
Remove all mock responses and activate Qdrant + Cohere + Gemini end-to-end.

**Requirements**
- `/api/rag/query` must:
  1. Create embeddings with **Cohere**
  2. Retrieve chunks from **Qdrant** (`rag_embedding`)
  3. Send context to **Gemini** via OpenAI-compatible SDK
  4. Return real answers + citations
- If no context found, return exactly:
  **\"I don't know based on the textbook.\"**
- **Do not change** response schema.

**LLM (MANDATORY)**
- Base URL: `https://generativelanguage.googleapis.com/v1beta/openai/`
- Model: `gemini-2.0-flash`
- API key: `GEMINI_API_KEY` (env)

**Env (do not hardcode)**
`QDRANT_URL`, `QDRANT_API_KEY`, `QDRANT_COLLECTION_NAME=rag_embedding`,
`COHERE_API_KEY`, `RAG_AGENT_URL`, `TARGET_WEBSITE_URL`, `SITEMAP_URL`"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Query textbook content with RAG (Priority: P1)

As a user, I want to ask questions about the textbook content and receive accurate, contextually relevant answers that are based on the actual textbook material, with proper citations to the source material.

**Why this priority**: This is the core functionality that transforms the chatbot from mock responses to real textbook-based answers, providing the primary value proposition of the feature.

**Independent Test**: Can be fully tested by sending a query to the RAG endpoint and verifying that the response contains actual textbook content with citations rather than mock responses, delivering real educational value.

**Acceptance Scenarios**:

1. **Given** a valid question about textbook content, **When** user submits query to `/api/rag/query`, **Then** response contains actual textbook-based answer with citations
2. **Given** a question that has no relevant context in the textbook, **When** user submits query to `/api/rag/query`, **Then** response returns exactly "I don't know based on the textbook."

---

### User Story 2 - Maintain existing response format (Priority: P2)

As a frontend developer, I want the RAG endpoint to maintain the same response schema so that existing client applications continue to work without modifications.

**Why this priority**: Ensures backward compatibility with existing frontend implementations and prevents breaking changes to dependent systems.

**Independent Test**: Can be fully tested by comparing the response structure of the new RAG implementation with the previous mock implementation to verify schema consistency.

**Acceptance Scenarios**:

1. **Given** any RAG query, **When** response is generated, **Then** response schema matches the existing format without changes

---

### User Story 3 - Secure access to external services (Priority: P3)

As a system administrator, I want the RAG pipeline to securely connect to external services (Qdrant, Cohere, Gemini) using environment-provided credentials without hardcoding secrets.

**Why this priority**: Ensures proper security practices are followed and system can be deployed across different environments with appropriate credential management.

**Independent Test**: Can be fully tested by verifying that external service connections use environment variables for authentication and no credentials are hardcoded in the source code.

**Acceptance Scenarios**:

1. **Given** configured environment variables, **When** RAG pipeline connects to external services, **Then** connections use credentials from environment variables
2. **Given** missing environment variables, **When** RAG pipeline attempts to connect, **Then** appropriate error handling occurs

---

### Edge Cases

- What happens when external services (Qdrant, Cohere, Gemini) are unavailable or return errors?
- How does the system handle queries with very long input text that might exceed API limits?
- What happens when the Qdrant collection is empty or contains no relevant content for the query?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST create text embeddings using Cohere API for user queries
- **FR-002**: System MUST retrieve relevant text chunks from Qdrant vector database using the `rag_embedding` collection
- **FR-003**: System MUST send retrieved context and user query to Gemini LLM via OpenAI-compatible SDK
- **FR-004**: System MUST return textbook-based answers with proper citations to the source material
- **FR-005**: System MUST return exactly "I don't know based on the textbook." when no relevant context is found
- **FR-006**: System MUST maintain the existing response schema without changes
- **FR-007**: System MUST use environment variables for all external service credentials (QDRANT_URL, QDRANT_API_KEY, COHERE_API_KEY, GEMINI_API_KEY)
- **FR-008**: System MUST connect to Gemini using the base URL `https://generativelanguage.googleapis.com/v1beta/openai/` and model `gemini-2.0-flash`

### Key Entities

- **Query**: User input question that requires textbook-based response
- **Embedding**: Vector representation of user query for similarity search
- **Text Chunk**: Relevant segment of textbook content retrieved from Qdrant
- **Response**: Structured answer containing textbook-based content and citations

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 100% of queries return actual textbook-based answers instead of mock responses when relevant content exists
- **SC-002**: Queries with no relevant textbook context return exactly "I don't know based on the textbook." 100% of the time
- **SC-003**: Response schema remains unchanged, ensuring 100% backward compatibility with existing frontend implementations
- **SC-004**: All external service connections use environment variables for credentials with 0 hardcoded secrets
- **SC-005**: Users can receive accurate answers to textbook-related questions with proper citations
