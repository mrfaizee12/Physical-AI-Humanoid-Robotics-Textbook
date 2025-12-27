# Research: Real RAG Pipeline Implementation

## Overview
This research document addresses the technical requirements for implementing the real RAG pipeline with Cohere, Qdrant, and Gemini integration.

## Decision: Qdrant Collection Verification
**Rationale**: Before implementing the RAG pipeline, we need to verify that textbook chunks exist in the Qdrant collection named `rag_embedding`.
**Implementation**: Will implement a verification function that connects to Qdrant and checks for the existence of the collection and its content.
**Alternatives considered**:
- Assume collection exists (risky approach that could lead to runtime errors)
- Create collection if missing (not appropriate for textbook content which should be pre-ingested)

## Decision: Cohere Embedding Integration
**Rationale**: Using Cohere's embedding API to convert user queries into vector representations for semantic search.
**Implementation**: Will use Cohere's Python SDK to generate embeddings, handling API errors and rate limiting appropriately.
**Alternatives considered**:
- OpenAI embeddings (already specified as Cohere in requirements)
- Sentence Transformers (local option but less accurate than Cohere)
- Hugging Face embeddings (local option but would require model management)

## Decision: Qdrant Vector Search Implementation
**Rationale**: Using Qdrant for efficient semantic search of textbook chunks using the `rag_embedding` collection.
**Implementation**: Will implement vector search with appropriate similarity thresholds and result limits to balance relevance and performance.
**Alternatives considered**:
- Pinecone (cloud-based alternative but not specified in requirements)
- Weaviate (alternative vector database but Qdrant already specified)
- Elasticsearch (traditional search but less optimal for semantic search)

## Decision: Gemini API Integration via OpenAI-Compatible SDK
**Rationale**: Using the OpenAI-compatible SDK to connect to Gemini with the specified base URL and model.
**Implementation**: Will configure the OpenAI SDK with the specific endpoint `https://generativelanguage.googleapis.com/v1beta/openai/` and model `gemini-2.0-flash`.
**Alternatives considered**:
- Direct HTTP calls (more complex but not necessary with SDK)
- Gemini-specific SDK (OpenAI-compatible SDK is sufficient and already supports this endpoint)

## Decision: Response Schema Preservation
**Rationale**: Maintaining the existing response schema to ensure backward compatibility with frontend applications.
**Implementation**: Will structure the RAG response to match the existing schema while adding the real textbook-based content.
**Alternatives considered**:
- New response schema (would break existing frontend implementations)

## Decision: Citation Handling
**Rationale**: Including proper citations in the response to meet the content accuracy requirements from the constitution.
**Implementation**: Will extract citation information from the Qdrant metadata and format it according to IEEE standards.
**Alternatives considered**:
- No citations (violates constitution requirement)
- Simplified citations (may not meet academic standards)

## Decision: No-Match Fallback Implementation
**Rationale**: Handling cases where no relevant context is found in the textbook.
**Implementation**: Will return the exact string "I don't know based on the textbook." as specified in requirements.
**Alternatives considered**:
- Generic error message (specific message required by spec)
- Alternative fallback responses (exact string required by spec)

## Technical Best Practices
1. **Environment Variable Management**: All API keys (QDRANT_API_KEY, COHERE_API_KEY, GEMINI_API_KEY) will be loaded from environment variables, never hardcoded.
2. **Error Handling**: Comprehensive error handling for network failures, API rate limits, and service unavailability.
3. **Performance Optimization**: Caching of embeddings where appropriate and efficient vector search parameters.
4. **Security**: Proper validation of inputs to prevent injection attacks.