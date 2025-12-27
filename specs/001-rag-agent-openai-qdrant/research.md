# Research: RAG-enabled Agent using OpenAI Agents SDK + Qdrant Retrieval

## Decision: OpenAI Agents SDK Implementation
**Rationale**: The specification requires using the OpenAI Agents SDK for orchestration. This provides a framework for creating agents that can use tools, which is perfect for our RAG implementation where we need a custom retrieval tool.

## Decision: Qdrant Integration
**Rationale**: The specification explicitly requires connecting to the Qdrant rag_embedding collection. Qdrant is a high-performance vector database that supports semantic search, making it ideal for RAG applications.

## Decision: Embedding Model Choice
**Rationale**: The spec mentions either Cohere or OpenAI embeddings, consistent with Spec-1. OpenAI embeddings (text-embedding-ada-002 or text-embedding-3-small/large) are chosen for consistency with OpenAI Agents SDK and for their proven performance in RAG applications.

## Decision: Custom Retrieval Tool Design
**Rationale**: The OpenAI Agents SDK allows for custom tools. We'll create a retrieval tool that takes user queries, embeds them, searches Qdrant, and returns relevant chunks as context for the agent.

## Decision: Architecture Pattern
**Rationale**: Following a service-oriented architecture with separate modules for embedding, retrieval, and agent orchestration to maintain clean separation of concerns and testability.

## Alternatives Considered

1. **Alternative Vector Databases**:
   - Pinecone: Commercial alternative but Qdrant was chosen as it's open-source and specifically mentioned in requirements
   - Weaviate: Another open-source option but Qdrant was specified in requirements

2. **Alternative Agent Frameworks**:
   - LangChain: More complex but OpenAI Agents SDK was specifically required
   - CrewAI: Alternative agent framework but not specified in requirements

3. **Alternative Embedding Models**:
   - Sentence Transformers: Open-source but OpenAI embeddings were mentioned in requirements
   - Cohere embeddings: Also an option but OpenAI was chosen for consistency with OpenAI Agents SDK

## Technical Requirements Identified

1. **Qdrant Connection Parameters**:
   - URL endpoint for Qdrant instance
   - API key for authentication
   - Collection name (rag_embedding as specified)

2. **Embedding Process**:
   - Convert user query text to embedding vector
   - Use same embedding model that was used to create the rag_embedding collection

3. **Similarity Search**:
   - Perform vector similarity search in Qdrant
   - Retrieve top-k most relevant document chunks
   - Handle cases where no relevant documents are found

4. **Agent Orchestration**:
   - Initialize OpenAI Agent with custom retrieval tool
   - Process user queries through the RAG pipeline
   - Ensure responses are grounded only in retrieved content
   - Provide single agent.invoke() entrypoint as required

## Implementation Approach

The agent will be implemented as a single agent.py file that:
1. Initializes an OpenAI Agent with a custom retrieval tool
2. Connects the retrieval tool to Qdrant rag_embedding collection
3. Embeds user queries and runs similarity search against Qdrant
4. Passes retrieved chunks as context for grounded answering
5. Provides the agent.invoke() interface as specified