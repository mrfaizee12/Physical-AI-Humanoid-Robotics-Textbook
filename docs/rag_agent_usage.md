# RAG Agent Usage Guide

## Overview
The RAG (Retrieval-Augmented Generation) Agent uses OpenAI's API and Qdrant vector database to answer user questions by retrieving relevant information and generating grounded responses.

## Prerequisites
- Python 3.11+
- OpenAI API key
- Qdrant database with rag_embedding collection populated

## Setup
1. Install dependencies: `pip install -r backend/requirements.txt`
2. Set environment variables in `.env` file:
   ```bash
   OPENAI_API_KEY=your_openai_api_key_here
   QDRANT_URL=your_qdrant_instance_url_here
   QDRANT_API_KEY=your_qdrant_api_key_here
   QDRANT_COLLECTION_NAME=rag_embedding
   ```

## Quick Start
```python
from backend.agent import RAGAgent

# Initialize the agent
agent = RAGAgent(top_k=5, min_score=0.3)

# Query the agent
response = await agent.invoke("Your question here")
print(response)
```

## Configuration
- `top_k`: Number of document chunks to retrieve (default: 5)
- `min_score`: Minimum similarity score for retrieved chunks (default: 0.3)

## Architecture
- **Agent Layer**: Main interface with `invoke()` method
- **Service Layer**: Qdrant retrieval service
- **Utility Layer**: Embedding utilities
- **Model Layer**: Data models for queries, chunks, and responses