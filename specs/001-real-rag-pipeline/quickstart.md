# Quickstart: Real RAG Pipeline

## Overview
This guide provides instructions for setting up and running the Real RAG Pipeline that integrates Cohere, Qdrant, and Gemini to provide textbook-based answers.

## Prerequisites
- Python 3.11+
- Access to Cohere API
- Access to Qdrant vector database with `rag_embedding` collection
- Access to Google Gemini API via OpenAI-compatible endpoint
- Environment with required dependencies installed

## Environment Setup

Create a `.env` file with the following variables:

```bash
# Qdrant Configuration
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=rag_embedding

# Cohere Configuration
COHERE_API_KEY=your_cohere_api_key

# Gemini Configuration
GEMINI_API_KEY=your_gemini_api_key

# Optional Configuration
RAG_AGENT_URL=optional_rag_agent_url
TARGET_WEBSITE_URL=optional_target_website_url
SITEMAP_URL=optional_sitemap_url
```

## Installation

1. Install required Python packages:
```bash
pip install fastapi cohere qdrant-client openai python-dotenv
```

2. Set up the project structure as defined in the implementation plan

## Running the Service

1. Start the FastAPI server:
```bash
uvicorn backend.main:app --reload
```

2. The RAG query endpoint will be available at:
   - POST `/api/rag/query`

## Testing the Endpoint

Example request:
```bash
curl -X POST http://localhost:8000/api/rag/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the key principles of humanoid robotics?",
    "max_chunks": 5,
    "similarity_threshold": 0.5
  }'
```

Expected response:
```json
{
  "answer": "Humanoid robotics is based on several key principles including...",
  "citations": [
    {
      "text": "[1] K. S. Fu, R. C. Gonzalez, and C. S. G. Lee, Robotics: Control, Sensing, Vision, and Intelligence. McGraw-Hill, 1987.",
      "source": "Fu et al. - Robotics Control Sensing Vision and Intelligence",
      "url": "https://example.com/textbook/chapter1"
    }
  ],
  "chunks_used": [
    {
      "id": "chunk_001",
      "content": "Humanoid robots are robots that are designed to look and/or behave like humans...",
      "metadata": {
        "source": "textbook/chapter1.md",
        "citation": "[1] K. S. Fu, R. C. Gonzalez, and C. S. G. Lee, Robotics: Control, Sensing, Vision, and Intelligence. McGraw-Hill, 1987."
      },
      "similarity_score": 0.85
    }
  ],
  "query": "What are the key principles of humanoid robotics?",
  "timestamp": "2025-12-19T10:30:00Z"
}
```

## Fallback Response

When no relevant context is found, the system returns exactly:
```json
{
  "answer": "I don't know based on the textbook.",
  "citations": [],
  "chunks_used": [],
  "query": "Your original query",
  "timestamp": "2025-12-19T10:30:00Z"
}
```

## Troubleshooting

1. **Connection Issues**: Verify all API keys and URLs in environment variables
2. **No Results**: Check that the Qdrant `rag_embedding` collection contains textbook content
3. **Performance**: Adjust `similarity_threshold` and `max_chunks` parameters as needed