# Quickstart: RAG-enabled Agent using OpenAI Agents SDK + Qdrant Retrieval

## Prerequisites

- Python 3.11+
- OpenAI API key
- Qdrant instance with rag_embedding collection populated
- Required Python packages (see requirements below)

## Installation

1. Install required dependencies:
```bash
pip install openai qdrant-client python-dotenv
```

2. Set up environment variables:
```bash
# Create .env file with:
OPENAI_API_KEY=your_openai_api_key
QDRANT_URL=your_qdrant_instance_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=rag_embedding
```

## Usage

1. Initialize the agent:
```python
from backend.agent import RAGAgent

# Initialize the agent with configuration
agent = RAGAgent()
```

2. Query the agent:
```python
# Simple query
response = agent.invoke("Your question here")
print(response)
```

## Configuration

The agent can be configured with:
- Qdrant connection parameters (URL, API key, collection name)
- Embedding model selection
- Number of retrieved chunks (top-k)
- Similarity threshold for relevance

## Running Tests

```bash
cd G:\textbook
python -m pytest specs/001-rag-agent-openai-qdrant/tests/
```