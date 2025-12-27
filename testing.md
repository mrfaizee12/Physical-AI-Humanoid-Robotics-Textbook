# RAG Pipeline Testing Instructions

## Setup Requirements
1. Ensure all environment variables are set in `.env`:
   ```
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_URL=your_qdrant_url
   COHERE_API_KEY=your_cohere_api_key
   OPENROUTER_API_KEY=your_openrouter_api_key
   TARGET_WEBSITE_URL=https://physical-ai-humanoid-robotics-textb-nu.vercel.app/
   SITEMAP_URL=https://physical-ai-humanoid-robotics-textb-nu.vercel.app/sitemap.xml
   QDRANT_COLLECTION_NAME=rag_embedding
   ```

## Ingestion Testing

### 1. Run the ingestion pipeline:
```bash
cd backend
python -m src.ingestion.textbook_ingestor
```

### 2. Verify ingestion:
- Check logs for successful crawling, chunking, and storage
- Verify Qdrant collection has content:
```bash
curl http://localhost:8000/api/rag/collection-info
```

## Query Testing

### 1. Terminal Testing (curl):
```bash
# Basic query
curl -X POST http://localhost:8000/api/rag/query \
  -H "Content-Type: application/json" \
  -d '{
    "text": "What is the main concept in humanoid robotics?",
    "max_chunks": 5,
    "similarity_threshold": 0.25
  }'

# Query with metadata filtering
curl -X POST http://localhost:8000/api/rag/query \
  -H "Content-Type: application/json" \
  -d '{
    "text": "Explain kinematics in robotics",
    "max_chunks": 5,
    "similarity_threshold": 0.25,
    "metadata_filters": {
      "module": "Module 2"
    }
  }'
```

### 2. Sample Queries for Testing:
```bash
# Test 1: General question
curl -X POST http://localhost:8000/api/rag/query \
  -H "Content-Type: application/json" \
  -d '{"text": "What are the key components of a humanoid robot?", "max_chunks": 5, "similarity_threshold": 0.25}'

# Test 2: Module-specific question
curl -X POST http://localhost:8000/api/rag/query \
  -H "Content-Type: application/json" \
  -d '{"text": "Explain inverse kinematics", "max_chunks": 5, "similarity_threshold": 0.25, "metadata_filters": {"module": "Module 3"}}'

# Test 3: Non-existent content (should return fallback)
curl -X POST http://localhost:8000/api/rag/query \
  -H "Content-Type: application/json" \
  -d '{"text": "What is quantum computing?", "max_chunks": 5, "similarity_threshold": 0.25}'
```

## Frontend Testing
1. Start the backend: `cd backend && uvicorn main:app --reload`
2. Start the frontend: `cd book && npm run start`
3. Navigate to the site and test the RAG query component

## Common Mistakes Checklist
- ❌ Using mock/sample text instead of real textbook content
- ❌ Hallucinating answers not present in retrieved context
- ❌ Not applying minimum similarity threshold
- ❌ Not including proper citations and sources
- ❌ Not extracting module/section information
- ❌ Using Gemini instead of OpenRouter as specified
- ❌ Not handling fallback responses properly

## Expected Response Format
The response should always include:
1. **answer**: The actual answer from textbook content
2. **citations**: List of citations with text, source, and URL
3. **chunks_used**: List of text chunks used to generate the answer
4. **query**: The original query text
5. **timestamp**: When the response was generated

## Troubleshooting
- **No results**: Verify Qdrant collection has content and similarity threshold is appropriate
- **Wrong answers**: Check that ingestion pipeline ran successfully and content was properly stored
- **API errors**: Verify all API keys are correct and services are accessible
- **Slow responses**: Check network connectivity and API rate limits