# Quickstart Guide: Frontend Integration for RAG Agent

## Overview
This guide provides instructions for setting up and running the RAG agent frontend integration in the textbook website.

## Prerequisites
- Node.js 18+ installed
- Python 3.11+ installed
- Access to the RAG agent backend service
- Git for version control

## Setup Instructions

### 1. Clone the Repository
```bash
git clone <repository-url>
cd textbook
```

### 2. Install Frontend Dependencies
```bash
cd frontend
npm install
```

### 3. Install Backend Dependencies
```bash
cd backend
pip install -r requirements.txt
```

### 4. Environment Configuration
Create a `.env` file in the backend directory with the following variables:
```
RAG_AGENT_URL=http://localhost:8000
OPENAI_API_KEY=your-openai-key
QDRANT_URL=your-qdrant-url
```

### 5. Start the Services
Start the backend service:
```bash
cd backend
uv run python -m src.api.main
```

Alternatively, if running the FastAPI server directly:
```bash
cd backend
uv run uvicorn src.api.main:app --reload --host 0.0.0.0 --port 8000
```

In a new terminal, start the frontend development server:
```bash
cd frontend
npm run dev
```

## Integration Component
The RAG agent integration is implemented as a React component that can be embedded in Docusaurus pages:

```jsx
import RAGQueryComponent from './components/RAGQueryComponent';

// In your Docusaurus page
function MyPage() {
  return (
    <div>
      <h1>Interactive Textbook Query</h1>
      <RAGQueryComponent />
    </div>
  );
}
```

## Testing the Integration
1. Navigate to the page containing the RAG query component
2. Enter a question about the textbook content
3. Verify that the response appears with proper citations
4. Test error handling by submitting an empty query
5. Verify loading indicators appear during processing

## API Endpoints
- `POST /api/rag/query` - Submit queries to the RAG agent
- `GET /api/rag/health` - Check the health of the RAG service

## Troubleshooting
- If queries return no results, verify the RAG agent backend is running and accessible
- If citations don't appear, check that the source documents are properly indexed
- For CORS issues, ensure the backend allows requests from the frontend origin