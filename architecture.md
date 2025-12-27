# Real RAG Pipeline Architecture

## Overview
This RAG (Retrieval-Augmented Generation) pipeline is designed to provide accurate answers to questions about textbook content by retrieving relevant information from a vector database and generating responses based only on that context.

## Architecture Diagram

```
┌─────────────────┐    ┌──────────────────┐    ┌──────────────────┐
│   Frontend      │    │   FastAPI        │    │   Vector DB      │
│   (Docusaurus)  │◄──►│   Backend        │◄──►│   (Qdrant)       │
└─────────────────┘    └──────────────────┘    └──────────────────┘
         │                       │                        │
         │                       │                        │
         │  HTTP Request         │                        │
         │──────────────────────►│                        │
         │                       │                        │
         │                       │ 1. Embed Query         │
         │                       │───────────────────────►│
         │                       │                        │
         │                       │ 2. Search Vectors      │
         │                       │◄───────────────────────│
         │                       │                        │
         │                       │ 3. Retrieve Chunks     │
         │                       │                        │
         │                       │ 4. Generate Response   │
         │◄──────────────────────│                        │
         │  HTTP Response        │                        │
         │                       │                        │

┌─────────────────┐    ┌──────────────────┐    ┌──────────────────┐
│   Crawling      │    │   Embedding      │    │   Storage        │
│   (Sitemap)     │───►│   (Cohere)       │───►│   (Qdrant)       │
└─────────────────┘    └──────────────────┘    └──────────────────┘
         │                       │                        │
         │                       │                        │
         │ 1. Fetch Pages        │                        │
         │──────────────────────►│                        │
         │                       │                        │
         │ 2. Chunk Content      │                        │
         │──────────────────────►│                        │
         │                       │                        │
         │ 3. Generate Embeddings│                        │
         │──────────────────────►│                        │
         │                       │                        │
         │ 4. Store in Qdrant    │                        │
         │──────────────────────►│                        │
```

## Component Flow

### 1. Ingestion Pipeline
- **Crawler**: Fetches content from textbook website using sitemap
- **Chunker**: Splits content into semantic chunks (500-800 tokens) with metadata
- **Ingestor**: Embeds chunks using Cohere and stores in Qdrant

### 2. Query Pipeline
- **Embedding Service**: Generates query embedding using Cohere
- **Qdrant Service**: Performs similarity search with metadata filtering
- **LLM Service**: Generates response using OpenRouter with strict context adherence
- **RAG Service**: Orchestrates the entire pipeline

## Tech Stack
- **Backend**: FastAPI
- **RAG**: OpenAI Agents SDK (via OpenRouter)
- **LLM**: OpenRouter (Mistral model)
- **Vector DB**: Qdrant Cloud
- **Embeddings**: Cohere (embed-english-v3.0)
- **Frontend**: Docusaurus with React components

## Response Format (MANDATORY)
1. **Answer**: Direct response based only on retrieved context
2. **Citations**: Exact textbook text from retrieved chunks
3. **Source URL**: Original URL of the content
4. **Module & Section Reference**: Module/Section information from metadata

## Key Features
- Semantic chunking with module/section extraction
- Metadata filtering capability
- Strict adherence to textbook content (no hallucination)
- Proper citation with source information
- Fallback response when content not found

## Configuration
- QDRANT_API_KEY: Qdrant Cloud API key
- QDRANT_URL: Qdrant Cloud URL
- COHERE_API_KEY: Cohere API key
- OPENROUTER_API_KEY: OpenRouter API key
- TARGET_WEBSITE_URL: Textbook website URL
- SITEMAP_URL: Sitemap URL for crawling
- QDRANT_COLLECTION_NAME: Collection name (default: rag_embedding)