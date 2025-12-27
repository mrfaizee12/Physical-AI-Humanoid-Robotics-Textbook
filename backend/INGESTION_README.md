# RAG Ingestion Pipeline for Physical AI & Humanoid Robotics Textbook

This document describes how to run the RAG ingestion pipeline to properly index the Physical AI & Humanoid Robotics Textbook content.

## Features

- **Web Crawling**: Fetches content from TARGET_WEBSITE_URL and SITEMAP_URL
- **Smart Chunking**: Splits content into fixed-size chunks (500-800 tokens) with sequential chunk_index
- **Sequential Indexing**: Each page has its own sequential chunk_index starting from 0 (0,1,2,3…)
- **Embedding**: Uses COHERE_API_KEY to generate embeddings
- **Qdrant Integration**: Upserts into QDRANT_COLLECTION_NAME at QDRANT_URL
- **Proper Payload**: Includes {url, title, chunk_index, content} in Qdrant

## Prerequisites

Make sure the following environment variables are set in your `.env` file:

```bash
COHERE_API_KEY=your_cohere_api_key
   QDRANT_URL=your_qdrant_cloud_url
   QDRANT_API_KEY=your_qdrant_api_key
   TARGET_WEBSITE_URL=https://physical-ai-humanoid-robotics-textb-nu.vercel.app/
   SITEMAP_URL=https://physical-ai-humanoid-robotics-textb-nu.vercel.app/sitemap.xml
   ```
```

## Installation

The required dependencies are already included in `requirements.txt`:

```bash
pip install aiohttp beautifulsoup4 lxml
```

## Usage

### 1. Run the Ingestion Pipeline

```bash
cd backend
python -c "from src.ingestion.ingestor import main; import asyncio; asyncio.run(main())"
```

### 2. Alternative: Run Step-by-Step

You can also run the pipeline components individually:

```bash
# Test the components first
python test_ingestion_simple.py

# Or run just the crawler
python -c "from src.ingestion.crawler import crawl_textbook_content; import asyncio; print(asyncio.run(crawl_textbook_content()))"

# Or run just the chunker on sample data
python -c "from src.ingestion.chunker import chunk_pages; print(chunk_pages([{'url': 'https://test.com', 'title': 'Test', 'content': 'This is test content. ' * 100}]))"
```

## Architecture

The ingestion pipeline consists of three main components:

### 1. Web Crawler (`src/ingestion/crawler.py`)
- Fetches URLs from sitemap
- Crawls TARGET_WEBSITE_URL and discovered URLs
- Extracts title and content from HTML pages

### 2. Text Chunker (`src/ingestion/chunker.py`)
- Splits content into fixed-size chunks (500-800 tokens)
- Maintains sentence boundaries when possible
- Assigns sequential chunk_index per page (0,1,2,3…)
- Handles large content by force-splitting at word boundaries

### 3. Ingestor (`src/ingestion/ingestor.py`)
- Generates embeddings using Cohere API
- Upserts to Qdrant with proper payload structure
- Includes error handling and batch processing

## Payload Structure in Qdrant

Each chunk is stored in Qdrant with the following payload:

```json
{
  "url": "https://example.com/page",
  "title": "Page Title",
  "chunk_index": 0,
  "content": "Chunk content text..."
}
```

## Query Behavior

After ingestion, the RAG system will:
- Retrieve relevant chunks from Qdrant based on user query
- Answer strictly from textbook content
- Provide citations with title, URL, and chunk_index
- Reply "I don't know based on the textbook." if information is not found

## Troubleshooting

1. **Connection Issues**: Verify QDRANT_URL and QDRANT_API_KEY are correct
2. **Crawling Issues**: Check TARGET_WEBSITE_URL and SITEMAP_URL are accessible
3. **Embedding Issues**: Verify COHERE_API_KEY is valid and has sufficient quota
4. **Chunking Issues**: Adjust min_chunk_size and max_chunk_size in TextChunker if needed

## Verification

After running the ingestion pipeline, you can verify the results by:
1. Checking the Qdrant collection for new vectors
2. Querying the `/api/rag/query` endpoint to test retrieval
3. Confirming that chunk_index values are sequential per page