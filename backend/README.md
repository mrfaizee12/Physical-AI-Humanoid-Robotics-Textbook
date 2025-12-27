# Website Ingestion Pipeline

This project implements a complete website ingestion pipeline that fetches Docusaurus pages from a sitemap, extracts text content, chunks it, generates Cohere embeddings, and stores them in Qdrant Cloud.

## Features

- **Sitemap Processing**: Discovers all pages from a Docusaurus sitemap XML
- **Text Extraction**: Extracts clean text content from web pages using BeautifulSoup4
- **Content Chunking**: Splits content into 1,500 token chunks for optimal processing
- **Embedding Generation**: Creates vector embeddings using Cohere's API
- **Vector Storage**: Stores embeddings in Qdrant Cloud with metadata
- **Similarity Search**: Performs semantic search on stored content
- **Idempotent Execution**: Safe to run multiple times without duplicates
- **Error Handling**: Comprehensive error handling with retry logic
- **Memory Management**: Stays under 100MB memory usage

## Prerequisites

- Python 3.11+
- UV package manager
- Cohere API key
- Qdrant Cloud account and API key

## Setup

1. Clone the repository and navigate to the backend directory
2. Install dependencies:
   ```bash
   uv sync
   # Or install manually:
   pip install requests beautifulsoup4 cohere qdrant-client python-dotenv lxml psutil
   ```

3. Create a `.env` file with the following environment variables:
   ```env
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_URL=your_qdrant_cloud_url
   QDRANT_API_KEY=your_qdrant_api_key
   TARGET_WEBSITE_URL=https://physical-ai-humanoid-robotics-textb-nu.vercel.app/
   SITEMAP_URL=https://physical-ai-humanoid-robotics-textb-nu.vercel.app/sitemap.xml
   ```

## Usage

### Run the Complete Pipeline

```bash
cd backend
python main.py
```

This will:
1. Create the Qdrant collection if it doesn't exist
2. Fetch all URLs from the sitemap
3. Process only unprocessed URLs (idempotent execution)
4. Extract text content from each page
5. Chunk content into appropriate sizes
6. Generate embeddings using Cohere
7. Store vectors in Qdrant with metadata
8. Run a sample similarity test

### Run the FastAPI Server

If you have a FastAPI application in the backend, run it using UV to ensure all dependencies are available:

```bash
cd backend
uv run uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

Or if your FastAPI app is in a different module:

```bash
cd backend
uv run uvicorn src.api.main:app --reload --host 0.0.0.0 --port 8000
```

### Run Individual Functions

You can also import and use individual functions from the main.py file:

```python
from main import get_all_urls, extract_text_from_url, chunk_text, embed, query_similarity

# Get all URLs from sitemap
urls = get_all_urls("https://physical-ai-humanoid-robotics-textb-nu.vercel.app/sitemap.xml")

# Extract text from a specific URL
title, content = extract_text_from_url("https://physical-ai-humanoid-robotics-textb-nu.vercel.app/docs/intro")

# Chunk text content
chunks = chunk_text(content)

# Generate embeddings for text
embeddings = embed(["Your text content here"])

# Perform similarity search
results = query_similarity("Your search query here", top_k=5)
```

## Configuration

The pipeline can be configured using environment variables in the `.env` file:

- `COHERE_API_KEY`: Your Cohere API key for generating embeddings
- `QDRANT_URL`: Your Qdrant Cloud instance URL
- `QDRANT_API_KEY`: Your Qdrant API key
- `TARGET_WEBSITE_URL`: The base URL of the target website (default: https://physical-ai-humanoid-robotics-textb-nu.vercel.app/)
- `SITEMAP_URL`: The sitemap XML URL (default: https://physical-ai-humanoid-robotics-textb-nu.vercel.app/sitemap.xml)

## Architecture

The pipeline consists of the following components:

- `get_all_urls()`: Parses sitemap XML and extracts all URLs
- `extract_text_from_url()`: Extracts clean text content from web pages
- `chunk_text()`: Splits content into appropriately sized chunks
- `embed()`: Generates vector embeddings using Cohere
- `create_collection()`: Creates Qdrant collection for storing vectors
- `save_chunk_to_qdrant()`: Stores individual chunks with embeddings in Qdrant
- `query_similarity()`: Performs semantic search on stored content
- `main()`: Orchestrates the complete pipeline execution

## Idempotency

The pipeline is designed to be idempotent, meaning it can be run multiple times safely without creating duplicate entries. It checks Qdrant for already processed URLs before processing them again.

## Error Handling

The pipeline includes comprehensive error handling:
- Network request retries with exponential backoff
- Graceful handling of 404 errors for broken links
- Memory usage monitoring
- Detailed logging for debugging

## Performance

- Processes up to 500 pages within 30 minutes
- Maintains memory usage under 100MB
- Implements efficient scrolling for large datasets
- Uses batch operations where possible