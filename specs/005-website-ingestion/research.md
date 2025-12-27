# Research Notes: Website Ingestion Pipeline

## Decision: Backend Project Structure
**Rationale**: User specifically requested to create a backend folder and initialize UV project environment with all functionality in a single main.py file.
**Alternatives considered**:
- Multiple module structure (rejected - user requested single file)
- Jupyter notebook approach (rejected - user requested main.py script)

## Decision: URL Discovery Method
**Rationale**: For the target Docusaurus site, we'll specifically use the sitemap at https://physical-ai-humanoid-robotics-textb-nu.vercel.app/sitemap.xml to discover all pages.
**Alternatives considered**:
- Crawling internal links from the main page (rejected - sitemap is more reliable and complete)
- Hardcoding all URLs (rejected - not scalable)
- Using Docusaurus API (if available) - not needed for basic crawling

## Decision: Target Website
**Rationale**: Using https://physical-ai-humanoid-robotics-textb-nu.vercel.app/ as the target site to extract and process content from.
**Alternatives considered**:
- Other URLs (rejected - user specified this exact URL)

## Decision: Text Extraction Library
**Rationale**: Using BeautifulSoup4 to parse HTML and extract clean text content from Docusaurus pages, removing navigation and layout elements.
**Alternatives considered**:
- Selenium (rejected - overkill for static content)
- Scrapy (rejected - too complex for simple extraction)
- Regular expressions (rejected - not robust for HTML parsing)

## Decision: Text Chunking Strategy
**Rationale**: Following constitution requirement for 1,500 token chunks, we'll implement chunking by paragraphs or sentences while respecting content boundaries.
**Alternatives considered**:
- Fixed character limits (rejected - doesn't respect content structure)
- Sentence-based only (rejected - may create too small chunks)

## Decision: Cohere Embedding Model
**Rationale**: Using Cohere's embedding model for generating vector representations of text chunks.
**Alternatives considered**:
- OpenAI embeddings (rejected - user specified Cohere)
- Local embedding models (rejected - user specified Cohere)

## Decision: Qdrant Collection Name
**Rationale**: Using "rag_embedding" as specified in user requirements for the Qdrant collection name.
**Alternatives considered**:
- Other names (not applicable - user specified)

## Decision: Error Handling Approach
**Rationale**: Implementing graceful error handling for network requests, API limits, and processing failures to ensure pipeline reliability.
**Alternatives considered**:
- Aggressive failure mode (rejected - pipeline should be resilient)