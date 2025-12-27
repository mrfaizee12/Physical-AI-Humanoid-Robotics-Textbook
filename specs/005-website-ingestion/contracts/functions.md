# Function Interface Contracts: Website Ingestion Pipeline

## Public Functions

### `get_all_urls(sitemap_url: str) -> List[str]`
- **Purpose**: Discover and return all Docusaurus page URLs from the given sitemap URL
- **Input**: Sitemap URL (https://physical-ai-humanoid-robotics-textb-nu.vercel.app/sitemap.xml)
- **Output**: List of valid URLs to process
- **Errors**: May raise exception if sitemap URL is inaccessible

### `extract_text_from_url(url: str) -> Tuple[str, str]`
- **Purpose**: Extract clean text content and page title from a given URL
- **Input**: URL of the page to extract content from (from target site)
- **Output**: Tuple of (page_title, clean_text_content)
- **Errors**: May raise exception if URL is inaccessible or content cannot be parsed

### `chunk_text(content: str, max_tokens: int = 1500) -> List[Dict]`
- **Purpose**: Split text content into appropriately sized chunks
- **Input**: Content string to chunk and maximum tokens per chunk
- **Output**: List of dictionaries containing chunk information
- **Errors**: None expected under normal conditions

### `embed(text_chunks: List[str]) -> List[List[float]]`
- **Purpose**: Generate Cohere embeddings for a list of text chunks
- **Input**: List of text strings to embed
- **Output**: List of embedding vectors (lists of floats)
- **Errors**: May raise exception due to API limits or invalid content

### `create_collection(collection_name: str = "rag_embedding") -> bool`
- **Purpose**: Create a Qdrant collection with the specified name
- **Input**: Name of the collection to create
- **Output**: Boolean indicating success
- **Errors**: May raise exception if Qdrant is unavailable

### `save_chunk_to_qdrant(chunk_id: str, embedding: List[float], metadata: Dict) -> bool`
- **Purpose**: Save a single chunk with its embedding to Qdrant
- **Input**: Chunk ID, embedding vector, and metadata dictionary
- **Output**: Boolean indicating success
- **Errors**: May raise exception if Qdrant is unavailable

### `main() -> None`
- **Purpose**: Execute the complete ingestion pipeline
- **Input**: None (reads from environment variables)
- **Output**: None (performs complete pipeline execution)
- **Errors**: May raise exceptions during execution, with appropriate error handling