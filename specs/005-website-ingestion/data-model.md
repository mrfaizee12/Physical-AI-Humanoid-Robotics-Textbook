# Data Model: Website Ingestion Pipeline

## Entities

### WebsitePage
- **url**: string (required) - The URL of the Docusaurus page from https://physical-ai-humanoid-robotics-textb-nu.vercel.app/
- **title**: string (required) - The title of the page extracted from HTML
- **content**: string (required) - The clean text content of the page
- **last_crawled**: datetime (required) - Timestamp when page was last processed
- **status**: enum (required) - Status of crawling (pending, processed, failed)

### TextChunk
- **id**: string (required) - Unique identifier for the chunk
- **content**: string (required) - The text content of the chunk (up to 1,500 tokens)
- **page_url**: string (required) - Reference to the source page URL from the target site
- **chunk_index**: integer (required) - Position of chunk within the source page
- **metadata**: object (required) - Additional metadata including page title and section info

### Embedding
- **chunk_id**: string (required) - Reference to the source text chunk
- **vector**: list<float> (required) - The embedding vector from Cohere
- **model**: string (required) - The model used to generate the embedding
- **created_at**: datetime (required) - Timestamp when embedding was created

### VectorRecord
- **id**: string (required) - Unique identifier for the vector record in Qdrant
- **vector**: list<float> (required) - The embedding vector
- **payload**: object (required) - Metadata payload containing chunk content, source URL, and additional info
- **collection**: string (required) - The Qdrant collection name ("rag_embedding")

## Relationships

- **WebsitePage** 1 → * **TextChunk**: One page can be split into multiple text chunks
- **TextChunk** 1 → 1 **Embedding**: One text chunk generates one embedding
- **Embedding** 1 → 1 **VectorRecord**: One embedding becomes one vector record in Qdrant

## Validation Rules

1. **WebsitePage**:
   - URL must be a valid HTTP/HTTPS URL from https://physical-ai-humanoid-robotics-textb-nu.vercel.app/
   - Content must not be empty
   - Status must be one of: pending, processed, failed

2. **TextChunk**:
   - Content must be less than 1,500 tokens
   - Chunk index must be non-negative
   - Page URL must reference an existing WebsitePage

3. **Embedding**:
   - Vector must have consistent dimensions based on the model
   - Model name must be valid Cohere model identifier

4. **VectorRecord**:
   - Collection name must be "rag_embedding"
   - ID must be unique within the collection