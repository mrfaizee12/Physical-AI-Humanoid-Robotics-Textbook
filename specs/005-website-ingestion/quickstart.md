# Quickstart Guide: Website Ingestion Pipeline

## Prerequisites

- Python 3.11 or higher
- UV package manager
- Cohere API key
- Qdrant Cloud account and API key
- Access to target Docusaurus website (https://physical-ai-humanoid-robotics-textb-nu.vercel.app/)

## Setup

1. **Create the backend directory and initialize UV project:**
   ```bash
   mkdir backend
   cd backend
   uv init
   ```

2. **Install required dependencies:**
   ```bash
   uv pip install requests beautifulsoup4 cohere qdrant-client python-dotenv lxml
   ```

3. **Create environment file:**
   ```bash
   touch .env
   ```

4. **Add required environment variables to `.env`:**
   ```env
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=your_qdrant_cluster_url
   QDRANT_API_KEY=your_qdrant_api_key
   TARGET_WEBSITE_URL=https://physical-ai-humanoid-robotics-textb-nu.vercel.app/
   SITEMAP_URL=https://physical-ai-humanoid-robotics-textb-nu.vercel.app/sitemap.xml
   ```

## Running the Pipeline

1. **Create the main.py file with the ingestion pipeline code**

2. **Execute the pipeline:**
   ```bash
   cd backend
   python main.py
   ```

## Expected Output

- All Docusaurus pages from the target website (https://physical-ai-humanoid-robotics-textb-nu.vercel.app/) will be discovered via sitemap
- Text content will be extracted and chunked
- Embeddings will be generated for each chunk
- Vectors will be stored in Qdrant collection "rag_embedding"
- A sample similarity test will be performed at the end

## Configuration

- Adjust chunk size by modifying the chunking parameters in main.py
- Change the target website URL in the .env file
- Modify Qdrant collection name if needed (default: "rag_embedding")

## Troubleshooting

- If you encounter rate limiting, the pipeline includes built-in delays
- Check your API keys are valid and have sufficient quota
- Ensure the target website and sitemap are accessible
- Monitor memory usage during processing of large sites