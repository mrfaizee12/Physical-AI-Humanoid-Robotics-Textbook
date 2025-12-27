# Quickstart: Retrieval Validation

## Prerequisites

- Python 3.11+
- Qdrant Cloud account with rag_embedding collection populated
- Cohere API key
- Existing backend environment configured (from ingestion pipeline)

## Setup

1. Ensure your `.env` file contains the required configuration:
```env
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
TARGET_WEBSITE_URL=https://physical-ai-humanoid-robotics-textb-nu.vercel.app/
SITEMAP_URL=https://physical-ai-humanoid-robotics-textb-nu.vercel.app/sitemap.xml
```

2. Install dependencies (if not already installed):
```bash
cd backend
pip install requests beautifulsoup4 cohere qdrant-client python-dotenv lxml psutil
```

## Running Validation

### Basic Validation
```bash
cd backend
python retrieval_validation.py
```

This will:
1. Connect to Qdrant collection
2. Validate all stored vectors
3. Run sample similarity searches
4. Generate a validation report

### Custom Validation
```bash
# Validate specific number of vectors
python retrieval_validation.py --limit 1000

# Run only connection validation
python retrieval_validation.py --test connection

# Run only similarity validation
python retrieval_validation.py --test similarity
```

## Validation Report

The validation will produce a report with:
- Connection status to Qdrant
- Vector integrity validation results
- Similarity search accuracy metrics
- Performance benchmarks
- Sample query results
- Recommendations for improvement

## Sample Output

```
Retrieval Validation Report
==========================
- Connected to Qdrant: YES
- Collection: rag_embedding
- Total vectors: 91
- Integrity validated: 91/91 (100%)
- Average similarity precision: 0.85
- Processing time: 2.34 minutes
- Sample queries tested: 5
```