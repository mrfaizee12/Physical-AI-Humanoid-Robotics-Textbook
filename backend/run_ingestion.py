#!/usr/bin/env python3
"""
RAG Ingestion Pipeline for Physical AI & Humanoid Robotics Textbook
"""
import asyncio
import os
from src.ingestion.ingestor import main as ingestion_main
from src.utils.logging import setup_logging


def check_environment_variables():
    """Check that required environment variables are set"""
    required_vars = [
        'QDRANT_URL',
        'QDRANT_API_KEY',
        'COHERE_API_KEY',
        'TARGET_WEBSITE_URL',
        'SITEMAP_URL'
    ]

    missing_vars = []
    for var in required_vars:
        if not os.getenv(var):
            missing_vars.append(var)

    if missing_vars:
        print(f"ERROR: Missing required environment variables: {', '.join(missing_vars)}")
        print("\nPlease set these variables in your environment or .env file:")
        for var in missing_vars:
            print(f"  export {var}=your_{var.lower()}_here")
        return False

    print("OK: All required environment variables are set")
    return True


async def run_ingestion_pipeline():
    """Run the complete ingestion pipeline"""
    print("Starting RAG Ingestion Pipeline for Physical AI & Humanoid Robotics Textbook...")
    print(f"Target Website: {os.getenv('TARGET_WEBSITE_URL')}")
    print(f"Sitemap: {os.getenv('SITEMAP_URL')}")
    print(f"Qdrant Collection: {os.getenv('QDRANT_COLLECTION_NAME', 'rag_embedding')}")
    print("-" * 60)

    try:
        await ingestion_main()
        print("\nSUCCESS: RAG ingestion pipeline completed successfully!")
        print("Textbook content is now available for querying via the RAG system.")
    except Exception as e:
        print(f"\nERROR: Ingestion pipeline failed with error: {e}")
        import traceback
        traceback.print_exc()
        return False

    return True


if __name__ == "__main__":
    # Set up logging
    setup_logging()

    # Check environment variables
    if not check_environment_variables():
        exit(1)

    # Run the ingestion pipeline
    success = asyncio.run(run_ingestion_pipeline())

    if success:
        print("\n" + "="*60)
        print("INGESTION COMPLETE")
        print("="*60)
        print("The Physical AI & Humanoid Robotics Textbook has been successfully")
        print("ingested with proper chunking (500-800 tokens) and sequential")
        print("chunk_index per page (0,1,2,3...).")
        print("\nNext steps:")
        print("1. Test the RAG query endpoint: /api/rag/query")
        print("2. Verify citations include title + url + chunk_index")
        print("3. Confirm fallback behavior: 'I don't know based on the textbook.'")
    else:
        print("\n" + "="*60)
        print("INGESTION FAILED")
        print("="*60)
        print("Please check the error messages above and try again.")
        exit(1)