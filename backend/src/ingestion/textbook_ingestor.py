"""
Textbook Ingestor - Complete pipeline for crawling, chunking, and storing textbook content in Qdrant
"""
import asyncio
import sys
from typing import List, Dict, Any
from src.ingestion.crawler import crawl_textbook_content
from src.ingestion.chunker import chunk_pages
from src.ingestion.ingestor import RAGIngestor
from src.utils.logging import get_logger
from src.config import settings

logger = get_logger(__name__)


async def run_textbook_ingestion_pipeline():
    """
    Complete pipeline: crawl -> chunk -> embed -> store in Qdrant
    """
    logger.info("Starting textbook ingestion pipeline...")

    # Validate configuration
    missing_creds = settings.check_missing_credentials()
    if missing_creds:
        logger.error(f"Missing required credentials: {', '.join(missing_creds)}")
        logger.error("Please set the required environment variables in your .env file")
        return False

    logger.info("Configuration validated successfully")

    # Initialize the ingestor
    ingestor = RAGIngestor()

    # Step 1: Crawl textbook content
    logger.info("Step 1: Crawling textbook content from website...")
    pages = await crawl_textbook_content()

    if not pages:
        logger.error("No pages were crawled. Check your TARGET_WEBSITE_URL and SITEMAP_URL configuration.")
        return False

    logger.info(f"Successfully crawled {len(pages)} pages")

    # Step 2: Chunk content with semantic awareness
    logger.info("Step 2: Chunking content with semantic boundaries...")
    chunks = chunk_pages(pages)

    if not chunks:
        logger.error("No chunks were created. Content may be too short or chunking failed.")
        return False

    logger.info(f"Successfully created {len(chunks)} chunks")

    # Step 3: Embed and store in Qdrant
    logger.info("Step 3: Embedding and storing in Qdrant...")
    await ingestor.embed_and_upsert_chunks(chunks)

    # Log final metrics
    logger.info("=== INGESTION PIPELINE COMPLETED ===")
    logger.info(f"Pages crawled: {len(pages)}")
    logger.info(f"Chunks created: {len(chunks)}")
    if chunks:
        final_chunk_index = max(int(chunk['chunk_index']) for chunk in chunks)
        logger.info(f"Final chunk index: {final_chunk_index}")
    logger.info("=====================================")

    return True


async def validate_ingestion():
    """
    Validate that the textbook content has been properly ingested
    """
    from src.services.qdrant_service import qdrant_service

    logger.info("Validating ingestion...")

    # Check if collection exists
    collection_exists = await qdrant_service.verify_collection_exists()
    if not collection_exists:
        logger.error(f"Collection '{qdrant_service.collection_name}' does not exist in Qdrant")
        return False

    # Get collection info
    info = await qdrant_service.get_collection_info()
    if info:
        logger.info(f"Collection '{info['name']}' has {info['point_count']} vectors")
        logger.info(f"Vector size: {info['vector_size']}, Distance: {info['distance']}")

        if info['point_count'] == 0:
            logger.warning("Collection exists but is empty")
            return False
        else:
            logger.info("✅ Textbook content is available in Qdrant")
            return True
    else:
        logger.error("Could not get collection info")
        return False


async def main():
    """
    Main function to run the textbook ingestion pipeline
    """
    print("📚 Starting Textbook RAG Ingestion Pipeline")
    print("=" * 50)

    # Run the ingestion pipeline
    success = await run_textbook_ingestion_pipeline()

    if success:
        print("\n✅ Ingestion pipeline completed successfully!")

        # Validate the ingestion
        print("\n🔍 Validating ingestion...")
        validation_success = await validate_ingestion()

        if validation_success:
            print("\n🎉 Textbook content is ready for RAG queries!")
            print(f"   - Collection: {settings.qdrant_collection_name}")
            print(f"   - URL: {settings.target_website_url}")
            print(f"   - Sitemap: {settings.sitemap_url}")
        else:
            print("\n❌ Validation failed - content may not be properly ingested")
            sys.exit(1)
    else:
        print("\n❌ Ingestion pipeline failed")
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())