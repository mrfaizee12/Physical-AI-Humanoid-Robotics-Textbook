import asyncio
import uuid
from typing import List, Dict
from qdrant_client.http import models
from src.services.qdrant_service import QdrantService
from src.services.embedding_service import embedding_service
from src.utils.logging import get_logger
from src.ingestion.crawler import crawl_textbook_content
from src.ingestion.chunker import chunk_pages


logger = get_logger(__name__)


class RAGIngestor:
    """
    Ingestor to embed and upsert textbook content to Qdrant
    """

    def __init__(self):
        self.qdrant_service = QdrantService()

    async def embed_and_upsert_chunks(self, chunks: List[Dict[str, str]]):
        """
        Embed chunks and upsert to Qdrant
        """
        logger.info(f"Starting ingestion of {len(chunks)} chunks...")

        # Process in batches to avoid memory issues
        batch_size = 100
        for i in range(0, len(chunks), batch_size):
            batch = chunks[i:i + batch_size]
            await self._process_batch(batch)
            logger.info(f"Processed batch {i//batch_size + 1}/{(len(chunks)-1)//batch_size + 1}")

        logger.info(f"Successfully ingested {len(chunks)} chunks to Qdrant")

    async def _process_batch(self, batch: List[Dict[str, str]]):
        """
        Process a batch of chunks - embed and upsert to Qdrant
        """
        # Extract content for embedding
        texts = [chunk['content'] for chunk in batch]

        # Generate embeddings
        embeddings = await embedding_service.generate_embeddings_batch(texts)

        # Prepare points for upsert
        points = []
        for i, (chunk, embedding) in enumerate(zip(batch, embeddings)):
            point_id = str(uuid.uuid5(uuid.NAMESPACE_DNS, f"{chunk['url']}_{chunk['chunk_index']}"))

            point = models.PointStruct(
                id=point_id,
                vector=embedding,
                payload={
                    "url": chunk['url'],
                    "title": chunk['title'],
                    "chunk_index": int(chunk['chunk_index']),
                    "content": chunk['content'],
                    "module": chunk.get('module', ''),
                    "section": chunk.get('section', ''),
                    "subsection": chunk.get('subsection', '')
                }
            )
            points.append(point)

        # Upsert to Qdrant
        try:
            self.qdrant_service.client.upsert(
                collection_name=self.qdrant_service.collection_name,
                points=points
            )
            logger.info(f"Upserted {len(points)} points to Qdrant collection {self.qdrant_service.collection_name}")
        except Exception as e:
            logger.error(f"Error upserting batch to Qdrant: {e}")
            raise

    async def ingest_textbook(self):
        """
        Complete ingestion pipeline: crawl -> chunk -> embed -> upsert
        """
        logger.info("Starting textbook ingestion pipeline...")

        # Step 1: Crawl website content
        logger.info("Step 1: Crawling website content...")
        pages = await crawl_textbook_content()
        if not pages:
            logger.warning("No pages crawled. Check TARGET_WEBSITE_URL and SITEMAP_URL configuration.")
            return

        logger.info(f"Crawled {len(pages)} pages")

        # Step 2: Chunk content
        logger.info("Step 2: Chunking content...")
        chunks = chunk_pages(pages)
        if not chunks:
            logger.warning("No chunks created. Content may be too short or chunking failed.")
            return

        logger.info(f"Created {len(chunks)} chunks")

        # Step 3: Embed and upsert to Qdrant
        logger.info("Step 3: Embedding and upserting to Qdrant...")
        await self.embed_and_upsert_chunks(chunks)

        # Log final metrics as required
        logger.info(f"=== INGESTION METRICS ===")
        logger.info(f"Total pages crawled: {len(pages)}")
        logger.info(f"Total chunks created: {len(chunks)}")
        if chunks:
            final_chunk_index = max(int(chunk['chunk_index']) for chunk in chunks)
            logger.info(f"Final chunk_index count: {final_chunk_index}")
        logger.info(f"========================")

        logger.info("Textbook ingestion completed successfully!")


def validate_qdrant_connection():
    """
    Validate that Qdrant connection is working
    """
    try:
        qdrant_service = QdrantService()
        collections = qdrant_service.client.get_collections()
        collection_names = [collection.name for collection in collections.collections]

        if qdrant_service.collection_name in collection_names:
            logger = get_logger(__name__)
            logger.info(f"Qdrant collection '{qdrant_service.collection_name}' exists")
            return True
        else:
            logger = get_logger(__name__)
            logger.warning(f"Qdrant collection '{qdrant_service.collection_name}' does not exist")
            return False
    except Exception as e:
        logger = get_logger(__name__)
        logger.error(f"Error connecting to Qdrant: {e}")
        return False


async def main():
    """
    Main function to run the ingestion pipeline
    """
    # Validate Qdrant connection first
    if not validate_qdrant_connection():
        logger.error("Qdrant connection validation failed. Please check your configuration.")
        return

    # Run ingestion
    ingestor = RAGIngestor()
    await ingestor.ingest_textbook()


if __name__ == "__main__":
    asyncio.run(main())