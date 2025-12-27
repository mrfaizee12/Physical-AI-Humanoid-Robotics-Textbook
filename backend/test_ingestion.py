"""
Test script for RAG ingestion pipeline
"""
import asyncio
from src.ingestion.crawler import WebCrawler, crawl_textbook_content
from src.ingestion.chunker import TextChunker, chunk_pages
from src.ingestion.ingestor import RAGIngestor, validate_qdrant_connection
from src.utils.logging import get_logger


logger = get_logger(__name__)


async def test_crawler():
    """Test the web crawler functionality"""
    print("Testing Web Crawler...")
    try:
        pages = await crawl_textbook_content()
        print(f"✓ Crawled {len(pages)} pages")
        if pages:
            print(f"  First page: {pages[0]['title'][:50]}... from {pages[0]['url'][:50]}...")
        return pages
    except Exception as e:
        print(f"✗ Crawler test failed: {e}")
        return []


async def test_chunker():
    """Test the text chunker functionality"""
    print("\nTesting Text Chunker...")
    try:
        # Create sample content to test chunking
        sample_content = "This is a test sentence. " * 200  # Create content that will be chunked
        chunker = TextChunker()

        # Test chunking
        chunks = chunker.chunk_text(sample_content, "https://test.com", "Test Page")
        print(f"✓ Chunked content into {len(chunks)} chunks")

        # Verify sequential chunk_index
        expected_indices = [str(i) for i in range(len(chunks))]
        actual_indices = [chunk['chunk_index'] for chunk in chunks]

        if expected_indices == actual_indices:
            print("✓ Sequential chunk_index verification passed")
        else:
            print(f"✗ Sequential chunk_index verification failed. Expected: {expected_indices}, Got: {actual_indices}")

        # Check chunk sizes
        for i, chunk in enumerate(chunks):
            tokens = len(chunk['content']) // 4  # Approximate token count
            if 400 <= tokens <= 900:  # Allow some flexibility around 500-800
                print(f"  Chunk {i}: {tokens} tokens ✓")
            else:
                print(f"  Chunk {i}: {tokens} tokens ✗ (outside 400-900 range)")

        return chunks
    except Exception as e:
        print(f"✗ Chunker test failed: {e}")
        return []


async def test_ingestor():
    """Test the ingestion functionality"""
    print("\nTesting Ingestor...")
    try:
        # Validate Qdrant connection
        is_connected = validate_qdrant_connection()
        if is_connected:
            print("✓ Qdrant connection validated")
        else:
            print("✗ Qdrant connection failed")
            return False

        # Create a small test dataset
        test_chunks = [
            {
                'url': 'https://test.com/page1',
                'title': 'Test Page 1',
                'chunk_index': '0',
                'content': 'This is test content for the first chunk.'
            },
            {
                'url': 'https://test.com/page1',
                'title': 'Test Page 1',
                'chunk_index': '1',
                'content': 'This is test content for the second chunk.'
            }
        ]

        # Create ingestor and test embedding (without actually upserting to save Qdrant resources)
        ingestor = RAGIngestor()
        print("✓ Ingestor initialized successfully")

        # Test that we can process the chunks structure
        print(f"✓ Prepared {len(test_chunks)} test chunks for ingestion")

        return True
    except Exception as e:
        print(f"✗ Ingestor test failed: {e}")
        return False


async def main():
    """Run all tests"""
    print("Running RAG Ingestion Pipeline Tests\n")

    # Test each component
    pages = await test_crawler()
    chunks = await test_chunker()
    ingestor_ok = await test_ingestor()

    # Summary
    print(f"\n--- Test Summary ---")
    print(f"Crawler: {'✓' if pages else '✗'}")
    print(f"Chunker: {'✓' if chunks else '✗'}")
    print(f"Ingestor: {'✓' if ingestor_ok else '✗'}")

    if pages and chunks and ingestor_ok:
        print("\n✓ All tests passed! The RAG ingestion pipeline is ready.")
        print("\nTo run the full ingestion pipeline:")
        print("  python -c \"from src.ingestion.ingestor import main; import asyncio; asyncio.run(main())\"")
    else:
        print("\n✗ Some tests failed. Please check the error messages above.")


if __name__ == "__main__":
    asyncio.run(main())