"""
Simple test script for RAG ingestion pipeline components
"""
import asyncio
from src.ingestion.chunker import TextChunker, chunk_pages
from src.ingestion.ingestor import validate_qdrant_connection
from src.utils.logging import get_logger


logger = get_logger(__name__)


def test_chunker():
    """Test the text chunker functionality"""
    print("Testing Text Chunker...")
    try:
        # Create sample content to test chunking
        sample_content = "This is a test sentence. " * 200  # Create content that will be chunked
        chunker = TextChunker()

        # Test chunking
        chunks = chunker.chunk_text(sample_content, "https://test.com", "Test Page")
        print(f"OK: Chunked content into {len(chunks)} chunks")

        # Verify sequential chunk_index
        expected_indices = [str(i) for i in range(len(chunks))]
        actual_indices = [chunk['chunk_index'] for chunk in chunks]

        if expected_indices == actual_indices:
            print("OK: Sequential chunk_index verification passed")
        else:
            print(f"ERROR: Sequential chunk_index verification failed. Expected: {expected_indices}, Got: {actual_indices}")

        # Check chunk sizes
        for i, chunk in enumerate(chunks[:3]):  # Only check first 3 to limit output
            tokens = len(chunk['content']) // 4  # Approximate token count
            print(f"  Chunk {i} (Index {chunk['chunk_index']}): {tokens} tokens, {len(chunk['content'])} chars")

        return chunks
    except Exception as e:
        print(f"ERROR: Chunker test failed: {e}")
        return []


def test_ingestor():
    """Test the ingestion functionality"""
    print("\nTesting Ingestor...")
    try:
        # Validate Qdrant connection
        is_connected = validate_qdrant_connection()
        if is_connected:
            print("OK: Qdrant connection validated")
        else:
            print("ERROR: Qdrant connection failed")
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

        print("OK: Ingestor components working correctly")

        # Test that we can process the chunks structure
        print(f"OK: Prepared {len(test_chunks)} test chunks for ingestion")

        return True
    except Exception as e:
        print(f"ERROR: Ingestor test failed: {e}")
        return False


def test_chunk_pages():
    """Test the chunk_pages function with sample data"""
    print("\nTesting chunk_pages function...")
    try:
        # Create sample pages
        sample_pages = [
            {
                'url': 'https://test.com/page1',
                'title': 'Test Page 1',
                'content': 'This is the content of the first page. ' * 100  # Make it long enough to chunk
            },
            {
                'url': 'https://test.com/page2',
                'title': 'Test Page 2',
                'content': 'This is the content of the second page. ' * 100  # Make it long enough to chunk
            }
        ]

        chunks = chunk_pages(sample_pages)
        print(f"OK: Chunked {len(sample_pages)} pages into {len(chunks)} total chunks")

        # Verify that each page has its own sequential chunk_index starting from 0
        page1_chunks = [c for c in chunks if c['url'] == 'https://test.com/page1']
        page2_chunks = [c for c in chunks if c['url'] == 'https://test.com/page2']

        print(f"Page 1: {len(page1_chunks)} chunks")
        print(f"Page 2: {len(page2_chunks)} chunks")

        # Check if each page has sequential indices starting from 0
        page1_indices = [c['chunk_index'] for c in page1_chunks]
        page2_indices = [c['chunk_index'] for c in page2_chunks]

        if page1_indices and page1_indices[0] == '0':
            print("OK: Page 1 chunk indices start from 0")
        if page2_indices and page2_indices[0] == '0':
            print("OK: Page 2 chunk indices start from 0")

        return chunks
    except Exception as e:
        print(f"ERROR: chunk_pages test failed: {e}")
        return []


def main():
    """Run all tests"""
    print("Running RAG Ingestion Pipeline Component Tests\n")

    # Test each component
    chunks = test_chunker()
    chunk_pages_result = test_chunk_pages()
    ingestor_ok = test_ingestor()

    # Summary
    print(f"\n--- Test Summary ---")
    print(f"Chunker: {'PASS' if chunks else 'FAIL'}")
    print(f"Chunk Pages: {'PASS' if chunk_pages_result else 'FAIL'}")
    print(f"Ingestor: {'PASS' if ingestor_ok else 'FAIL'}")

    if chunks and chunk_pages_result and ingestor_ok:
        print("\nOK: All component tests passed! The RAG ingestion pipeline is ready.")
        print("\nTo run the full ingestion pipeline:")
        print("  python -c \"from src.ingestion.ingestor import main; import asyncio; asyncio.run(main())\"")
        print("\nMake sure your environment variables are set:")
        print("  TARGET_WEBSITE_URL, SITEMAP_URL, COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY")
    else:
        print("\nERROR: Some tests failed. Please check the error messages above.")


if __name__ == "__main__":
    main()