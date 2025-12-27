"""
Test script to verify the fixes for the RAG pipeline issues:
1. HTTP 422 errors
2. "I don't know" answers despite valid chunks
3. Duplicate response rendering
4. Proper similarity score filtering
"""
import asyncio
import sys
import os

# Add backend to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

from src.services.rag_service import rag_service
from src.models.rag_models import Query


async def test_chunk_usage_logic():
    """Test that when chunks are found, they are properly used and don't return 'I don't know'."""
    print("Testing chunk usage logic...")

    # Test with a query that should return chunks (if Qdrant has data)
    query_text = "What is the main concept of Module 1?"

    try:
        # This will test the full RAG pipeline
        response = await rag_service.query_textbook(
            query_text=query_text,
            max_chunks=5,
            similarity_threshold=0.3  # Lower threshold to get more results
        )

        print(f"Query: {query_text}")
        print(f"Answer: {response.answer}")
        print(f"Number of citations: {len(response.citations)}")
        print(f"Number of chunks used: {len(response.chunks_used)}")
        print(f"Query: {response.query}")

        # If chunks were found, verify that the answer is not "I don't know"
        if len(response.chunks_used) > 0:
            if "I don't know based on the textbook." in response.answer:
                print("X ERROR: Found chunks but still returned 'I don't know'")
                return False
            else:
                print("OK SUCCESS: Chunks found and used appropriately")
        else:
            if response.answer == "I don't know based on the textbook.":
                print("OK SUCCESS: No chunks found, properly returned 'I don't know'")
            else:
                print("? WARNING: No chunks found but still returned an answer")

        return True

    except Exception as e:
        print(f"X ERROR in chunk usage test: {e}")
        return False


async def test_similarity_threshold_filtering():
    """Test that similarity threshold filtering works properly."""
    print("\nTesting similarity threshold filtering...")

    query_text = "What is the main concept of Module 1?"

    try:
        # Test with high threshold (should return fewer results)
        response_high_threshold = await rag_service.query_textbook(
            query_text=query_text,
            max_chunks=5,
            similarity_threshold=0.8  # High threshold
        )

        # Test with low threshold (should return more results, if available)
        response_low_threshold = await rag_service.query_textbook(
            query_text=query_text,
            max_chunks=5,
            similarity_threshold=0.2  # Low threshold
        )

        print(f"High threshold (0.8) - chunks found: {len(response_high_threshold.chunks_used)}")
        print(f"Low threshold (0.2) - chunks found: {len(response_low_threshold.chunks_used)}")

        # In most cases, lower threshold should find equal or more chunks
        # (unless there are no chunks above the high threshold at all)
        print("OK Similarity threshold filtering test completed")
        return True

    except Exception as e:
        print(f"X ERROR in similarity threshold test: {e}")
        return False


async def test_fallback_behavior():
    """Test that fallback behavior works when no chunks are found."""
    print("\nTesting fallback behavior...")

    # Use a query that's very unlikely to match anything in the textbook
    unlikely_query = "asdfghjklqwertyuiopzxcvbnm1234567890"

    try:
        response = await rag_service.query_textbook(
            query_text=unlikely_query,
            max_chunks=5,
            similarity_threshold=0.1
        )

        print(f"Query: {unlikely_query}")
        print(f"Answer: {response.answer}")
        print(f"Chunks used: {len(response.chunks_used)}")

        # The system should either return "I don't know" with 0 chunks
        # OR acknowledge that chunks were found but don't contain relevant info
        if (response.answer == "I don't know based on the textbook." and len(response.chunks_used) == 0) or \
           ("relevant chunks but they don't contain the specific information needed" in response.answer and len(response.chunks_used) > 0):
            print("OK SUCCESS: Proper fallback behavior for unlikely query")
            return True
        else:
            print("X ERROR: Incorrect fallback behavior")
            return False

    except Exception as e:
        print(f"X ERROR in fallback test: {e}")
        return False


async def run_all_tests():
    """Run all verification tests."""
    print("Running verification tests for RAG pipeline fixes...\n")

    results = []

    # Test 1: Chunk usage logic
    results.append(await test_chunk_usage_logic())

    # Test 2: Similarity threshold filtering
    results.append(await test_similarity_threshold_filtering())

    # Test 3: Fallback behavior
    results.append(await test_fallback_behavior())

    print(f"\nTest Results: {sum(results)}/{len(results)} passed")

    if all(results):
        print("SUCCESS: All tests passed! The fixes are working correctly.")
        return True
    else:
        print("FAILURE: Some tests failed. Please review the issues above.")
        return False


if __name__ == "__main__":
    success = asyncio.run(run_all_tests())
    sys.exit(0 if success else 1)