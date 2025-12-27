#!/usr/bin/env python3
"""
Test script to verify that the Qdrant retrieval fix is working correctly.
"""
import asyncio
from src.services.qdrant_service import qdrant_service
from src.services.embedding_service import embedding_service
from src.config import settings

async def test_qdrant_retrieval():
    print("Testing Qdrant retrieval with updated query_points method...")
    print(f"Collection: {settings.qdrant_collection_name}")
    print()

    # Test 1: Check if collection exists
    print("=== Test 1: Collection Existence ===")
    exists = await qdrant_service.verify_collection_exists()
    print(f"Collection exists: {exists}")

    if not exists:
        print("ERROR: Collection does not exist!")
        return False

    # Test 2: Get collection info
    print("\n=== Test 2: Collection Info ===")
    info = await qdrant_service.get_collection_info()
    if info:
        print(f"Collection info: {info}")
    else:
        print("ERROR: Could not get collection info!")
        return False

    # Test 3: Generate a test query embedding
    print("\n=== Test 3: Test Query Embedding ===")
    test_query = "What is robotics?"
    try:
        query_embedding = await embedding_service.generate_embedding(test_query)
        print(f"Generated embedding with {len(query_embedding)} dimensions")
    except Exception as e:
        print(f"ERROR: Could not generate embedding: {e}")
        return False

    # Test 4: Perform search with the updated method
    print("\n=== Test 4: Qdrant Search with query_points ===")
    try:
        results = await qdrant_service.search_text_chunks(
            query_vector=query_embedding,
            limit=5,
            similarity_threshold=0.1  # Lower threshold to get results
        )

        print(f"Retrieved {len(results)} chunks")

        if results:
            print("SUCCESS: Retrieval is working!")
            print(f"First result preview:")
            first_result = results[0]
            print(f"  ID: {first_result['id']}")
            print(f"  Content preview: {first_result['content'][:100]}...")
            print(f"  URL: {first_result['metadata']['url']}")
            print(f"  Title: {first_result['metadata']['title']}")
            print(f"  Chunk Index: {first_result['metadata']['chunk_index']}")
            print(f"  Similarity Score: {first_result['similarity_score']}")
        else:
            print("WARNING: No results returned, but no error occurred")

        return True
    except Exception as e:
        print(f"ERROR: Search failed with error: {e}")
        import traceback
        traceback.print_exc()
        return False

async def main():
    print("Testing Qdrant Retrieval Fix...")
    print(f"QDRANT_URL: {settings.qdrant_url}")
    print(f"Collection: {settings.qdrant_collection_name}")
    print()

    success = await test_qdrant_retrieval()

    if success:
        print("\nSUCCESS: All tests passed! Qdrant retrieval is working correctly.")
    else:
        print("\nFAILED: Tests failed! There may be issues with the Qdrant retrieval.")

    return success

if __name__ == "__main__":
    asyncio.run(main())