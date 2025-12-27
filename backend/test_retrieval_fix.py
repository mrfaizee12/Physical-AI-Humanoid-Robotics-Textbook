#!/usr/bin/env python3
"""
Test script to verify that the RAG retrieval fix is working correctly.
"""
import asyncio
from src.services.qdrant_service import qdrant_service
from src.services.embedding_service import embedding_service
from src.config import settings

async def test_retrieval_fix():
    print("Testing RAG retrieval fix...")
    print(f"Collection: {settings.qdrant_collection_name}")
    print(f"Embedding model: {embedding_service.model}")
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
        print(f"Vector size: {info['vector_size']}")
        print(f"Point count: {info['point_count']}")
        print(f"Distance: {info['distance']}")
    else:
        print("ERROR: Could not get collection info!")
        return False

    # Test 3: Generate a test query embedding with logging
    print("\n=== Test 3: Test Query Embedding ===")
    test_query = "What is robotics?"
    try:
        query_embedding = await embedding_service.generate_embedding(test_query)
        print(f"Generated embedding with {len(query_embedding)} dimensions")

        # Verify it matches the collection's vector size
        if len(query_embedding) == info['vector_size']:
            print(f"SUCCESS: Embedding dimensions match Qdrant vector size ({info['vector_size']})")
        else:
            print(f"ERROR: Embedding dimensions ({len(query_embedding)}) don't match Qdrant vector size ({info['vector_size']})")
            return False
    except Exception as e:
        print(f"ERROR: Could not generate embedding: {e}")
        return False

    # Test 4: Perform search with the fixed method (no score threshold)
    print("\n=== Test 4: Qdrant Search (Fixed) ===")
    try:
        results = await qdrant_service.search_text_chunks(
            query_vector=query_embedding,
            limit=5
            # similarity_threshold parameter is now ignored as required
        )

        print(f"Retrieved {len(results)} chunks (previously was 0)")

        if results:
            print("SUCCESS: Retrieval is now working!")
            print(f"First result preview:")
            first_result = results[0]
            print(f"  ID: {first_result['id']}")
            print(f"  Content preview: {first_result['content'][:100]}...")
            print(f"  URL: {first_result['metadata']['url']}")
            print(f"  Title: {first_result['metadata']['title']}")
            print(f"  Chunk Index: {first_result['metadata']['chunk_index']}")
            print(f"  Similarity Score: {first_result['similarity_score']}")
        else:
            print("WARNING: Still no results returned - there may be an issue with the embedding model match")
            print("Note: If data was ingested with a different model, vectors won't match.")

        return len(results) > 0  # Return True if we got results
    except Exception as e:
        print(f"ERROR: Search failed with error: {e}")
        import traceback
        traceback.print_exc()
        return False

async def main():
    print("Testing RAG Retrieval Fix...")
    print(f"QDRANT_URL: {settings.qdrant_url}")
    print(f"Collection: {settings.qdrant_collection_name}")
    print(f"Embedding model: {embedding_service.model}")
    print()

    success = await test_retrieval_fix()

    if success:
        print("\nSUCCESS: Retrieval is working correctly!")
        print("The fix has resolved the zero results issue.")
    else:
        print("\nWARNING: Retrieval may still have issues.")
        print("Note: If data was ingested with a different embedding model,")
        print("the vectors won't match and re-ingestion with the same model would be needed.")

    return success

if __name__ == "__main__":
    asyncio.run(main())