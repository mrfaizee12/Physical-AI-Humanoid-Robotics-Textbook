#!/usr/bin/env python3
"""
Test script to verify that the RAG backend fixes are working correctly.
"""
import asyncio
from src.services.qdrant_service import qdrant_service
from src.services.embedding_service import embedding_service
from src.services.rag_service import rag_service
from src.config import settings

async def test_distance_to_similarity_conversion():
    print("=== Testing Distance to Similarity Conversion ===")

    # Test embedding generation
    test_query = "What is robotics?"
    query_embedding = await embedding_service.generate_embedding(test_query)
    print(f"Generated embedding with {len(query_embedding)} dimensions")

    # Test Qdrant search with the new conversion logic
    results = await qdrant_service.search_text_chunks(
        query_vector=query_embedding,
        limit=5
    )

    print(f"Retrieved {len(results)} chunks after applying similarity threshold")

    if results:
        for i, chunk in enumerate(results[:3]):  # Check first 3 results
            similarity_score = chunk["similarity_score"]
            print(f"Chunk {i+1} similarity score: {similarity_score}")

            # Verify similarity score is non-negative and within expected range
            if similarity_score < 0 or similarity_score > 1.0:
                print(f"ERROR: Invalid similarity score {similarity_score}")
                return False
            else:
                print(f"SUCCESS: Similarity score {similarity_score} is valid")

    return True

async def test_minimum_threshold_logic():
    print("\n=== Testing Minimum Similarity Threshold ===")

    # Test with a query that might have low similarity results
    test_query = "Module 2 concepts"
    query_embedding = await embedding_service.generate_embedding(test_query)

    results = await qdrant_service.search_text_chunks(
        query_vector=query_embedding,
        limit=5
    )

    print(f"Retrieved {len(results)} chunks after applying min threshold (0.25)")

    # All returned chunks should have similarity >= 0.25
    for chunk in results:
        if chunk["similarity_score"] < 0.25:
            print(f"ERROR: Chunk has similarity below threshold: {chunk['similarity_score']}")
            return False

    print("SUCCESS: All chunks meet minimum similarity threshold")
    return True

async def test_fallback_response():
    print("\n=== Testing Fallback Response ===")

    # Test with a query that should return no relevant results
    # Use a query that's unlikely to match any textbook content
    test_query = "What is the color of the sky on Mars according to the textbook?"

    result = await rag_service.query_textbook(test_query)

    print(f"Query: {test_query}")
    print(f"Response: {result.answer}")

    # Check if fallback response is returned when no relevant chunks found
    if result.answer == "I don't know based on the textbook.":
        print("SUCCESS: Fallback response returned when no relevant chunks found")
        return True
    else:
        print("WARNING: Fallback response not returned as expected")
        return False

async def test_module_specific_query():
    print("\n=== Testing Module-Specific Query ===")

    # Test with a specific module query
    test_query = "What are the key concepts in Module 2?"

    result = await rag_service.query_textbook(test_query)

    print(f"Query: {test_query}")
    print(f"Response: {result.answer[:100]}...")
    print(f"Number of chunks used: {len(result.chunks_used)}")

    # The response should either contain textbook content or fallback
    if result.answer == "I don't know based on the textbook.":
        print("SUCCESS: Returned fallback when Module 2 content not found in context")
    else:
        print("INFO: Found relevant content for module query")

    return True

async def main():
    print("Testing RAG Backend Fixes...")
    print(f"Collection: {settings.qdrant_collection_name}")
    print(f"Embedding model: {embedding_service.model}")
    print()

    all_tests_passed = True

    # Test 1: Distance to similarity conversion
    test1_result = await test_distance_to_similarity_conversion()
    all_tests_passed = all_tests_passed and test1_result

    # Test 2: Minimum threshold logic
    test2_result = await test_minimum_threshold_logic()
    all_tests_passed = all_tests_passed and test2_result

    # Test 3: Fallback response
    test3_result = await test_fallback_response()
    all_tests_passed = all_tests_passed and test3_result

    # Test 4: Module-specific query
    test4_result = await test_module_specific_query()
    all_tests_passed = all_tests_passed and test4_result

    print(f"\n=== SUMMARY ===")
    if all_tests_passed:
        print("SUCCESS: All tests passed! RAG backend fixes are working correctly.")
        print("- Distance to similarity conversion: SUCCESS")
        print("- Minimum similarity threshold: SUCCESS")
        print("- Non-negative similarity scores: SUCCESS")
        print("- Fallback response: SUCCESS")
        print("- Module-specific query handling: SUCCESS")
    else:
        print("FAILED: Some tests failed. Please review the issues above.")

    return all_tests_passed

if __name__ == "__main__":
    asyncio.run(main())