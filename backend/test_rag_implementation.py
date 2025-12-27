"""
Basic test to verify the RAG pipeline implementation works as expected.
"""
import asyncio
import os
from src.services.rag_service import rag_service
from src.models.rag_models import Query


async def test_rag_implementation():
    """
    Test the RAG pipeline implementation with a sample query.
    """
    print("Testing RAG Pipeline Implementation...")

    # Check if required environment variables are set
    required_vars = ["QDRANT_URL", "QDRANT_API_KEY", "COHERE_API_KEY", "GEMINI_API_KEY"]
    missing_vars = [var for var in required_vars if not os.getenv(var)]

    if missing_vars:
        print(f"Warning: Missing environment variables: {missing_vars}")
        print("Please set these variables to fully test the implementation.")
        print("Testing will proceed with validation checks only.")

    # Test 1: Verify textbook content exists
    print("\n1. Checking if textbook content exists in Qdrant...")
    content_exists = await rag_service.verify_textbook_content_exists()
    print(f"   Textbook content available: {content_exists}")

    if content_exists:
        # Test 2: Process a sample query
        print("\n2. Testing with a sample query...")
        sample_query = "What are the key principles of humanoid robotics?"

        response = await rag_service.query_textbook(
            query_text=sample_query,
            max_chunks=3,
            similarity_threshold=0.3
        )

        print(f"   Query: {sample_query}")
        print(f"   Answer: {response.answer[:100]}...")
        print(f"   Citations: {len(response.citations)} found")
        print(f"   Chunks used: {len(response.chunks_used)}")
        print(f"   Response schema: OK")
    else:
        # Test the fallback response
        print("\n2. Testing fallback response (no content available)...")
        fallback_query = "Test query for fallback"

        response = await rag_service.query_textbook(
            query_text=fallback_query
        )

        expected_fallback = "I don't know based on the textbook."
        if response.answer == expected_fallback:
            print(f"   Fallback response correct: {response.answer}")
        else:
            print(f"   Unexpected response: {response.answer}")

    # Test 3: Verify response schema
    print("\n3. Verifying response schema...")
    schema_attributes = ['answer', 'citations', 'chunks_used', 'query', 'timestamp']
    response_attributes = [attr for attr in schema_attributes if hasattr(response, attr)]

    if len(response_attributes) == len(schema_attributes):
        print("   Response schema: OK - all required attributes present")
    else:
        print(f"   Response schema: Missing attributes: {set(schema_attributes) - set(response_attributes)}")

    print("\nTest completed!")


if __name__ == "__main__":
    asyncio.run(test_rag_implementation())