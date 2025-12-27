"""
Test script to verify RAG functionality
"""
import asyncio
import os
import sys
from pathlib import Path

# Add backend to path
sys.path.insert(0, str(Path(__file__).parent / "backend"))

from src.services.rag_service import rag_service
from src.utils.logging import get_logger

logger = get_logger(__name__)


async def test_rag_functionality():
    """
    Test the RAG functionality to ensure it's working properly
    """
    print("🔍 Testing RAG functionality...")

    # Test if textbook content is available
    content_exists = await rag_service.verify_textbook_content_exists()
    if not content_exists:
        print("❌ Textbook content not found in Qdrant")
        print("💡 Run the ingestion pipeline first: python -m src.ingestion.textbook_ingestor")
        return False

    print("✅ Textbook content is available in Qdrant")

    # Get collection info
    info = await rag_service.get_collection_info()
    if info:
        print(f"📊 Collection info: {info['point_count']} vectors, {info['vector_size']} dimensions")

    # Test a sample query
    print("\n🧪 Testing sample query...")
    try:
        response = await rag_service.query_textbook(
            query_text="What is this textbook about?",
            max_chunks=3,
            similarity_threshold=0.25
        )

        print(f"✅ Query successful!")
        print(f"❓ Query: {response.query}")
        print(f"📝 Answer: {response.answer[:200]}...")
        print(f"🔗 Citations: {len(response.citations)} found")

        if response.citations:
            for i, citation in enumerate(response.citations[:2]):  # Show first 2 citations
                print(f"  Citation {i+1}: {citation.source} - {citation.url}")

        return True

    except Exception as e:
        print(f"❌ Error during query test: {e}")
        return False


async def main():
    print("📚 RAG Pipeline Test Suite")
    print("=" * 40)

    success = await test_rag_functionality()

    print("\n" + "=" * 40)
    if success:
        print("🎉 All tests passed! RAG pipeline is working correctly.")
    else:
        print("❌ Some tests failed. Please check the configuration and ingestion.")
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())