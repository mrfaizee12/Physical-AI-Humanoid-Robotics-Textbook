import pytest
import asyncio
from unittest.mock import patch
from backend.services.retrieval_service import retrieval_service

class TestRetrieval:
    """Tests for vector search functionality"""

    @pytest.mark.asyncio
    async def test_vector_search_contract(self):
        """Contract test for vector search functionality"""
        # Test that the retrieval service has the required methods
        assert hasattr(retrieval_service, 'retrieve_chunks')
        assert callable(retrieval_service.retrieve_chunks)

    @pytest.mark.asyncio
    async def test_vector_search_integration(self):
        """Integration test for vector search"""
        # Mock the actual Qdrant search to avoid needing a real database
        with patch.object(retrieval_service.client, 'search') as mock_search:
            # Mock search results
            mock_search.return_value = [
                type('MockResult', (), {
                    'id': 'test-id-1',
                    'payload': {'content': 'Test content', 'metadata': {'source': 'test-source'}},
                    'score': 0.8
                })()
            ]

            # Test the retrieve_chunks method
            results = await retrieval_service.retrieve_chunks("test query", top_k=5, min_score=0.3)

            # Verify the results structure
            assert isinstance(results, list)
            assert len(results) == 1
            assert 'id' in results[0]
            assert 'content' in results[0]
            assert 'score' in results[0]
            assert results[0]['content'] == 'Test content'

    @pytest.mark.asyncio
    async def test_retrieval_with_different_params(self):
        """Test retrieval with different parameters"""
        with patch.object(retrieval_service.client, 'search') as mock_search:
            # Mock search results
            mock_search.return_value = [
                type('MockResult', (), {
                    'id': 'test-id-1',
                    'payload': {'content': 'Test content 1', 'metadata': {'source': 'test-source-1'}},
                    'score': 0.8
                })(),
                type('MockResult', (), {
                    'id': 'test-id-2',
                    'payload': {'content': 'Test content 2', 'metadata': {'source': 'test-source-2'}},
                    'score': 0.6
                })()
            ]

            # Test with different top_k and min_score values
            results = await retrieval_service.retrieve_chunks(
                "test query",
                top_k=2,
                min_score=0.5
            )

            # Verify results
            assert isinstance(results, list)
            assert len(results) == 2
            for result in results:
                assert 'id' in result
                assert 'content' in result
                assert 'score' in result
                assert result['score'] >= 0.5  # Should respect min_score

if __name__ == "__main__":
    pytest.main([__file__])