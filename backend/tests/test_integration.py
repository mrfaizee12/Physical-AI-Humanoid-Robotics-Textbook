import pytest
import asyncio
from unittest.mock import AsyncMock, patch
from backend.agent import RAGAgent
from backend.services.retrieval_service import retrieval_service

class TestIntegration:
    """Integration tests for the complete RAG agent functionality"""

    @pytest.mark.asyncio
    async def test_complete_agent_workflow(self):
        """Test the complete workflow from query to response"""
        agent = RAGAgent(top_k=3, min_score=0.2)

        # Mock the retrieval service to avoid actual Qdrant calls
        with patch.object(retrieval_service, 'retrieve_chunks') as mock_retrieve:
            mock_retrieve.return_value = [
                {
                    "id": "test-id-1",
                    "content": "The capital of France is Paris.",
                    "score": 0.8,
                    "metadata": {"source": "geography_doc", "page": 12}
                },
                {
                    "id": "test-id-2",
                    "content": "Paris is located in northern France on the Seine river.",
                    "score": 0.7,
                    "metadata": {"source": "geography_doc", "page": 15}
                }
            ]

            # Test a query that should be answerable with the mocked context
            response = await agent.invoke("What is the capital of France?")

            # Verify the response contains expected information
            assert isinstance(response, str)
            assert len(response) > 0
            assert "Paris" in response or "capital" in response  # Response should be relevant

    @pytest.mark.asyncio
    async def test_no_relevant_documents_case(self):
        """Test handling when no relevant documents are found"""
        agent = RAGAgent(top_k=5, min_score=0.8)  # High threshold to trigger no results

        # Mock retrieval to return empty list
        with patch.object(retrieval_service, 'retrieve_chunks') as mock_retrieve:
            mock_retrieve.return_value = []

            response = await agent.invoke("What is the capital of France?")

            # Should return a specific message when no documents found
            assert "couldn't find any relevant information" in response.lower()

    @pytest.mark.asyncio
    async def test_empty_query_handling(self):
        """Test that empty queries are properly rejected"""
        agent = RAGAgent()

        with pytest.raises(Exception):  # Should raise QueryProcessingError
            await agent.invoke("")

    @pytest.mark.asyncio
    async def test_short_query_handling(self):
        """Test that very short queries are properly rejected"""
        agent = RAGAgent()

        with pytest.raises(Exception):  # Should raise QueryProcessingError
            await agent.invoke("hi")

    @pytest.mark.asyncio
    async def test_connection_validation(self):
        """Test the connection validation functionality"""
        agent = RAGAgent()

        # Mock both Qdrant and OpenAI validation
        with patch.object(retrieval_service, 'validate_connection') as mock_qdrant:
            with patch.object(agent.client.chat.completions, 'create') as mock_openai:
                mock_qdrant.return_value = True
                mock_openai.return_value = AsyncMock()

                result = await agent.validate_connection()
                assert result is True

    @pytest.mark.asyncio
    async def test_agent_invoke_signature(self):
        """Test that agent.invoke has the correct signature and behavior"""
        agent = RAGAgent()

        # Verify method exists and is callable
        assert hasattr(agent, 'invoke')
        assert callable(agent.invoke)

        # Test that it returns a string when given proper input (with mocking)
        with patch.object(retrieval_service, 'retrieve_chunks') as mock_retrieve:
            mock_retrieve.return_value = [
                {
                    "id": "test-id-1",
                    "content": "Test content",
                    "score": 0.8,
                    "metadata": {}
                }
            ]

            result = await agent.invoke("Test query")
            assert isinstance(result, str)

if __name__ == "__main__":
    pytest.main([__file__])