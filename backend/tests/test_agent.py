import pytest
import asyncio
from unittest.mock import AsyncMock, patch
from backend.agent import RAGAgent
from backend.models.text_query import TextQuery
from backend.models.retrieved_chunk import RetrievedChunk

class TestAgent:
    """Tests for the RAG agent functionality"""

    @pytest.mark.asyncio
    async def test_agent_invoke_contract(self):
        """Contract test for agent.invoke() interface"""
        agent = RAGAgent()

        # Test that the interface exists and accepts a string
        result = await agent.invoke("Test query")

        # Should return a string response
        assert isinstance(result, str)
        assert len(result) > 0

    @pytest.mark.asyncio
    async def test_query_to_response_pipeline(self):
        """Integration test for query-to-response pipeline"""
        agent = RAGAgent()

        # Mock the retrieval service to avoid actual Qdrant calls
        with patch('backend.services.retrieval_service.retrieval_service.retrieve_chunks') as mock_retrieve:
            mock_retrieve.return_value = [
                {
                    "id": "test-id-1",
                    "content": "This is a relevant document chunk",
                    "score": 0.8,
                    "metadata": {"source": "test-doc"}
                }
            ]

            # Test the pipeline
            response = await agent.invoke("What is the capital of France?")

            # Should return a response string
            assert isinstance(response, str)
            assert "Paris" in response or len(response) > 0  # Basic check

    @pytest.mark.asyncio
    async def test_empty_query_handling(self):
        """Test that the agent handles empty queries appropriately"""
        agent = RAGAgent()

        with pytest.raises(ValueError):
            await agent.invoke("")

    @pytest.mark.asyncio
    async def test_short_query_handling(self):
        """Test that the agent handles very short queries appropriately"""
        agent = RAGAgent()

        with pytest.raises(ValueError):
            await agent.invoke("hi")

if __name__ == "__main__":
    pytest.main([__file__])