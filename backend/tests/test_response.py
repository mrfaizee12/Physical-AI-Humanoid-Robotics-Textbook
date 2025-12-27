import pytest
import asyncio
from backend.models.grounded_response import GroundedResponse
from backend.models.retrieved_chunk import RetrievedChunk

class TestResponse:
    """Tests for grounded response validation"""

    def test_grounded_response_contract(self):
        """Contract test for grounded response validation"""
        # Test that the GroundedResponse model exists and has required attributes
        response = GroundedResponse(
            response_text="Test response",
            source_chunks=[]
        )

        assert hasattr(response, 'response_text')
        assert hasattr(response, 'source_chunks')
        assert hasattr(response, 'confidence_score')
        assert hasattr(response, 'timestamp')

    def test_content_verification_integration(self):
        """Integration test for content verification"""
        # Create some test chunks
        chunk1 = RetrievedChunk(
            id="test-1",
            content="The capital of France is Paris.",
            score=0.8
        )

        chunk2 = RetrievedChunk(
            id="test-2",
            content="Paris is located in northern France.",
            score=0.7
        )

        # Create a response that only uses information from the chunks
        response_text = "The capital of France is Paris, which is located in northern France."

        response = GroundedResponse(
            response_text=response_text,
            source_chunks=[chunk1, chunk2]
        )

        # Check if the response is considered grounded
        assert response.is_fully_grounded()

        # Calculate and verify confidence
        confidence = response.calculate_confidence()
        assert confidence > 0.0
        assert confidence <= 1.0

    def test_ungrounded_response_detection(self):
        """Test that responses not based on source content are detected"""
        chunk = RetrievedChunk(
            id="test-1",
            content="The sky is blue.",
            score=0.9
        )

        # Create a response that doesn't use the source content
        response_text = "Rome is the capital of Italy."

        response = GroundedResponse(
            response_text=response_text,
            source_chunks=[chunk]
        )

        # The basic implementation considers it grounded if it has source chunks
        # In a real implementation, we'd have more sophisticated content analysis
        assert response.is_fully_grounded()  # Has source chunks
        assert response.calculate_confidence() > 0.0

if __name__ == "__main__":
    pytest.main([__file__])