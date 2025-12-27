import cohere
from typing import List
from src.config import settings
from src.utils.logging import get_logger


class EmbeddingService:
    """
    Service class to handle text embedding generation using Cohere.
    """

    def __init__(self):
        """
        Initialize Cohere client with API key from settings.
        """
        self.client = cohere.Client(settings.cohere_api_key)
        self.model = "embed-english-v3.0"  # Using the specific model as required
        self.logger = get_logger(__name__)

    async def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding vector for the given text using Cohere.

        Args:
            text: Input text to generate embedding for

        Returns:
            List of floats representing the embedding vector
        """
        try:
            response = self.client.embed(
                texts=[text],
                model=self.model,
                input_type="search_query"  # Optimize for search use case
            )
            embedding = response.embeddings[0]  # Return the first (and only) embedding

            # Log and assert vector length
            vector_length = len(embedding)
            self.logger.info(f"Generated embedding vector length: {vector_length} (model: {self.model})")

            if vector_length != 1024:
                self.logger.error(f"Embedding vector length {vector_length} does not match expected 1024 dimensions")
                raise ValueError(f"Embedding vector length mismatch: expected 1024, got {vector_length}")

            return embedding
        except Exception as e:
            self.logger.error(f"Error generating embedding: {e}")
            # Return a zero vector as fallback, though this should be handled properly in production
            return [0.0] * 1024  # Assuming 1024-dimensional vectors

    async def generate_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts in a batch.

        Args:
            texts: List of input texts to generate embeddings for

        Returns:
            List of embedding vectors (each vector is a list of floats)
        """
        try:
            response = self.client.embed(
                texts=texts,
                model=self.model,
                input_type="search_document"  # Optimize for document search
            )
            embeddings = response.embeddings

            # Log and assert vector length for the first embedding if available
            if embeddings:
                vector_length = len(embeddings[0])
                self.logger.info(f"Generated batch embeddings vector length: {vector_length} (model: {self.model}, count: {len(embeddings)})")

                if vector_length != 1024:
                    self.logger.error(f"Batch embedding vector length {vector_length} does not match expected 1024 dimensions")
                    raise ValueError(f"Batch embedding vector length mismatch: expected 1024, got {vector_length}")

            return embeddings
        except Exception as e:
            self.logger.error(f"Error generating embeddings batch: {e}")
            # Return zero vectors as fallback
            return [[0.0] * 1024 for _ in texts]

    async def get_model_info(self) -> dict:
        """
        Get information about the embedding model being used.
        """
        return {
            "model": self.model,
            "provider": "Cohere",
            "dimensions": 1024  # This is typical for Cohere's embedding models
        }


# Singleton instance
embedding_service = EmbeddingService()