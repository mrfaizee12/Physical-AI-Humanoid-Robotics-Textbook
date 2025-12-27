import logging
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from utils.embedding_utils import get_embedding, preprocess_query
from config.settings import settings

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class QdrantRetrievalService:
    """Service for retrieving relevant document chunks from Qdrant vector database"""

    def __init__(self):
        """Initialize the Qdrant client with connection settings"""
        try:
            if settings.qdrant.url.startswith("https://"):
                # Use HTTPS connection with API key
                self.client = QdrantClient(
                    url=settings.qdrant.url,
                    api_key=settings.qdrant.api_key,
                    prefer_grpc=False  # Using REST API
                )
            else:
                # Use local/insecure connection
                self.client = QdrantClient(
                    host=settings.qdrant.url,
                    api_key=settings.qdrant.api_key
                )

            self.collection_name = settings.qdrant.collection_name
            logger.info(f"Qdrant client initialized for collection: {self.collection_name}")
        except Exception as e:
            logger.error(f"Failed to initialize Qdrant client: {str(e)}")
            raise

    async def retrieve_chunks(
        self,
        query_text: str,
        top_k: int = 5,
        min_score: float = 0.3,
        filters: Optional[Dict[str, Any]] = None
    ) -> List[Dict[str, Any]]:
        """
        Retrieve relevant document chunks from Qdrant based on query text.

        Args:
            query_text: The query text to search for
            top_k: Number of chunks to retrieve (default: 5)
            min_score: Minimum similarity score for retrieved chunks (default: 0.3)
            filters: Optional filters to apply to the search

        Returns:
            List of retrieved chunks with content, score, and metadata
        """
        try:
            # Preprocess the query
            processed_query = await preprocess_query(query_text)

            # Generate embedding for the query
            query_embedding = await get_embedding(processed_query)

            # Prepare filters if provided
            qdrant_filters = None
            if filters:
                qdrant_filters = self._convert_to_qdrant_filter(filters)

            # Perform search in Qdrant
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k,
                score_threshold=min_score,
                query_filter=qdrant_filters
            )

            # Format results
            retrieved_chunks = []
            for result in search_results:
                chunk = {
                    "id": result.id,
                    "content": result.payload.get("content", ""),
                    "score": result.score,
                    "metadata": result.payload.get("metadata", {}),
                    "payload": result.payload  # Include full payload for additional data
                }
                retrieved_chunks.append(chunk)

            logger.info(f"Retrieved {len(retrieved_chunks)} chunks for query: {query_text[:50]}...")
            return retrieved_chunks

        except Exception as e:
            logger.error(f"Error retrieving chunks: {str(e)}")
            raise

    def _convert_to_qdrant_filter(self, filters: Dict[str, Any]) -> models.Filter:
        """
        Convert a dictionary of filters to Qdrant Filter object.

        Args:
            filters: Dictionary of field-value pairs to filter on

        Returns:
            Qdrant Filter object
        """
        conditions = []
        for field, value in filters.items():
            if isinstance(value, list):
                # Handle multiple values for the same field (OR condition)
                field_conditions = [models.FieldCondition(
                    key=field,
                    match=models.MatchValue(value=v)
                ) for v in value]
                conditions.append(models.Should(conditions=field_conditions))
            else:
                # Handle single value
                conditions.append(models.FieldCondition(
                    key=field,
                    match=models.MatchValue(value=value)
                ))

        return models.Filter(must=conditions)

    async def retrieve_chunks_with_payload(
        self,
        query_text: str,
        top_k: int = 5,
        min_score: float = 0.3,
        with_payload: bool = True,
        with_vectors: bool = False
    ) -> List[Dict[str, Any]]:
        """
        Retrieve chunks with more control over what data is returned.

        Args:
            query_text: The query text to search for
            top_k: Number of chunks to retrieve
            min_score: Minimum similarity score
            with_payload: Whether to return payload data
            with_vectors: Whether to return the vector embeddings

        Returns:
            List of retrieved chunks
        """
        try:
            # Preprocess the query
            processed_query = await preprocess_query(query_text)

            # Generate embedding for the query
            query_embedding = await get_embedding(processed_query)

            # Perform search in Qdrant with specified options
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k,
                score_threshold=min_score,
                with_payload=with_payload,
                with_vectors=with_vectors
            )

            # Format results
            retrieved_chunks = []
            for result in search_results:
                chunk = {
                    "id": result.id,
                    "content": result.payload.get("content", "") if result.payload else "",
                    "score": result.score,
                    "metadata": result.payload.get("metadata", {}) if result.payload else {},
                    "payload": result.payload if result.payload else {}
                }
                retrieved_chunks.append(chunk)

            logger.info(f"Retrieved {len(retrieved_chunks)} chunks with payload for query: {query_text[:50]}...")
            return retrieved_chunks

        except Exception as e:
            logger.error(f"Error retrieving chunks with payload: {str(e)}")
            raise

    async def get_available_payload_keys(self) -> List[str]:
        """
        Get the available payload keys in the collection.

        Returns:
            List of available payload keys
        """
        try:
            collection_info = self.client.get_collection(self.collection_name)
            # Note: This is a simplified approach; in practice, you might need to
            # sample points to determine all possible keys
            return ["content", "metadata", "source", "document_id"]  # Common keys
        except Exception as e:
            logger.error(f"Error getting payload keys: {str(e)}")
            return ["content", "metadata"]

    async def validate_connection(self) -> bool:
        """
        Validate that we can connect to Qdrant and access the collection.

        Returns:
            True if connection is valid, False otherwise
        """
        try:
            # Try to get collection info
            collection_info = self.client.get_collection(self.collection_name)
            logger.info(f"Successfully connected to collection: {self.collection_name}")
            logger.info(f"Collection vectors count: {collection_info.points_count}")
            return True
        except Exception as e:
            logger.error(f"Failed to validate Qdrant connection: {str(e)}")
            return False

    async def get_collection_stats(self) -> Dict[str, Any]:
        """
        Get statistics about the Qdrant collection.

        Returns:
            Dictionary with collection statistics
        """
        try:
            collection_info = self.client.get_collection(self.collection_name)
            return {
                "vectors_count": collection_info.points_count,
                "config": collection_info.config,
                "payload_schema": collection_info.payload_schema
            }
        except Exception as e:
            logger.error(f"Error getting collection stats: {str(e)}")
            raise

# Global instance of the retrieval service
retrieval_service = QdrantRetrievalService()