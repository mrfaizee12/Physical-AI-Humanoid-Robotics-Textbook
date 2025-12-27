import asyncio
from typing import List, Optional, Dict, Any
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams, PointStruct
from src.config import settings
from src.utils.logging import get_logger


class QdrantService:
    """
    Service class to handle interactions with Qdrant vector database.
    """

    def __init__(self):
        """
        Initialize Qdrant client with configuration from settings.
        """
        # Determine the appropriate client initialization based on URL format
        if settings.qdrant_url.startswith("http"):
            # Handle HTTP URLs
            self.client = QdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key,
                prefer_grpc=False
            )
        elif "localhost" in settings.qdrant_url or "." in settings.qdrant_url:
            # Handle host:port or host formats
            if ":" in settings.qdrant_url and not settings.qdrant_url.startswith("http"):
                try:
                    host, port = settings.qdrant_url.split(":")
                    self.client = QdrantClient(
                        host=host,
                        port=int(port),
                        api_key=settings.qdrant_api_key
                    )
                except ValueError:
                    # If it's just a host without port, try default port 6333
                    self.client = QdrantClient(
                        host=settings.qdrant_url,
                        api_key=settings.qdrant_api_key
                    )
            else:
                self.client = QdrantClient(
                    host=settings.qdrant_url,
                    api_key=settings.qdrant_api_key
                )
        else:
            # Assume it's a local instance
            self.client = QdrantClient(
                host=settings.qdrant_url,
                api_key=settings.qdrant_api_key
            )

        self.collection_name = settings.qdrant_collection_name

    async def verify_collection_exists(self) -> bool:
        """
        Verify that the required collection exists in Qdrant.
        """
        try:
            collections = self.client.get_collections()
            collection_names = [collection.name for collection in collections.collections]
            return self.collection_name in collection_names
        except Exception as e:
            print(f"Error checking collection existence: {e}")
            return False

    async def search_text_chunks(self, query_vector: List[float], limit: int = 5, similarity_threshold: float = 0.5, metadata_filters: Optional[Dict[str, Any]] = None) -> List[Dict[str, Any]]:
        """
        Search for text chunks in Qdrant that are similar to the query vector with optional metadata filtering.

        Args:
            query_vector: The embedding vector to search for
            limit: Maximum number of results to return
            similarity_threshold: Minimum similarity score for results
            metadata_filters: Optional filters for metadata fields (e.g., {'module': 'Module 1'})

        Returns:
            List of text chunks with metadata
        """
        try:
            # Build optional filters for metadata
            search_filter = None
            if metadata_filters:
                from qdrant_client.http import models as qdrant_models

                filter_conditions = []
                for key, value in metadata_filters.items():
                    if value is not None:
                        filter_conditions.append(
                            qdrant_models.FieldCondition(
                                key=key,
                                match=qdrant_models.MatchValue(value=str(value))
                            )
                        )

                if filter_conditions:
                    search_filter = qdrant_models.Filter(
                        must=filter_conditions
                    )

            # Perform the search in Qdrant using query_points (new method)
            search_results = self.client.query_points(
                collection_name=self.collection_name,
                query=query_vector,
                limit=limit,
                with_payload=True,
                query_filter=search_filter  # Add the metadata filter
            )

            # Add debug logging
            points_returned = len(search_results.points)
            logger = get_logger(__name__)
            logger.info(f"Qdrant query returned {points_returned} points")

            # If points were returned, log the top score
            if search_results.points:
                top_score = search_results.points[0].score
                logger.info(f"Top raw score (distance) value: {top_score}")

            # Format the results to match our TextChunk model
            # Convert distance to similarity (cosine distance to similarity)
            chunks = []
            for result in search_results.points:
                # Convert distance to similarity: similarity = max(0.0, 1 - distance)
                # This ensures similarity is never negative
                raw_distance = result.score
                # For cosine distance, similarity = 1 - distance, but ensure it's in [0,1] range
                similarity = max(0.0, min(1.0, 1 - raw_distance))

                chunk = {
                    "id": str(result.id),
                    "content": result.payload.get("content", ""),
                    "metadata": {
                        "title": result.payload.get("title", ""),  # Include title as required
                        "url": result.payload.get("url", ""),
                        "chunk_index": result.payload.get("chunk_index", 0),
                        "source": result.payload.get("source", ""),
                        "page_number": result.payload.get("page_number"),
                        "section_title": result.payload.get("section_title", ""),
                        "citation": result.payload.get("citation", ""),
                        "module": result.payload.get("module", ""),  # Add module information if available
                        "section": result.payload.get("section", "")  # Add section information if available
                    },
                    "similarity_score": similarity  # Now using proper similarity score
                }
                chunks.append(chunk)

            # Apply minimum similarity threshold filter
            min_similarity = similarity_threshold  # Use the provided threshold parameter instead of hardcoded 0.25
            filtered_chunks = [chunk for chunk in chunks if chunk["similarity_score"] >= min_similarity]

            logger.info(f"After applying min similarity threshold ({min_similarity}), {len(filtered_chunks)} chunks remain")

            return filtered_chunks
        except Exception as e:
            logger = get_logger(__name__)
            logger.error(f"Error searching Qdrant: {e}")
            return []

    async def get_all_collections(self) -> List[str]:
        """
        Get a list of all collection names in Qdrant.
        """
        try:
            collections = self.client.get_collections()
            return [collection.name for collection in collections.collections]
        except Exception as e:
            print(f"Error getting collections: {e}")
            return []

    async def get_collection_info(self, collection_name: Optional[str] = None) -> Optional[Dict[str, Any]]:
        """
        Get information about a specific collection.
        """
        try:
            cname = collection_name or self.collection_name
            info = self.client.get_collection(cname)
            return {
                "name": info.config.params.vectors.size,
                "vector_size": info.config.params.vectors.size,
                "distance": info.config.params.vectors.distance,
                "point_count": info.points_count
            }
        except Exception as e:
            print(f"Error getting collection info: {e}")
            return None


# Singleton instance
qdrant_service = QdrantService()