from fastapi import APIRouter, HTTPException, Depends
from typing import Optional
from datetime import datetime
from src.services.rag_service import rag_service
from src.models.rag_models import Query as QueryModel, RAGResponseModel
from src.models.response_models import ErrorResponseModel
from src.utils.logging import get_logger
from src.config import settings
import time


# Initialize router
router = APIRouter(prefix="/api")
logger = get_logger(__name__)


@router.post("/rag/query", response_model=RAGResponseModel, responses={
    400: {"model": ErrorResponseModel},
    500: {"model": ErrorResponseModel}
})
async def query_textbook_endpoint(query_data: QueryModel):
    """
    Query textbook content using RAG pipeline with Cohere, Qdrant, and OpenRouter.

    Process a user query against textbook content using RAG pipeline with Cohere, Qdrant, and OpenRouter.
    """
    start_time = time.time()
    logger.info(f"Received RAG query request: {query_data.text[:100]}...")

    try:
        # Validate that required services are configured
        if not all([settings.qdrant_url, settings.cohere_api_key, settings.openrouter_api_key]):
            raise HTTPException(
                status_code=500,
                detail="Missing required configuration for RAG services"
            )

        # Process the query through the RAG pipeline
        response = await rag_service.query_textbook(
            query_text=query_data.text,
            max_chunks=query_data.max_chunks or 5,
            similarity_threshold=query_data.similarity_threshold or 0.5,
            metadata_filters=query_data.metadata_filters
        )

        # Ensure response schema matches exactly what's expected for frontend compatibility
        # The response should maintain the exact structure expected by existing clients
        final_response = RAGResponseModel(
            answer=response.answer,
            citations=response.citations,
            chunks_used=response.chunks_used,
            query=response.query,
            timestamp=response.timestamp
        )

        logger.info(f"Successfully processed query in {time.time() - start_time:.2f}s")
        return final_response

    except HTTPException:
        # Re-raise HTTP exceptions as they are
        raise
    except Exception as e:
        logger.error(f"Error processing RAG query: {e}", exc_info=True)

        # Return the fallback response as specified in requirements
        # Ensure this matches the exact response schema expected by the frontend
        fallback_response = RAGResponseModel(
            answer="I don't know based on the textbook.",
            citations=[],
            chunks_used=[],
            query=query_data.text,
            timestamp=datetime.utcnow().isoformat()  # Use proper datetime format
        )

        return fallback_response


@router.get("/rag/health")
async def health_check():
    """
    Health check endpoint to verify RAG services are available.
    """
    try:
        # Check if textbook content exists
        content_exists = await rag_service.verify_textbook_content_exists()

        return {
            "status": "healthy",
            "textbook_content_available": content_exists,
            "timestamp": time.time()
        }
    except Exception as e:
        logger.error(f"Health check failed: {e}")
        raise HTTPException(status_code=503, detail=f"Service unhealthy: {str(e)}")


# Additional endpoint to get collection info
@router.get("/rag/collection-info")
async def get_collection_info():
    """
    Get information about the textbook collection in Qdrant.
    """
    try:
        info = await rag_service.get_collection_info()
        if info is None:
            raise HTTPException(status_code=404, detail="Collection not found or inaccessible")
        return info
    except Exception as e:
        logger.error(f"Error getting collection info: {e}")
        raise HTTPException(status_code=500, detail=f"Error accessing collection info: {str(e)}")