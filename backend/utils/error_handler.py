import logging
from typing import Any
from fastapi import HTTPException, status
from pydantic import BaseModel

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class ErrorResponse(BaseModel):
    """Standard error response model"""
    error: str
    detail: Any = None
    timestamp: str = None

class RAGAgentError(Exception):
    """Base exception class for RAG agent errors"""
    def __init__(self, message: str, detail: Any = None):
        self.message = message
        self.detail = detail
        super().__init__(self.message)

class QueryProcessingError(RAGAgentError):
    """Exception raised when query processing fails"""
    pass

class RetrievalError(RAGAgentError):
    """Exception raised when document retrieval fails"""
    pass

class ResponseGenerationError(RAGAgentError):
    """Exception raised when response generation fails"""
    pass

def handle_error(error: Exception, context: str = "") -> ErrorResponse:
    """
    Handle an error and return a standard error response

    Args:
        error: The exception that occurred
        context: Additional context about where the error occurred

    Returns:
        ErrorResponse with error details
    """
    import datetime
    timestamp = datetime.datetime.now().isoformat()

    error_msg = f"{context}: {str(error)}" if context else str(error)
    logger.error(error_msg, exc_info=True)  # Log with traceback

    return ErrorResponse(
        error=type(error).__name__,
        detail=str(error),
        timestamp=timestamp
    )

def create_http_exception(
    status_code: int,
    detail: str,
    headers: dict = None
) -> HTTPException:
    """
    Create an HTTP exception with proper error handling

    Args:
        status_code: HTTP status code
        detail: Error detail message
        headers: Optional headers to include

    Returns:
        HTTPException instance
    """
    return HTTPException(
        status_code=status_code,
        detail=detail,
        headers=headers
    )