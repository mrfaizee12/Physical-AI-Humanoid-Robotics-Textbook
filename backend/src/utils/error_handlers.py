from fastapi import Request, HTTPException
from fastapi.responses import JSONResponse
import logging
from typing import Dict, Any
import traceback
from datetime import datetime

logger = logging.getLogger(__name__)

class RAGException(Exception):
    """Base exception class for RAG-related errors."""

    def __init__(self, message: str, error_code: str = "GENERAL_ERROR", status_code: int = 500):
        self.message = message
        self.error_code = error_code
        self.status_code = status_code
        super().__init__(self.message)

class QueryValidationException(RAGException):
    """Exception raised when query validation fails."""

    def __init__(self, message: str = "Invalid query format"):
        super().__init__(message, "INVALID_QUERY", 400)

class ServiceUnavailableException(RAGException):
    """Exception raised when the RAG service is unavailable."""

    def __init__(self, message: str = "RAG agent is temporarily unavailable"):
        super().__init__(message, "SERVICE_UNAVAILABLE", 503)

class TimeoutException(RAGException):
    """Exception raised when a request times out."""

    def __init__(self, message: str = "Request timed out"):
        super().__init__(message, "REQUEST_TIMEOUT", 408)

async def http_exception_handler(request: Request, exc: HTTPException):
    """Handle HTTP exceptions."""
    logger.error(f"HTTP Exception: {exc.status_code} - {exc.detail}")

    # Log additional request context
    logger.info(f"Request path: {request.url.path}")
    logger.info(f"Request method: {request.method}")

    return JSONResponse(
        status_code=exc.status_code,
        content={
            "error": f"HTTP_{exc.status_code}",
            "message": str(exc.detail),
            "timestamp": datetime.utcnow().isoformat() + "Z"
        }
    )

async def rag_exception_handler(request: Request, exc: RAGException):
    """Handle custom RAG exceptions."""
    logger.error(f"RAG Exception: {exc.error_code} - {exc.message}")

    # Log additional request context
    logger.info(f"Request path: {request.url.path}")
    logger.info(f"Request method: {request.method}")

    return JSONResponse(
        status_code=exc.status_code,
        content={
            "error": exc.error_code,
            "message": exc.message,
            "timestamp": datetime.utcnow().isoformat() + "Z"
        }
    )

async def general_exception_handler(request: Request, exc: Exception):
    """Handle general exceptions."""
    logger.error(f"General Exception: {str(exc)}", exc_info=True)

    # Log additional request context and full traceback
    logger.info(f"Request path: {request.url.path}")
    logger.info(f"Request method: {request.method}")
    logger.error(f"Full traceback: {traceback.format_exc()}")

    return JSONResponse(
        status_code=500,
        content={
            "error": "INTERNAL_ERROR",
            "message": "An internal server error occurred",
            "timestamp": datetime.utcnow().isoformat() + "Z"
        }
    )

def register_exception_handlers(app):
    """Register all exception handlers with the FastAPI app."""
    app.add_exception_handler(HTTPException, http_exception_handler)
    app.add_exception_handler(RAGException, rag_exception_handler)
    app.add_exception_handler(Exception, general_exception_handler)