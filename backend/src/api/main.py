from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
from typing import AsyncGenerator
import logging

# Import error handlers
from src.utils.error_handlers import register_exception_handlers

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncGenerator[None, None]:
    """
    Lifecycle events for the application.
    """
    logger.info("Starting up RAG API service...")
    yield
    logger.info("Shutting down RAG API service...")

# Create FastAPI app instance
app = FastAPI(
    title="RAG Agent API",
    description="API for RAG agent integration with textbook content",
    version="1.0.0",
    lifespan=lifespan
)

# Configure CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "http://127.0.0.1:3000",
        "https://physical-ai-humanoid-robotics-textb-nu.vercel.app"
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Register exception handlers
register_exception_handlers(app)

# Import and include routers here
# from .routers import rag_router
# app.include_router(rag_router, prefix="/api/rag", tags=["rag"])

# Import models and services for the query endpoint
from src.models.api_models import QueryRequest, QueryResponse
from src.services.rag_service import SimpleRAGService
from src.utils.error_handlers import QueryValidationException, ServiceUnavailableException

# Initialize the RAG service
rag_service = SimpleRAGService()

@app.post("/api/rag/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest):
    """Process a query and return a grounded response with citations."""
    # Validate the query
    is_valid = await rag_service.validate_query(request.query)
    if not is_valid:
        raise QueryValidationException("Query must be at least 3 characters long and not exceed 1000 characters")

    # Process the query
    try:
        response = await rag_service.query(request.query, request.userId, request.sessionId)
        return response
    except Exception as e:
        raise ServiceUnavailableException(f"Unable to process query: {str(e)}")

@app.get("/")
async def root_endpoint():
    """Root endpoint to confirm the API is running."""
    return {
        "status": "RAG API is running",
        "docs": "/docs",
        "query_endpoint": "/api/rag/query"
    }


@app.get("/health")
async def health_check():
    """Health check endpoint to verify the service is running."""
    from src.models.api_models import HealthResponse
    return HealthResponse(status="healthy", service="RAG Agent API")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)