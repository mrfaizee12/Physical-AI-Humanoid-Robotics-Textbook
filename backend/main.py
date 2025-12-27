from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from src.api.rag_routes import router as rag_router
from src.utils.logging import setup_logging
from src.config import settings


# Set up logging first
setup_logging()

# Create FastAPI app
app = FastAPI(
    title="Real RAG Pipeline API",
    description="API for querying textbook content using Retrieval-Augmented Generation",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, configure specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routes
app.include_router(rag_router)

# Root endpoint
@app.get("/")
async def root():
    return {
        "message": "Real RAG Pipeline API",
        "status": "running",
        "endpoints": ["/api/rag/query", "/api/rag/health"]
    }

# Health check endpoint
@app.get("/health")
async def health():
    return {"status": "healthy", "service": "Real RAG Pipeline"}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=True
    )
