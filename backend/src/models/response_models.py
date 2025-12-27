from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
from datetime import datetime


class ErrorResponse(BaseModel):
    """
    Standardized error response format.
    """
    error: str
    code: str
    details: Optional[Dict[str, Any]] = None


class Citation(BaseModel):
    """
    Model for citation information.
    """
    text: str
    source: str
    url: Optional[str] = None


class TextChunk(BaseModel):
    """
    Model for a text chunk retrieved from Qdrant.
    """
    id: str
    content: str
    metadata: Dict[str, Any]
    similarity_score: float = Field(ge=0.0, le=1.0)


class QueryModel(BaseModel):
    """
    Model for the incoming query request.
    """
    query: str = Field(min_length=1, max_length=1000)
    max_chunks: Optional[int] = Field(default=5, ge=1, le=10)
    similarity_threshold: Optional[float] = Field(default=0.5, ge=0.0, le=1.0)


class RAGResponse(BaseModel):
    """
    Structured response containing textbook-based content and citations.
    Following the exact schema required to maintain compatibility with existing frontend.
    """
    answer: str
    citations: List[Citation]
    chunks_used: List[TextChunk]
    query: str
    timestamp: str = Field(default_factory=lambda: datetime.utcnow().isoformat())


# Import this for backward compatibility if needed
ResponseModel = RAGResponse