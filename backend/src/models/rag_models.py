from pydantic import BaseModel, Field
from pydantic.functional_validators import field_validator
from typing import List, Optional, Dict, Any
from datetime import datetime
from .response_models import Citation, TextChunk


class Query(BaseModel):
    """
    Model for user input question that requires textbook-based response.
    """
    text: str = Field(
        min_length=1,
        max_length=1000,
        description="The user's question text"
    )
    max_chunks: Optional[int] = Field(
        default=5,
        ge=1,
        le=10,
        description="Maximum number of text chunks to retrieve"
    )
    similarity_threshold: Optional[float] = Field(
        default=0.5,
        ge=0.0,
        le=1.0,
        description="Minimum similarity score for retrieved chunks"
    )
    metadata_filters: Optional[Dict[str, Any]] = Field(
        default=None,
        description="Optional filters for metadata fields (e.g., {'module': 'Module 1'})"
    )

    @field_validator('text')
    @classmethod
    def validate_text(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('Query text cannot be empty')
        if len(v) > 1000:
            raise ValueError('Query text must not exceed 1000 characters')
        return v.strip()

    @field_validator('max_chunks')
    @classmethod
    def validate_max_chunks(cls, v):
        if v is not None:
            if v < 1 or v > 10:
                raise ValueError('max_chunks must be between 1 and 10')
        return v

    @field_validator('similarity_threshold')
    @classmethod
    def validate_similarity_threshold(cls, v):
        if v is not None:
            if v < 0.0 or v > 1.0:
                raise ValueError('similarity_threshold must be between 0.0 and 1.0')
        return v


class Embedding(BaseModel):
    """
    Model for vector representation of user query for similarity search.
    """
    vector: List[float]
    model: str
    dimension: int


class RAGResponseModel(BaseModel):
    """
    Model for structured response containing textbook-based content and citations.
    """
    answer: str
    citations: List[Citation]
    chunks_used: List[TextChunk]
    query: str
    timestamp: str = Field(default_factory=lambda: datetime.utcnow().isoformat())


class ErrorResponseModel(BaseModel):
    """
    Model for standardized error response format.
    """
    error: str
    code: str
    details: Optional[Dict[str, Any]] = None