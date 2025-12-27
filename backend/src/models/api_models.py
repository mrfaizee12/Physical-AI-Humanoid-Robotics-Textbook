from pydantic import BaseModel, Field
from typing import List, Optional
from datetime import datetime
import uuid

class CitationReference(BaseModel):
    """Model for citation references in responses."""
    id: str = Field(..., description="Unique identifier for the citation")
    text: str = Field(..., description="The cited text from the source")
    sourceUrl: str = Field(..., description="URL to the source document/section")
    pageReference: str = Field(..., description="Page number or section reference")
    context: str = Field(..., description="Surrounding context of the citation")

class QueryRequest(BaseModel):
    """Model for the query request from the frontend."""
    query: str = Field(..., min_length=3, max_length=1000, description="The natural language question from the user")
    userId: Optional[str] = Field(None, description="Optional user identifier for session tracking")
    sessionId: Optional[str] = Field(None, description="Session identifier for tracking")

    def __init__(self, **data):
        super().__init__(**data)
        # Generate a session ID if not provided
        if not self.sessionId:
            self.sessionId = str(uuid.uuid4())

class QueryResponse(BaseModel):
    """Model for the response from the RAG agent."""
    id: str = Field(..., description="Unique identifier for the response")
    queryId: str = Field(..., description="Reference to the original query")
    content: str = Field(..., description="The response text from the RAG agent")
    citations: List[CitationReference] = Field(default_factory=list, description="Sources used in the response")
    confidence: float = Field(..., ge=0.0, le=1.0, description="Confidence score of the response")
    timestamp: str = Field(..., description="When the response was generated")

class ErrorResponse(BaseModel):
    """Model for error responses."""
    error: str = Field(..., description="Error code")
    message: str = Field(..., description="Human-readable error message")
    timestamp: str = Field(..., description="When the error occurred")

    @classmethod
    def create_error(cls, error_code: str, message: str):
        """Create an error response with the current timestamp."""
        return cls(
            error=error_code,
            message=message,
            timestamp=datetime.utcnow().isoformat() + "Z"
        )

class HealthResponse(BaseModel):
    """Model for health check responses."""
    status: str = Field(..., description="Service status")
    service: str = Field(..., description="Name of the service")