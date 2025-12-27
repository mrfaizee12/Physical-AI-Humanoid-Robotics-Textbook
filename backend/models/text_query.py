from datetime import datetime
from typing import List, Optional
from pydantic import BaseModel, validator

class TextQuery(BaseModel):
    """
    Model representing a user's text query that requires an answer based on retrieved knowledge
    """
    query_text: str
    query_embedding: Optional[List[float]] = None
    timestamp: datetime = datetime.now()

    class Config:
        # Allow arbitrary types for embedding (list of floats)
        arbitrary_types_allowed = True

    @validator('query_text')
    def validate_query_text(cls, v):
        """Validate that query text is not empty and contains meaningful content"""
        if not v or not v.strip():
            raise ValueError('Query text cannot be empty or just whitespace')
        if len(v.strip()) < 3:
            raise ValueError('Query text is too short, must be at least 3 characters')
        return v.strip()

    def is_valid(self) -> bool:
        """Check if the query is valid based on our validation rules"""
        try:
            self.validate_query_text(self.query_text)
            return True
        except ValueError:
            return False