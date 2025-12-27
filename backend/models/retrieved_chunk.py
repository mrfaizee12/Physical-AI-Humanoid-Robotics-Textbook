from typing import Dict, Any, Optional
from pydantic import BaseModel

class RetrievedChunk(BaseModel):
    """
    Model representing document segments retrieved from the Qdrant rag_embedding collection
    that are relevant to the query
    """
    id: str
    content: str
    score: float
    metadata: Dict[str, Any] = {}
    source: Optional[str] = None

    def __str__(self):
        """String representation of the chunk"""
        return f"Chunk(id={self.id}, score={self.score}, content='{self.content[:50]}...')"

    def is_relevant(self, min_score: float = 0.3) -> bool:
        """Check if the chunk meets the minimum relevance threshold"""
        return self.score >= min_score