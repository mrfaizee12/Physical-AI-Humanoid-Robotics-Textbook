import os
from typing import Optional
from pydantic import BaseModel
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

class QdrantConnection(BaseModel):
    """Configuration for connecting to the Qdrant vector database"""
    url: str = os.getenv("QDRANT_URL", "")
    api_key: str = os.getenv("QDRANT_API_KEY", "")
    collection_name: str = os.getenv("QDRANT_COLLECTION_NAME", "rag_embedding")
    vector_size: int = 1536  # Default size for OpenAI embeddings

class Settings(BaseModel):
    """Application settings"""
    gemini_api_key: str = os.getenv("GEMINI_API_KEY", "")
    qdrant: QdrantConnection = QdrantConnection()

    class Config:
        env_file = ".env"

# Global settings instance
settings = Settings()