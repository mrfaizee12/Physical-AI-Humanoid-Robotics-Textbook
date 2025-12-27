import os
from typing import List, Optional
from pydantic_settings import SettingsConfigDict, BaseSettings


class Settings(BaseSettings):
    """
    Application settings loaded from environment variables.
    """
    # Qdrant Configuration
    qdrant_url: str = os.getenv("QDRANT_URL", "")
    qdrant_api_key: str = os.getenv("QDRANT_API_KEY", "")
    qdrant_collection_name: str = os.getenv("QDRANT_COLLECTION_NAME", "rag_embedding")

    # Cohere Configuration
    cohere_api_key: str = os.getenv("COHERE_API_KEY", "")

    # OpenRouter Configuration
    openrouter_api_key: str = os.getenv("OPENROUTER_API_KEY", "")

    # Optional Configuration
    rag_agent_url: Optional[str] = os.getenv("RAG_AGENT_URL")
    target_website_url: Optional[str] = os.getenv("TARGET_WEBSITE_URL")
    sitemap_url: Optional[str] = os.getenv("SITEMAP_URL")

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        extra="ignore"
    )

    def validate_required_settings(self):
        """Validate that all required settings are present."""
        errors = []

        if not self.qdrant_url:
            errors.append("QDRANT_URL is required")
        if not self.qdrant_api_key:
            errors.append("QDRANT_API_KEY is required")
        if not self.cohere_api_key:
            errors.append("COHERE_API_KEY is required")
        if not self.openrouter_api_key:
            errors.append("OPENROUTER_API_KEY is required")

        if errors:
            raise ValueError(f"Missing required environment variables: {', '.join(errors)}")

    def check_missing_credentials(self) -> List[str]:
        """Check for missing credentials and return list of missing ones."""
        missing = []
        if not self.qdrant_url:
            missing.append("QDRANT_URL")
        if not self.qdrant_api_key:
            missing.append("QDRANT_API_KEY")
        if not self.cohere_api_key:
            missing.append("COHERE_API_KEY")
        if not self.openrouter_api_key:
            missing.append("OPENROUTER_API_KEY")
        return missing


# Create a singleton instance of settings
settings = Settings()

# Validate settings on import
settings.validate_required_settings()