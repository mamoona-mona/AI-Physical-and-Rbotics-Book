"""Configuration management for the backend."""
import os
from functools import lru_cache
from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # Groq Configuration (LLM)
    groq_api_key: str = "gsk-placeholder"
    groq_model: str = "llama-3.3-70b-versatile"

    # FastEmbed Configuration (Embeddings)
    fastembed_model: str = "BAAI/bge-small-en-v1.5"

    # Neon PostgreSQL Configuration
    database_url: str = "postgresql://user:password@localhost/physical_ai"

    # Qdrant Cloud Configuration
    qdrant_url: str = "https://localhost:6333"
    qdrant_api_key: str = "placeholder-key"
    qdrant_collection: str = "physical_ai_textbook"

    # Better-Auth Configuration
    better_auth_secret: str = "secret-key-min-32-chars-placeholder"
    better_auth_url: str = "http://localhost:3000"

    # Google OAuth Configuration
    google_client_id: str = "placeholder-client-id"
    google_client_secret: str = "placeholder-secret"
    google_redirect_uri: str = "http://localhost:8000/auth/google/callback"

    # Frontend URL
    frontend_url: str = "http://localhost:3000"

    # ChatKit Configuration
    chatkit_url: str = "http://localhost:3001"

    class Config:
        # Look for .env in backend/ directory
        env_file = ".env"
        env_file_encoding = "utf-8"
        case_sensitive = False  # Case insensitive for Windows
        extra = "ignore"  # Ignore extra environment variables


@lru_cache()
def get_settings() -> Settings:
    """Get cached settings instance."""
    # Find .env file in backend directory
    backend_dir = os.path.dirname(os.path.dirname(__file__))
    env_path = os.path.join(backend_dir, ".env")

    if os.path.exists(env_path):
        return Settings(_env_file=env_path)

    return Settings()
