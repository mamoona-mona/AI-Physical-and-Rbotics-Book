"""Qdrant package."""
from app.qdrant.client import QdrantManager, get_qdrant_manager, TEXTBOOK_COLLECTION
__all__ = ["QdrantManager", "get_qdrant_manager", "TEXTBOOK_COLLECTION"]
