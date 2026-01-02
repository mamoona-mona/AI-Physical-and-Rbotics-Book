"""Qdrant REST API client with FastEmbed for embeddings."""
import httpx
from app.config import get_settings
from typing import Optional, List, Dict, Any
import hashlib
import uuid
import logging

logger = logging.getLogger(__name__)

settings = get_settings()

# Collection names for different RAG modes
GLOBAL_COLLECTION = "physical_ai_global"
SELECTION_COLLECTION = "physical_ai_selections"

# FastEmbed embedding dimension for BAAI/bge-small-en-v1.5
EMBEDDING_DIMENSION = 384


class QdrantManager:
    """Manager for Qdrant Cloud using REST API directly."""

    def __init__(self):
        """Initialize Qdrant REST client."""
        self._available = False
        self._demo_mode = True
        self.embedding_model = None
        self._fastembed_attempted = False
        self._connection_tested = False

        # Don't connect in __init__ - causes issues with process forking
        logger.info("QdrantManager initialized (will connect on first use)")

    def _ensure_connection(self):
        """Ensure connection is established (lazy init)."""
        if self._connection_tested:
            return

        self._connection_tested = True

        try:
            headers = {"api-key": settings.qdrant_api_key}
            response = httpx.get(
                f"{settings.qdrant_url}/collections",
                headers=headers,
                timeout=10.0
            )

            if response.status_code == 200:
                self._available = True
                self._demo_mode = False
                logger.info("Qdrant REST API connection successful")
            else:
                logger.warning(f"Qdrant returned status {response.status_code}")

        except Exception as e:
            logger.warning(f"Qdrant not available, using demo mode: {e}")

    def _get_client(self) -> httpx.Client:
        """Get httpx client (creates new one each time to avoid process issues)."""
        headers = {"api-key": settings.qdrant_api_key}
        return httpx.Client(
            base_url=settings.qdrant_url,
            headers=headers,
            timeout=30.0
        )

    def _get_collection_name(self, mode: str) -> str:
        """Get collection name based on RAG mode."""
        if mode == "selection":
            return SELECTION_COLLECTION
        return GLOBAL_COLLECTION

    def is_available(self) -> bool:
        """Check if Qdrant is available."""
        return self._available

    def get_embedding(self, text: str) -> List[float]:
        """Generate embedding for text using FastEmbed."""
        # Demo mode returns fixed vector
        if self._demo_mode:
            return [0.1] * EMBEDDING_DIMENSION

        # Try FastEmbed with lazy initialization
        if self.embedding_model is None and not self._fastembed_attempted:
            try:
                from fastembed import TextEmbedding
                self.embedding_model = TextEmbedding(model_name=settings.fastembed_model)
                logger.info("FastEmbed initialized on first use")
            except Exception as e:
                logger.warning(f"FastEmbed initialization failed: {e}")
                self._fastembed_attempted = True
                return [0.1] * EMBEDDING_DIMENSION

        if self.embedding_model is None:
            return [0.1] * EMBEDDING_DIMENSION

        try:
            embeddings = list(self.embedding_model.embed([text]))
            return embeddings[0].tolist()
        except Exception as e:
            logger.warning(f"Failed to generate embedding: {e}")
            return [0.1] * EMBEDDING_DIMENSION

    def get_embeddings(self, texts: List[str]) -> List[List[float]]:
        """Generate embeddings for multiple texts using FastEmbed."""
        if self._demo_mode:
            return [[0.1] * EMBEDDING_DIMENSION for _ in texts]

        if self.embedding_model is None and not self._fastembed_attempted:
            try:
                from fastembed import TextEmbedding
                self.embedding_model = TextEmbedding(model_name=settings.fastembed_model)
                logger.info("FastEmbed initialized on first use")
            except Exception as e:
                logger.warning(f"FastEmbed initialization failed: {e}")
                self._fastembed_attempted = True
                return [[0.1] * EMBEDDING_DIMENSION for _ in texts]

        if self.embedding_model is None:
            return [[0.1] * EMBEDDING_DIMENSION for _ in texts]

        try:
            embeddings = list(self.embedding_model.embed(texts))
            return [e.tolist() for e in embeddings]
        except Exception as e:
            logger.warning(f"Failed to generate embeddings: {e}")
            return [[0.1] * EMBEDDING_DIMENSION for _ in texts]

    def upload_document(
        self,
        document_id: str,
        content: str,
        embedding: List[float],
        metadata: Dict[str, Any],
        mode: str = "global",
    ):
        """Upload a document to appropriate collection via REST API."""
        self._ensure_connection()

        if self._demo_mode:
            # Store in memory for demo
            collection_name = self._get_collection_name(mode)
            if not hasattr(self, '_demo_documents'):
                self._demo_documents = {}
            if collection_name not in self._demo_documents:
                self._demo_documents[collection_name] = {}
            self._demo_documents[collection_name][document_id] = {
                "content": content,
                "embedding": embedding,
                "metadata": metadata,
            }
            return

        # Use REST API to upload
        collection = self._get_collection_name(mode)

        # Generate UUID from document_id
        doc_hash = hashlib.md5(document_id.encode()).hexdigest()
        doc_uuid = str(uuid.UUID(doc_hash))

        # Generate embedding if not provided
        if not embedding or len(embedding) == 0:
            embedding = self.get_embedding(content)

        point = {
            "id": doc_uuid,
            "vector": embedding,
            "payload": {
                "content": content,
                **metadata
            }
        }

        try:
            client = self._get_client()
            response = client.put(
                f"/collections/{collection}/points",
                json={"points": [point]}
            )
            client.close()

            if response.status_code not in [200, 201]:
                logger.error(f"Failed to upload point: {response.text}")
        except Exception as e:
            logger.error(f"Upload error: {e}")

    def search(
        self,
        query_embedding: List[float],
        limit: int = 5,
        score_threshold: float = 0.5,
        mode: str = "global",
        doc_filter: Optional[Dict[str, str]] = None,
    ) -> List[Dict[str, Any]]:
        """Search for similar documents via REST API."""
        self._ensure_connection()

        if self._demo_mode:
            return []

        collection = self._get_collection_name(mode)

        # Build search request
        search_payload = {
            "vector": query_embedding,
            "limit": limit,
            "score_threshold": score_threshold,
            "with_payload": True,
        }

        # Add filter if provided
        if doc_filter:
            search_payload["filter"] = {
                "must": [
                    {"key": k, "match": {"value": v}}
                    for k, v in doc_filter.items()
                ]
            }

        try:
            client = self._get_client()
            response = client.post(
                f"/collections/{collection}/points/search",
                json=search_payload
            )
            client.close()

            if response.status_code == 200:
                results = response.json().get("result", [])
                return [
                    {
                        "id": hit["id"],
                        "score": hit["score"],
                        "content": hit["payload"].get("content", ""),
                        "metadata": {k: v for k, v in hit["payload"].items() if k != "content"},
                    }
                    for hit in results
                ]
        except Exception as e:
            logger.error(f"Search error: {e}")

        return []

    def search_by_document(
        self,
        query_embedding: List[float],
        document_slug: str,
        limit: int = 5,
    ) -> List[Dict[str, Any]]:
        """Search within a specific document for Selection RAG."""
        return self.search(
            query_embedding=query_embedding,
            limit=limit,
            mode="selection",
            doc_filter={"path": document_slug},
        )

    def delete_document(self, document_id: str, mode: str = "global"):
        """Delete a document from the specified collection."""
        self._ensure_connection()

        if self._demo_mode:
            return

        collection = self._get_collection_name(mode)
        doc_hash = hashlib.md5(document_id.encode()).hexdigest()
        doc_uuid = str(uuid.UUID(doc_hash))

        try:
            client = self._get_client()
            client.post(
                f"/collections/{collection}/points/delete",
                json={"points": [doc_uuid]}
            )
            client.close()
        except Exception as e:
            logger.error(f"Delete error: {e}")

    def get_collection_info(self, mode: str = "global") -> Dict[str, Any]:
        """Get information about a collection via REST API."""
        self._ensure_connection()

        if self._demo_mode:
            return {"error": "Demo mode"}

        collection = self._get_collection_name(mode)
        try:
            client = self._get_client()
            response = client.get(f"/collections/{collection}")
            client.close()

            if response.status_code == 200:
                info = response.json()["result"]
                return {
                    "name": collection,
                    "status": info.get("status"),
                    "vectors_count": info.get("points_count", 0),
                    "vector_size": EMBEDDING_DIMENSION,
                    "distance": "Cosine",
                }
        except Exception as e:
            return {"error": str(e)}

        return {"error": "Unknown"}


# Global instance
_qdrant_manager: Optional[QdrantManager] = None


def get_qdrant_manager() -> QdrantManager:
    """Get or create the Qdrant manager instance."""
    global _qdrant_manager
    if _qdrant_manager is None:
        _qdrant_manager = QdrantManager()
    return _qdrant_manager
