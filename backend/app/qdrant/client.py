"""Qdrant Cloud vector database client with FastEmbed for embeddings."""
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct, Filter, FieldCondition, MatchValue
from app.config import get_settings
from typing import Optional, List, Dict, Any
import uuid
import logging

logger = logging.getLogger(__name__)

settings = get_settings()

# Collection names for different RAG modes
TEXTBOOK_COLLECTION = settings.qdrant_collection
GLOBAL_COLLECTION = "physical_ai_global"
SELECTION_COLLECTION = "physical_ai_selections"

# FastEmbed embedding dimension for BAAI/bge-small-en-v1.5
EMBEDDING_DIMENSION = 384


class QdrantManager:
    """Manager for Qdrant Cloud vector database operations with FastEmbed."""

    def __init__(self):
        """Initialize Qdrant client and FastEmbed model."""
        self._available = False
        self._demo_mode = True
        self._demo_embeddings = {}
        self._demo_documents = {}
        self.embedding_model = None  # Lazy initialization
        self._fastembed_attempted = False

        # Try to initialize Qdrant with REST API (bypass SDK issues)
        try:
            import httpx
            # Test connection with REST API
            headers = {"api-key": settings.qdrant_api_key}
            test_response = httpx.get(
                f"{settings.qdrant_url}/collections",
                headers=headers,
                timeout=10.0
            )

            if test_response.status_code == 200:
                # SDK initialization - use prefer_grpc=False for REST only
                self.client = QdrantClient(
                    url=settings.qdrant_url,
                    api_key=settings.qdrant_api_key,
                    timeout=10.0,
                    prefer_grpc=False,
                    https=True if settings.qdrant_url.startswith('https') else False,
                )
                self._available = True
                self._demo_mode = False
                logger.info("Qdrant manager initialized successfully via REST API")
            else:
                raise Exception(f"Qdrant REST API returned {test_response.status_code}")

        except Exception as e:
            logger.warning(f"Qdrant not available, running in demo mode: {e}")

    def _ensure_collections(self):
        """Ensure all required collections exist."""
        for collection_name in [GLOBAL_COLLECTION, SELECTION_COLLECTION]:
            try:
                self.client.get_collection(collection_name)
                logger.info(f"Collection {collection_name} already exists")
            except Exception as e:
                try:
                    logger.info(f"Creating collection {collection_name}...")
                    self.client.create_collection(
                        collection_name=collection_name,
                        vectors_config=VectorParams(
                            size=EMBEDDING_DIMENSION,
                            distance=Distance.COSINE,
                        ),
                    )
                    logger.info(f"Collection {collection_name} created successfully")
                except Exception as create_error:
                    logger.error(f"Failed to create collection {collection_name}: {create_error}")

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
        if getattr(self, '_demo_mode', False) or not hasattr(self, 'embedding_model'):
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
            # Use timeout to avoid blocking
            embeddings = list(self.embedding_model.embed([text]))
            return embeddings[0].tolist()
        except Exception as e:
            logger.warning(f"Failed to generate embedding: {e}")
            return [0.1] * EMBEDDING_DIMENSION

    def get_embeddings(self, texts: List[str]) -> List[List[float]]:
        """Generate embeddings for multiple texts using FastEmbed."""
        if getattr(self, '_demo_mode', False) or not hasattr(self, 'embedding_model'):
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
        """Upload a document to the appropriate collection."""
        collection = self._get_collection_name(mode)
        self.client.upsert(
            collection_name=collection,
            points=[
                PointStruct(
                    id=document_id,
                    vector=embedding,
                    payload={
                        "content": content,
                        "metadata": metadata,
                    },
                )
            ],
        )

    def search(
        self,
        query_embedding: List[float],
        limit: int = 5,
        score_threshold: float = 0.5,
        mode: str = "global",
        doc_filter: Optional[Dict[str, str]] = None,
    ) -> List[Dict[str, Any]]:
        """Search for similar documents in the specified mode."""
        if getattr(self, '_demo_mode', False):
            # Return empty results in demo mode
            return []

        collection = self._get_collection_name(mode)

        # Build filter if provided
        filter_conditions = None
        if doc_filter:
            filter_conditions = Filter(
                must=[
                    FieldCondition(key=k, match=MatchValue(value=v))
                    for k, v in doc_filter.items()
                ]
            )

        results = self.client.search(
            collection_name=collection,
            query_vector=query_embedding,
            limit=limit,
            score_threshold=score_threshold,
            query_filter=filter_conditions,
        )

        return [
            {
                "id": hit.id,
                "score": hit.score,
                "content": hit.payload.get("content", ""),
                "metadata": hit.payload.get("metadata", {}),
            }
            for hit in results
        ]

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
            doc_filter={"document_slug": document_slug},
        )

    def delete_document(self, document_id: str, mode: str = "global"):
        """Delete a document from the specified collection."""
        collection = self._get_collection_name(mode)
        self.client.delete(
            collection_name=collection,
            points_selector=[document_id],
        )

    def get_collection_info(self, mode: str = "global") -> Dict[str, Any]:
        """Get information about a collection."""
        collection = self._get_collection_name(mode)
        try:
            collection_info = self.client.get_collection(collection)
            points_count = self.client.count(
                collection_name=collection,
                exact=True,
            ).count
            return {
                "name": collection,
                "status": collection_info.status,
                "vectors_count": points_count,
                "vector_size": collection_info.config.params.vectors.size,
                "distance": collection_info.config.params.vectors.distance,
            }
        except Exception as e:
            return {"error": str(e)}

    def batch_ingest(
        self,
        documents: List[Dict[str, Any]],
        embeddings: List[List[float]],
        mode: str = "global",
    ):
        """Batch ingest multiple documents with embeddings."""
        collection = self._get_collection_name(mode)

        points = []
        for doc, embedding in zip(documents, embeddings):
            points.append(
                PointStruct(
                    id=doc.get("id", str(uuid.uuid4())),
                    vector=embedding,
                    payload={
                        "content": doc["content"],
                        "metadata": doc.get("metadata", {}),
                    },
                )
            )

        self.client.upsert(collection_name=collection, points=points)


# Global instance
_qdrant_manager: Optional[QdrantManager] = None


def get_qdrant_manager() -> QdrantManager:
    """Get or create the Qdrant manager instance."""
    global _qdrant_manager
    if _qdrant_manager is None:
        _qdrant_manager = QdrantManager()
    return _qdrant_manager
