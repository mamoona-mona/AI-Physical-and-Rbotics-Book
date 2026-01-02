"""
Document ingestion script using Qdrant REST API directly.

This script bypasses the Qdrant SDK and uses httpx to directly call the REST API.
"""
import os
import re
import httpx
from pathlib import Path
from typing import List, Dict, Any
import logging
from dotenv import load_dotenv
import json

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Configuration
QDRANT_URL = os.getenv('QDRANT_URL')
QDRANT_API_KEY = os.getenv('QDRANT_API_KEY')
EMBEDDING_DIM = 384

# Collection names
GLOBAL_COLLECTION = "physical_ai_global"
SELECTION_COLLECTION = "physical_ai_selections"


def clean_markdown(text: str) -> str:
    """Remove markdown syntax artifacts."""
    text = re.sub(r'```[\s\S]*?```', '', text)
    text = re.sub(r'`[^`]+`', '', text)
    text = re.sub(r'\[([^\]]+)\]\([^)]+\)', r'\1', text)
    text = re.sub(r'\n\s*\n', '\n', text)
    text = re.sub(r'^#{1,6}\s+', '', text, flags=re.MULTILINE)
    return text.strip()


def chunk_text(text: str, chunk_size: int = 500, overlap: int = 50) -> List[str]:
    """Split text into overlapping chunks."""
    if not text:
        return []

    chunks = []
    start = 0

    while start < len(text):
        end = start + chunk_size
        chunk = text[start:end]

        if end < len(text):
            sentence_ends = ['. ', '! ', '? ', '\n\n', ';\n']
            for sep in sentence_ends:
                last_sep = chunk.rfind(sep)
                if last_sep > len(chunk) * 0.5:
                    chunk = chunk[:last_sep + len(sep)]
                    break

        chunks.append(chunk.strip())
        start = end - overlap

    return [c for c in chunks if c and len(c) > 50]


def extract_frontmatter(text: str) -> tuple:
    """Extract YAML frontmatter."""
    frontmatter_pattern = r'^---\s*\n(.*?)\n---\s*\n'
    match = re.match(frontmatter_pattern, text, re.DOTALL)

    if match:
        frontmatter_str = match.group(1)
        content = text[match.end():].strip()
        frontmatter = {}
        for line in frontmatter_str.split('\n'):
            if ':' in line:
                key, value = line.split(':', 1)
                frontmatter[key.strip()] = value.strip()
        return frontmatter, content

    return {}, text


def get_embedding_model():
    """Initialize FastEmbed model."""
    try:
        from fastembed import TextEmbedding
        model = TextEmbedding(model_name="BAAI/bge-small-en-v1.5")
        logger.info("FastEmbed model initialized successfully")
        return model
    except Exception as e:
        logger.error(f"Failed to initialize FastEmbed: {e}")
        return None


def get_embeddings(model, texts: List[str]) -> List[List[float]]:
    """Generate embeddings using FastEmbed."""
    if model is None:
        logger.warning("Using dummy embeddings (FastEmbed not available)")
        return [[0.1] * EMBEDDING_DIM for _ in texts]

    try:
        embeddings = list(model.embed(texts))
        return [e.tolist() for e in embeddings]
    except Exception as e:
        logger.error(f"Error generating embeddings: {e}")
        return [[0.1] * EMBEDDING_DIM for _ in texts]


def create_collection(client: httpx.Client, collection_name: str):
    """Create a Qdrant collection via REST API."""
    try:
        # Check if exists
        response = client.get(f"/collections/{collection_name}")
        if response.status_code == 200:
            logger.info(f"Collection {collection_name} already exists")
            return True
    except:
        pass

    # Create collection
    try:
        response = client.put(
            f"/collections/{collection_name}",
            json={
                "vectors": {
                    "size": EMBEDDING_DIM,
                    "distance": "Cosine"
                }
            }
        )
        if response.status_code in [200, 201]:
            logger.info(f"Created collection {collection_name}")
            return True
        else:
            logger.error(f"Failed to create {collection_name}: {response.text}")
            return False
    except Exception as e:
        logger.error(f"Error creating collection: {e}")
        return False


def upload_points(client: httpx.Client, collection_name: str, points: List[Dict]):
    """Upload points to Qdrant via REST API."""
    try:
        response = client.put(
            f"/collections/{collection_name}/points",
            json={"points": points},
            timeout=30.0
        )
        if response.status_code in [200, 201]:
            return True
        else:
            logger.error(f"Failed to upload points: {response.text}")
            return False
    except Exception as e:
        logger.error(f"Error uploading points: {e}")
        return False


def ingest_document(client: httpx.Client, doc: Dict, collection_name: str, doc_index: int, embedding_model) -> bool:
    """Ingest a single document."""
    try:
        import uuid
        import hashlib

        # Generate UUID from path for consistent IDs
        path_hash = hashlib.md5(doc['relative_path'].encode()).hexdigest()
        doc_uuid = str(uuid.UUID(path_hash))

        metadata = {
            'title': doc['title'],
            'source': doc['title'],
            'filename': doc['filename'],
            'path': doc['relative_path'],
            'type': 'markdown',
            'chunk_count': len(doc['chunks']),
        }

        # Prepare all texts for batch embedding
        texts_to_embed = [doc['content'][:5000]] + doc['chunks']

        # Generate embeddings in batch for efficiency
        embeddings = get_embeddings(embedding_model, texts_to_embed)

        # Create points for all chunks
        points = []

        # Main document point
        points.append({
            "id": doc_uuid,
            "vector": embeddings[0],
            "payload": {
                "content": doc['content'][:5000],
                **metadata
            }
        })

        # Chunk points
        for i, chunk in enumerate(doc['chunks']):
            chunk_path = f"{doc['relative_path']}_chunk_{i}"
            chunk_hash = hashlib.md5(chunk_path.encode()).hexdigest()
            chunk_uuid = str(uuid.UUID(chunk_hash))

            points.append({
                "id": chunk_uuid,
                "vector": embeddings[i + 1],  # +1 because first embedding is for main doc
                "payload": {
                    "content": chunk,
                    **metadata,
                    "is_chunk": True,
                    "chunk_index": i,
                }
            })

        # Upload in batch
        success = upload_points(client, collection_name, points)

        if success:
            logger.info(f"  Uploaded: {doc['relative_path']} ({len(doc['chunks'])} chunks)")
            return True
        return False

    except Exception as e:
        logger.error(f"Failed to ingest {doc['relative_path']}: {e}")
        return False


def main():
    """Main execution."""
    logger.info("=" * 60)
    logger.info("Physical AI Textbook - REST API Ingestion with FastEmbed")
    logger.info("=" * 60)

    # Initialize embedding model
    logger.info("Initializing FastEmbed model...")
    embedding_model = get_embedding_model()
    if embedding_model is None:
        logger.error("Failed to initialize FastEmbed. Exiting.")
        return

    # Find docs directory
    script_path = Path(__file__).resolve().parent
    backend_dir = script_path / 'backend' if (script_path / 'backend').exists() else script_path
    project_root = backend_dir.parent if backend_dir.name == 'backend' else backend_dir
    docs_dir = project_root / 'docs'

    logger.info(f"Docs directory: {docs_dir}")

    # Scan for markdown files
    documents = []
    for md_file in docs_dir.rglob('*.md'):
        if 'node_modules' in str(md_file):
            continue

        try:
            with open(md_file, 'r', encoding='utf-8') as f:
                text = f.read()

            frontmatter, content = extract_frontmatter(text)
            clean_content = clean_markdown(content)

            try:
                rel_path = str(md_file.relative_to(project_root)).replace('\\', '/')
            except:
                rel_path = md_file.name

            doc = {
                'filename': md_file.name,
                'relative_path': rel_path,
                'title': frontmatter.get('title', md_file.stem),
                'content': clean_content,
                'chunks': chunk_text(clean_content),
            }
            documents.append(doc)
            logger.info(f"Found: {rel_path} ({len(doc['chunks'])} chunks)")
        except Exception as e:
            logger.error(f"Error reading {md_file}: {e}")

    logger.info(f"\nFound {len(documents)} documents to ingest")

    # Create HTTP client
    headers = {"api-key": QDRANT_API_KEY, "Content-Type": "application/json"}
    client = httpx.Client(base_url=QDRANT_URL, headers=headers, timeout=30.0)

    # Create collections
    logger.info("\nCreating collections...")
    create_collection(client, GLOBAL_COLLECTION)
    create_collection(client, SELECTION_COLLECTION)

    # Ingest to global collection
    logger.info("\n" + "=" * 60)
    logger.info("Ingesting to GLOBAL collection with real embeddings")
    logger.info("=" * 60)

    success_count = 0
    total_chunks = 0

    for i, doc in enumerate(documents, 1):
        logger.info(f"[{i}/{len(documents)}] Processing: {doc['relative_path']}")
        if ingest_document(client, doc, GLOBAL_COLLECTION, i, embedding_model):
            success_count += 1
            total_chunks += len(doc['chunks'])

    logger.info(f"\n" + "=" * 60)
    logger.info(f"INGESTION COMPLETE")
    logger.info(f"=" * 60)
    logger.info(f"Successful: {success_count}/{len(documents)}")
    logger.info(f"Total chunks: {total_chunks}")

    client.close()


if __name__ == "__main__":
    main()
