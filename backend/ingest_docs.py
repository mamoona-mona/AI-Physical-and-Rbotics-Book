"""
Document ingestion script for Physical AI Textbook.

Reads markdown files from /docs directory, chunks them into
manageable pieces, generates FastEmbed embeddings, and uploads to Qdrant.
"""
import os
import re
from pathlib import Path
from typing import List, Dict, Any
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Add parent directory to path for imports
import sys
sys.path.insert(0, str(Path(__file__).parent))

from app.qdrant.client import QdrantManager
from app.config import get_settings

settings = get_settings()


def clean_markdown(text: str) -> str:
    """Remove markdown syntax artifacts for better embedding."""
    # Remove code blocks
    text = re.sub(r'```[\s\S]*?[\s\S]*?```', '', text)
    # Remove inline code
    text = re.sub(r'`[^`]+`', '', text)
    # Remove links
    text = re.sub(r'\[([^\]]+)\]\([^)]+\)', r'\1', text)
    # Remove extra whitespace
    text = re.sub(r'\n\s*\n', '\n', text)
    # Remove markdown headings markers but keep text
    text = re.sub(r'^#{1,6}\s+', '', text, flags=re.MULTILINE)
    return text.strip()


def chunk_text(text: str, chunk_size: int = 500, overlap: int = 50) -> List[str]:
    """
    Split text into overlapping chunks for better retrieval.

    Args:
        text: The text to chunk
        chunk_size: Maximum characters per chunk
        overlap: Character overlap between chunks

    Returns:
        List of text chunks
    """
    if not text:
        return []

    chunks = []
    start = 0

    while start < len(text):
        end = start + chunk_size
        chunk = text[start:end]

        # Try to break at sentence boundaries
        if end < len(text):
            # Look for sentence end markers
            sentence_ends = ['. ', '! ', '? ', '\n\n', ';\n']
            for sep in sentence_ends:
                last_sep = chunk.rfind(sep)
                if last_sep > len(chunk) * 0.5:  # At least halfway through
                    chunk = chunk[:last_sep + len(sep)]

        chunks.append(chunk.strip())
        start = end - overlap

    return [c for c in chunks if c and len(c) > 50]  # Filter out tiny chunks


def extract_frontmatter(text: str) -> tuple[Dict[str, Any], str]:
    """
    Extract YAML frontmatter from markdown file.

    Returns:
        (frontmatter_dict, content_without_frontmatter)
    """
    frontmatter_pattern = r'^---\s*\n(.*?)\n---\s*\n'
    match = re.match(frontmatter_pattern, text, re.DOTALL)

    if match:
        frontmatter_str = match.group(1)
        content = text[match.end():].strip()

        # Parse simple key: value pairs
        frontmatter = {}
        for line in frontmatter_str.split('\n'):
            if ':' in line:
                key, value = line.split(':', 1)
                frontmatter[key.strip()] = value.strip()
        return frontmatter, content

    return {}, text


def read_markdown_file(file_path: Path) -> Dict[str, Any]:
    """
    Read and parse a markdown file.

    Returns:
        Dictionary with file metadata and content
    """
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            text = f.read()

        # Extract frontmatter and content
        frontmatter, content = extract_frontmatter(text)

        # Clean content for embedding
        clean_content = clean_markdown(content)

        # Get relative path
        abs_path = str(file_path.resolve())
        rel_path = file_path.name

        # Try to get relative path from docs
        try:
            cwd = Path.cwd().resolve()
            rel_path = str(file_path.relative_to(cwd)).replace('\\', '/')
        except:
            rel_path = file_path.name.replace('\\', '/')

        return {
            'file_path': str(file_path),
            'filename': file_path.name,
            'relative_path': rel_path,
            'title': frontmatter.get('title', file_path.stem),
            'sidebar_position': frontmatter.get('sidebar_position', 0),
            'content': clean_content,
            'raw_content': content,
            'frontmatter': frontmatter,
            'chunks': chunk_text(clean_content),
        }
    except Exception as e:
        logger.error(f"Error reading {file_path}: {e}")
        return None


def scan_docs_directory(docs_dir: Path) -> List[Dict[str, Any]]:
    """
    Scan /docs directory for all markdown files.

    Returns:
        List of parsed markdown documents
    """
    docs = []

    # Find all .md files recursively
    for md_file in docs_dir.rglob('*.md'):
        # Skip files in node_modules or other hidden directories
        if 'node_modules' in str(md_file) or md_file.name.startswith('.'):
            continue

        doc = read_markdown_file(md_file)
        if doc:
            docs.append(doc)
            logger.info(f"Found: {doc['relative_path']} ({len(doc['chunks'])} chunks)")

    return docs


def ingest_document_to_qdrant(qdrant: QdrantManager, doc: Dict[str, Any],
                                 mode: str = "global") -> bool:
    """
    Ingest a single document and its chunks to Qdrant.

    Returns:
        True if successful, False otherwise
    """
    try:
        # Use relative path as document ID
        document_id = doc['relative_path'].replace('/', '_').replace('\\', '_')

        # Add metadata
        metadata = {
            'title': doc['title'],
            'source': doc['title'],
            'filename': doc['filename'],
            'path': doc['relative_path'],
            'chapter': extract_chapter_name(doc['relative_path']),
            'type': 'markdown',
            'chunk_count': len(doc['chunks']),
        }

        # Uploads full document (main reference)
        qdrant.upload_document(
            document_id=document_id,
            content=doc['content'][:5000],  # First 5000 chars as reference
            embedding=[],  # Will be generated by QdrantManager
            metadata=metadata,
            mode=mode,
        )

        # Upload each chunk as a separate point with document_id as part of ID
        for i, chunk in enumerate(doc['chunks']):
            chunk_id = f"{document_id}_chunk_{i}"
            chunk_metadata = {
                **metadata,
                'is_chunk': True,
                'chunk_index': i,
                'total_chunks': len(doc['chunks']),
            }

            qdrant.upload_document(
                document_id=chunk_id,
                content=chunk,
                embedding=[],
                metadata=chunk_metadata,
                mode=mode,
            )

        logger.info(f"  Uploaded: {doc['relative_path']} ({len(doc['chunks'])} chunks)")
        return True

    except Exception as e:
        logger.error(f"Failed to ingest {doc['relative_path']}: {e}")
        return False


def extract_chapter_name(path: str) -> str:
    """Extract chapter/module name from file path."""
    # Pattern: docs/01-ros2-fundamentals/intro.md
    if '/' in path or '\\' in path:
        parts = re.split(r'[\\/]', path)
        if len(parts) >= 2:
            # Get the directory name (e.g., "01-ros2-fundamentals")
            return parts[-2]
    return path


def ingest_all_documents(qdrant: QdrantManager, docs: List[Dict[str, Any]],
                          mode: str = "global") -> Dict[str, int]:
    """
    Ingest all documents to Qdrant.

    Returns:
        Dictionary with success/failure counts
    """
    stats = {'success': 0, 'failed': 0, 'total_chunks': 0}

    logger.info(f"Starting ingestion for {len(docs)} documents in {mode} mode...")

    for i, doc in enumerate(docs, 1):
        logger.info(f"[{i}/{len(docs)}] Processing: {doc['relative_path']}")

        if ingest_document_to_qdrant(qdrant, doc, mode):
            stats['success'] += 1
            stats['total_chunks'] += len(doc['chunks'])
        else:
            stats['failed'] += 1

    logger.info(f"\nIngestion complete!")
    logger.info(f"  Successful: {stats['success']}")
    logger.info(f"  Failed: {stats['failed']}")
    logger.info(f"  Total chunks: {stats['total_chunks']}")

    # Get collection info
    global_info = qdrant.get_collection_info(mode="global")
    selection_info = qdrant.get_collection_info(mode="selection")

    logger.info(f"\nCollection Info:")
    logger.info(f"  Global: {global_info.get('vectors_count', 0)} vectors")
    logger.info(f"  Selection: {selection_info.get('vectors_count', 0)} vectors")

    return stats


def main():
    """Main execution function."""
    logger.info("=" * 60)
    logger.info("Physical AI Textbook - Document Ingestion Script")
    logger.info("=" * 60)

    # Find docs directory - use absolute path
    script_path = Path(__file__).resolve().parent
    backend_dir = script_path / 'backend' if (script_path / 'backend').exists() else script_path
    project_root = backend_dir.parent
    docs_dir = project_root / 'docs'

    if not docs_dir.exists():
        logger.error(f"Docs directory not found: {docs_dir}")
        return

    logger.info(f"Scanning docs directory: {docs_dir}")

    # Scan for documents
    documents = scan_docs_directory(docs_dir)

    if not documents:
        logger.warning("No markdown files found in /docs directory")
        return

    logger.info(f"Found {len(documents)} documents to ingest")

    # Initialize Qdrant manager
    try:
        qdrant = QdrantManager()
        logger.info(f"Qdrant initialized: available={qdrant.is_available()}")
    except Exception as e:
        logger.error(f"Failed to initialize Qdrant: {e}")
        return

    # Ingest to global collection (for Global RAG)
    logger.info("\n" + "=" * 60)
    logger.info("Ingesting to GLOBAL collection (for Global RAG queries)")
    logger.info("=" * 60)

    global_stats = ingest_all_documents(qdrant, documents, mode="global")

    # Also ingest to selection collection for testing Selection RAG
    # In production, Selection RAG would ingest per-document on user selection
    logger.info("\n" + "=" * 60)
    logger.info("Ingesting to SELECTION collection (for Selection RAG testing)")
    logger.info("=" * 60)

    selection_stats = ingest_all_documents(qdrant, documents, mode="selection")

    # Final summary
    logger.info("\n" + "=" * 60)
    logger.info("INGESTION SUMMARY")
    logger.info("=" * 60)
    logger.info(f"Global Collection: {global_stats['success']} docs, {global_stats['total_chunks']} chunks")
    logger.info(f"Selection Collection: {selection_stats['success']} docs, {selection_stats['total_chunks']} chunks")
    logger.info("\nYou can now test the chatbot!")


if __name__ == "__main__":
    main()
