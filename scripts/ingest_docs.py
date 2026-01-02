#!/usr/bin/env python3
"""
Script to ingest markdown documents from Docusaurus docs into Qdrant Cloud.
"""
import os
import sys
import glob
from pathlib import Path

# Add backend to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from app.rag.chain import get_rag_chain


def extract_frontmatter(content):
    """Extract title and frontmatter from markdown content."""
    lines = content.split('\n')
    title = None
    metadata = {}

    # Try to find title from first h1
    for line in lines:
        if line.startswith('# '):
            title = line[2:].strip()
            break

    return title, metadata


def read_markdown_file(filepath):
    """Read a markdown file and return its content."""
    with open(filepath, 'r', encoding='utf-8') as f:
        return f.read()


def get_document_slug(filepath, docs_dir):
    """Generate a slug from the file path."""
    rel_path = os.path.relpath(filepath, docs_dir)
    slug = rel_path.replace('.md', '').replace(os.sep, '/')
    return slug


def ingest_directory(docs_dir, mode='global'):
    """Ingest all markdown files from a directory."""
    rag_chain = get_rag_chain()

    # Find all markdown files
    md_files = glob.glob(os.path.join(docs_dir, '**/*.md'), recursive=True)

    # Filter out index files
    md_files = [f for f in md_files if not f.endswith('/index.md')]

    print(f"Found {len(md_files)} markdown files to ingest in mode: {mode}")

    for filepath in md_files:
        slug = get_document_slug(filepath, docs_dir)
        content = read_markdown_file(filepath)
        title, _ = extract_frontmatter(content)

        if not title:
            title = Path(filepath).stem.replace('-', ' ').title()

        print(f"Ingesting: {slug}")

        try:
            rag_chain.ingest_markdown(
                slug=slug,
                title=title,
                content=content,
                chapter='',
                mode=mode,
            )
            print(f"  ✓ Ingested: {title}")
        except Exception as e:
            print(f"  ✗ Error ingesting {slug}: {e}")

    print(f"\nCompleted ingesting {len(md_files)} documents in mode: {mode}")


def main():
    """Main entry point."""
    docs_dir = os.path.join(os.path.dirname(__file__), 'docs')

    if len(sys.argv) > 1:
        docs_dir = sys.argv[1]

    if not os.path.exists(docs_dir):
        print(f"Error: Directory {docs_dir} does not exist")
        sys.exit(1)

    # Ingest to global collection
    print("=" * 50)
    print("Ingesting to GLOBAL collection...")
    print("=" * 50)
    ingest_directory(docs_dir, mode='global')

    # Ingest to selection collection
    print("\n" + "=" * 50)
    print("Ingesting to SELECTION collection...")
    print("=" * 50)
    ingest_directory(docs_dir, mode='selection')


if __name__ == '__main__':
    main()
