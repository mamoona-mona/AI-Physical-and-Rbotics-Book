#!/usr/bin/env python3
"""
Simple backend test script - verifies imports and configuration.
"""
# -*- coding: utf-8 -*-
import sys
import os

# Force UTF-8 encoding output
os.environ['PYTHONIOENCODING'] = 'utf-8'

# Add backend to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Use ASCII-safe characters
PASS = "[PASS]"
FAIL = "[FAIL]"
WARN = "[WARN]"
OK = "[OK]"


def test_imports():
    """Test that all imports work."""
    print("Testing imports...")

    try:
        from app.config import get_settings, Settings
        print(f"  {OK} Config loaded")
        print(f"    - Model: {Settings.model_fields['openai_model'].default}")
        print(f"    - Embedding: {Settings.model_fields['openai_embedding_model'].default}")
    except Exception as e:
        print(f"  {FAIL} Config error: {e}")
        return False

    try:
        from app.rag.chain import RAGChain, get_rag_chain
        print(f"  {OK} RAG chain imported")
    except Exception as e:
        print(f"  {FAIL} RAG error: {e}")
        return False

    try:
        from app.qdrant.client import QdrantManager, get_qdrant_manager
        print(f"  {OK} Qdrant client imported")
    except Exception as e:
        print(f"  {FAIL} Qdrant error: {e}")
        return False

    try:
        from app.api.auth import router as auth_router
        print(f"  {OK} Auth router imported")
    except Exception as e:
        print(f"  {FAIL} Auth error: {e}")
        return False

    try:
        from app.api.routes import router as api_router
        print(f"  {OK} API routes imported")
    except Exception as e:
        print(f"  {FAIL} Routes error: {e}")
        return False

    return True


def test_settings():
    """Test settings configuration."""
    print("\nTesting settings...")

    try:
        settings = get_settings()

        # Check required fields
        required = ['openai_api_key', 'database_url', 'qdrant_url', 'qdrant_api_key']
        for field in required:
            value = getattr(settings, field, None)
            if value and 'your-' not in value.lower():
                print(f"  {OK} {field}: set")
            else:
                print(f"  {WARN} {field}: not configured (update .env)")

        return True
    except Exception as e:
        print(f"  {FAIL} Settings error: {e}")
        return False


def test_fastapi_app():
    """Test FastAPI app creation."""
    print("\nTesting FastAPI app...")

    try:
        from fastapi import FastAPI
        from app.config import get_settings

        settings = get_settings()
        app = FastAPI(
            title="Physical AI & Humanoid Robotics API",
            version="1.0.0"
        )

        # Check routes are registered
        from app.api.routes import router as api_router
        from app.api.auth import router as auth_router

        app.include_router(api_router, prefix="/api/v1")
        app.include_router(auth_router, prefix="/api/v1")

        print(f"  {OK} FastAPI app created")
        print(f"  {OK} Routes registered: /api/v1/chat, /api/v1/auth/*")

        # List available endpoints
        routes = [r.path for r in app.routes if hasattr(r, 'path')]
        print(f"  {OK} Available endpoints: {len(routes)}")

        return True
    except Exception as e:
        print(f"  {FAIL} FastAPI error: {e}")
        return False


def main():
    print("=" * 50)
    print("Physical AI Backend - Test Suite")
    print("=" * 50)

    results = []

    results.append(("Imports", test_imports()))
    results.append(("Settings", test_settings()))
    results.append(("FastAPI App", test_fastapi_app()))

    print("\n" + "=" * 50)
    print("Results")
    print("=" * 50)

    all_passed = True
    for name, passed in results:
        status = f"  {PASS}" if passed else f"  {FAIL}"
        print(f"{status}: {name}")
        if not passed:
            all_passed = False

    print()
    if all_passed:
        print("All tests passed!")
        print("\nTo start the server:")
        print("  cd backend")
        print("  uvicorn app.main:app --reload")
        print("\nAPI docs at: http://localhost:8000/docs")
    else:
        print("Some tests failed. Check the errors above.")
        print("\nCommon issues:")
        print("  - Missing API keys in .env")
        print("  - Dependencies not installed: pip install -r requirements.txt")

    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())
