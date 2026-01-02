"""API routes for the RAG chatbot."""
from fastapi import APIRouter, HTTPException, Query, Path
from pydantic import BaseModel
from typing import Optional, List, Dict, Any, AsyncGenerator
from app.rag.chain import get_rag_chain

router = APIRouter()


class ChatMessage(BaseModel):
    """Chat message model."""

    role: str
    content: str


class ChatRequest(BaseModel):
    """Chat request model with RAG mode support."""

    message: str
    history: Optional[List[ChatMessage]] = []
    stream: bool = False
    mode: str = "global"  # "global" or "selection"
    document_slug: Optional[str] = None
    selected_text: Optional[str] = None


class ChatResponse(BaseModel):
    """Chat response model."""

    answer: str
    sources: List[Dict[str, Any]]
    mode: str
    stream: bool = False


class SelectionChatRequest(BaseModel):
    """Selection RAG request model for highlighted text."""

    message: str
    selected_text: str
    document_slug: str
    history: Optional[List[ChatMessage]] = []
    stream: bool = False


class IngestRequest(BaseModel):
    """Document ingestion request model."""

    document_id: str
    content: str
    metadata: Dict[str, Any]
    mode: str = "global"


class HealthResponse(BaseModel):
    """Health check response model."""

    status: str
    version: str = "1.0.0"


class CollectionInfoResponse(BaseModel):
    """Collection info response model."""

    global_collection: Dict[str, Any]
    selection_collection: Dict[str, Any]


@router.get("/health", response_model=HealthResponse)
async def health_check():
    """Health check endpoint."""
    return HealthResponse(status="healthy")


@router.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """Chat endpoint for the RAG chatbot with mode support."""
    try:
        rag_chain = get_rag_chain()

        # Convert history to the expected format
        history = [
            {"role": msg.role, "content": msg.content}
            for msg in (request.history or [])
        ]

        # Route to appropriate RAG method based on mode
        if request.mode == "selection" and request.selected_text:
            result = rag_chain.answer_with_selection(
                question=request.message,
                selected_text=request.selected_text,
                document_slug=request.document_slug or "",
                conversation_history=history,
            )
        else:
            result = rag_chain.answer(
                question=request.message,
                mode=request.mode,
                document_slug=request.document_slug,
                conversation_history=history,
            )

        if request.stream:
            raise HTTPException(
                status_code=400,
                detail="Use /chat/stream for streaming responses",
            )

        return ChatResponse(
            answer=result["answer"],
            sources=result["sources"],
            mode=result["mode"],
            stream=False,
        )

    except HTTPException:
        raise
    except Exception as e:
        import logging
        import traceback
        logger = logging.getLogger(__name__)
        logger.error(f"Chat endpoint error: {e}")
        logger.error(traceback.format_exc())
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/chat/selection")
async def chat_selection(request: SelectionChatRequest):
    """Selection RAG endpoint for answering questions about highlighted text."""
    try:
        rag_chain = get_rag_chain()

        history = [
            {"role": msg.role, "content": msg.content}
            for msg in (request.history or [])
        ]

        result = rag_chain.answer_with_selection(
            question=request.message,
            selected_text=request.selected_text,
            document_slug=request.document_slug,
            conversation_history=history,
        )

        return {
            "answer": result["answer"],
            "sources": result["sources"],
            "mode": "selection",
            "document_slug": request.document_slug,
        }

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/chat/stream")
async def chat_stream(request: ChatRequest):
    """Streaming chat endpoint with mode support."""
    try:
        rag_chain = get_rag_chain()

        history = [
            {"role": msg.role, "content": msg.content}
            for msg in (request.history or [])
        ]

        if request.mode == "selection" and request.selected_text:
            # For selection mode with streaming, use main answer method
            result = rag_chain.answer_with_selection(
                question=request.message,
                selected_text=request.selected_text,
                document_slug=request.document_slug or "",
                conversation_history=history,
            )
            # Create a fake stream from the result
            async def fake_stream():
                yield result["answer"]

            return {"answer_stream": fake_stream()}
        else:
            result = rag_chain.answer_stream(
                question=request.message,
                mode=request.mode,
                document_slug=request.document_slug,
                conversation_history=history,
            )
            return {"answer_stream": result}

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/ingest")
async def ingest_document(request: IngestRequest):
    """Ingest a document into the vector store."""
    try:
        rag_chain = get_rag_chain()
        rag_chain.ingest_document(
            document_id=request.document_id,
            content=request.content,
            metadata=request.metadata,
            mode=request.mode,
        )
        return {"status": "success", "document_id": request.document_id, "mode": request.mode}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/ingest/markdown")
async def ingest_markdown(
    slug: str,
    title: str,
    chapter: str = "",
    mode: str = Query("global", regex="^(global|selection)$"),
):
    """Ingest markdown document from Docusaurus docs."""
    try:
        # This would be called by a Docusaurus plugin during build
        # For now, return instructions
        return {
            "status": "pending",
            "message": "Use the Docusaurus plugin to ingest markdown files during build",
            "instructions": {
                "endpoint": "POST /api/v1/ingest",
                "body": {
                    "document_id": f"{slug}_{mode}",
                    "content": "Markdown content here",
                    "metadata": {
                        "source": title,
                        "slug": slug,
                        "chapter": chapter,
                        "type": "markdown"
                    },
                    "mode": mode
                }
            }
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/collections", response_model=CollectionInfoResponse)
async def get_collection_info():
    """Get information about both vector store collections."""
    try:
        qdrant = get_rag_chain().qdrant
        global_info = qdrant.get_collection_info(mode="global")
        selection_info = qdrant.get_collection_info(mode="selection")

        return CollectionInfoResponse(
            global_collection=global_info,
            selection_collection=selection_info,
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.delete("/collections/{mode}/documents/{document_id}")
async def delete_document(document_id: str, mode: str = Path(..., regex="^(global|selection)$")):
    """Delete a document from the vector store."""
    try:
        qdrant = get_rag_chain().qdrant
        qdrant.delete_document(document_id, mode=mode)
        return {"status": "success", "document_id": document_id, "mode": mode}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
