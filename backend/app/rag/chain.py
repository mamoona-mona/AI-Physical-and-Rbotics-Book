"""RAG (Retrieval-Augmented Generation) chain using Groq LLM and FastEmbed."""
from app.config import get_settings
try:
    from app.qdrant.client_rest import get_qdrant_manager
except ImportError:
    from app.qdrant.client import get_qdrant_manager
from typing import List, Dict, Any, Optional, AsyncGenerator
import logging

logger = logging.getLogger(__name__)

settings = get_settings()


class RAGChain:
    """RAG chain with Groq LLM for answering questions about the textbook."""

    def __init__(self):
        """Initialize the RAG chain with Groq client."""
        self._llm_available = False
        try:
            from groq import Groq
            self.client = Groq(api_key=settings.groq_api_key)
            self.model = settings.groq_model
            self._llm_available = True
            logger.info("Groq LLM initialized successfully")
        except Exception as e:
            logger.warning(f"Groq not available, running in demo mode: {e}")
            self._llm_available = False
            self._demo_mode = True

        self.qdrant = get_qdrant_manager()

    def _get_embedding(self, text: str) -> List[float]:
        """Get embedding for a text using FastEmbed."""
        return self.qdrant.get_embedding(text)

    def _build_system_prompt(self, context: str, history: str = "") -> str:
        """Build the system prompt with context and history."""
        return f"""You are a helpful teaching assistant for a Physical AI & Humanoid Robotics textbook.
Your role is to help students understand complex concepts in embodied intelligence, robotics, and physical AI.

Guidelines:
1. Use the provided context from the textbook to answer questions
2. If the context doesn't contain enough information, say so and suggest looking at related topics
3. Provide clear, educational explanations with examples when helpful
4. Use markdown formatting for better readability
5. Keep answers concise but thorough
6. If citing specific documents, reference them by number

Context from the textbook:
{context}

Conversation history (for continuity):
{history}

Answer the user's question based on the context above."""

    def _retrieve(
        self,
        question: str,
        mode: str = "global",
        document_slug: Optional[str] = None,
        limit: int = 5,
    ) -> List[Dict[str, Any]]:
        """Retrieve relevant documents based on the question."""
        question_embedding = self._get_embedding(question)

        if mode == "selection" and document_slug:
            results = self.qdrant.search_by_document(
                query_embedding=question_embedding,
                document_slug=document_slug,
                limit=limit,
            )
        else:
            results = self.qdrant.search(
                query_embedding=question_embedding,
                limit=limit,
                mode=mode,
            )

        return results

    def _format_context(self, documents: List[Dict[str, Any]]) -> str:
        """Format retrieved documents into context string."""
        if not documents:
            return "No relevant context found in the textbook."

        context_parts = []
        for i, doc in enumerate(documents, 1):
            metadata = doc.get("metadata", {})
            source = metadata.get("source", f"Document {doc['id']}")
            chapter = metadata.get("chapter", "")
            content = doc["content"][:1000] if len(doc["content"]) > 1000 else doc["content"]
            context_parts.append(f"[{i}] {source} {chapter}:\n{content}")

        return "\n\n".join(context_parts)

    def answer(
        self,
        question: str,
        mode: str = "global",
        document_slug: Optional[str] = None,
        conversation_history: Optional[List[Dict[str, str]]] = None,
    ) -> Dict[str, Any]:
        """Answer a question using RAG."""
        # Retrieve relevant documents
        documents = self._retrieve(
            question=question,
            mode=mode,
            document_slug=document_slug,
            limit=5 if mode == "selection" else 10,
        )

        # Build context
        context = self._format_context(documents)

        # Build history
        history = ""
        if conversation_history:
            history_lines = []
            for msg in conversation_history[-5:]:
                role = "User" if msg["role"] == "user" else "Assistant"
                history_lines.append(f"{role}: {msg['content']}")
            history = "\n".join(history_lines)

        # Generate response
        if getattr(self, '_llm_available', False) and hasattr(self, 'client'):
            try:
                system_prompt = self._build_system_prompt(context, history)
                response = self.client.chat.completions.create(
                    model=self.model,
                    messages=[
                        {"role": "system", "content": system_prompt},
                        {"role": "user", "content": question},
                    ],
                    temperature=0.7,
                    max_tokens=1500,
                )
                answer = response.choices[0].message.content
            except Exception as e:
                logger.warning(f"LLM generation failed: {e}")
                answer = self._get_demo_answer(question, context)
        else:
            answer = self._get_demo_answer(question, context)

        return {
            "answer": answer,
            "sources": [
                {
                    "id": doc["id"],
                    "score": doc["score"],
                    "source": doc.get("metadata", {}).get("source", ""),
                    "chapter": doc.get("metadata", {}).get("chapter", ""),
                }
                for doc in documents
            ],
            "mode": mode,
        }

    def _get_demo_answer(self, question: str, context: str) -> str:
        """Get a demo response when LLM is not available."""
        return f"""I'm running in demo mode because the LLM is not configured.

Your question: {question}

Context found: {context[:200] if len(context) > 200 else context}

To use the full chatbot:
1. Set up a Groq API key in the .env file
2. Configure Qdrant (optional - can use demo mode)
3. Restart the backend"""

    def answer_stream(
        self,
        question: str,
        mode: str = "global",
        document_slug: Optional[str] = None,
        conversation_history: Optional[List[Dict[str, str]]] = None,
    ) -> AsyncGenerator[str, None]:
        """Stream a response using the retrieved context."""
        # Get documents and context
        documents = self._retrieve(
            question=question,
            mode=mode,
            document_slug=document_slug,
            limit=5 if mode == "selection" else 10,
        )
        context = self._format_context(documents)

        history = ""
        if conversation_history:
            history_lines = []
            for msg in conversation_history[-5:]:
                role = "User" if msg["role"] == "user" else "Assistant"
                history_lines.append(f"{role}: {msg['content']}")
            history = "\n".join(history_lines)

        if getattr(self, '_llm_available', False) and hasattr(self, 'client'):
            try:
                system_prompt = self._build_system_prompt(context, history)
                stream = self.client.chat.completions.create(
                    model=self.model,
                    messages=[
                        {"role": "system", "content": system_prompt},
                        {"role": "user", "content": question},
                    ],
                    temperature=0.7,
                    max_tokens=1500,
                    stream=True,
                )
                for chunk in stream:
                    if chunk.choices[0].delta.content:
                        yield chunk.choices[0].delta.content
                return
            except Exception as e:
                logger.warning(f"LLM stream failed: {e}")

        # Demo mode fallback
        demo_answer = self._get_demo_answer(question, context)
        yield demo_answer

    def answer_with_selection(
        self,
        question: str,
        selected_text: str,
        document_slug: str,
        conversation_history: Optional[List[Dict[str, str]]] = None,
    ) -> Dict[str, Any]:
        """Answer a question specifically about selected text (Selection RAG)."""
        # Combine selected text with question for more context
        combined_query = f"Selected text: '{selected_text}'\n\nQuestion: {question}"

        # Retrieve from selection collection (within the specific document)
        question_embedding = self._get_embedding(combined_query)
        results = self.qdrant.search_by_document(
            query_embedding=question_embedding,
            document_slug=document_slug,
            limit=3,
        )

        # Build enhanced context with selected text
        selected_context = f"Selected text from current document:\n{selected_text}\n\n"
        if results:
            selected_context += "Related content:\n" + self._format_context(results)
        else:
            selected_context += "No directly related content found."

        # Generate response
        history = ""
        if conversation_history:
            history_lines = []
            for msg in conversation_history[-3:]:
                role = "User" if msg["role"] == "user" else "Assistant"
                history_lines.append(f"{role}: {msg['content']}")
            history = "\n".join(history_lines)

        if getattr(self, '_llm_available', False) and hasattr(self, 'client'):
            try:
                system_prompt = self._build_system_prompt(selected_context, history)
                response = self.client.chat.completions.create(
                    model=self.model,
                    messages=[
                        {"role": "system", "content": system_prompt},
                        {"role": "user", "content": question},
                    ],
                    temperature=0.7,
                    max_tokens=1000,
                )
                answer = response.choices[0].message.content
            except Exception as e:
                logger.warning(f"LLM selection failed: {e}")
                answer = self._get_demo_answer(question, selected_context)
        else:
            answer = self._get_demo_answer(question, selected_context)

        return {
            "answer": answer,
            "sources": [
                {
                    "id": doc["id"],
                    "score": doc["score"],
                    "source": doc.get("metadata", {}).get("source", ""),
                }
                for doc in results
            ],
            "mode": "selection",
            "document_slug": document_slug,
        }

    def ingest_document(
        self,
        document_id: str,
        content: str,
        metadata: Dict[str, Any],
        mode: str = "global",
    ):
        """Ingest a document into the vector store."""
        embedding = self._get_embedding(content)
        self.qdrant.upload_document(
            document_id=document_id,
            content=content,
            embedding=embedding,
            metadata=metadata,
            mode=mode,
        )

    def ingest_markdown(
        self,
        slug: str,
        title: str,
        content: str,
        chapter: str = "",
        mode: str = "global",
    ):
        """Ingest a markdown document with proper metadata."""
        document_id = f"{slug}_{mode}"
        metadata = {
            "source": title,
            "slug": slug,
            "chapter": chapter,
            "type": "markdown",
        }
        self.ingest_document(
            document_id=document_id,
            content=content,
            metadata=metadata,
            mode=mode,
        )
        return document_id


# Global instance
_rag_chain: Optional[RAGChain] = None


def get_rag_chain() -> RAGChain:
    """Get or create the RAG chain instance."""
    global _rag_chain
    if _rag_chain is None:
        _rag_chain = RAGChain()
    return _rag_chain
