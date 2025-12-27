from typing import List, Dict, Any, Optional, Union
from src.services.qdrant_service import qdrant_service
from src.services.embedding_service import embedding_service
from src.services.llm_service import llm_service
from src.models.rag_models import Query, RAGResponseModel, TextChunk, Citation
from src.utils.logging import get_logger, log_performance
import time
import uuid
from datetime import datetime


class RAGService:
    """
    Service class to orchestrate the RAG pipeline: embedding -> search -> LLM generation.
    """

    def __init__(self):
        self.logger = get_logger(__name__)

    async def query_textbook(self, query_text: str, max_chunks: int = 5, similarity_threshold: float = 0.5, metadata_filters: Optional[Dict[str, Any]] = None) -> RAGResponseModel:
        """
        Main method to process a query through the RAG pipeline.

        Args:
            query_text: The user's question about textbook content
            max_chunks: Maximum number of text chunks to retrieve
            similarity_threshold: Minimum similarity score for retrieved chunks
            metadata_filters: Optional filters for metadata fields (e.g., {'module': 'Module 1'})

        Returns:
            RAGResponseModel containing the answer and citations
        """
        start_time = time.time()
        self.logger.info(f"Processing RAG query: {query_text[:100]}...")

        # Check if the query is a greeting or very short social message
        if self._is_greeting_or_social_message(query_text):
            self.logger.info("Detected greeting/social message, returning static response")
            response = RAGResponseModel(
                answer="Hello! How can I assist you with the textbook content?",
                citations=[],
                chunks_used=[],
                query=query_text
            )
            log_performance("greeting_response", time.time() - start_time, {"type": "greeting"})
            return response

        try:
            # Step 1: Generate embedding for the query
            query_embedding = await embedding_service.generate_embedding(query_text)
            self.logger.debug(f"Generated embedding with {len(query_embedding)} dimensions")

            # Step 2: Search for relevant chunks in Qdrant
            relevant_chunks = await qdrant_service.search_text_chunks(
                query_vector=query_embedding,
                limit=max_chunks,
                similarity_threshold=similarity_threshold,
                metadata_filters=metadata_filters
            )
            self.logger.debug(f"Found {len(relevant_chunks)} relevant chunks")

            # Step 3: If no chunks found, return the fallback response
            if not relevant_chunks:
                self.logger.info("No relevant chunks found, returning fallback response")
                # Return exactly as specified in requirements: "I don't know based on the textbook."
                response = RAGResponseModel(
                    answer="I don't know based on the textbook.",
                    citations=[],
                    chunks_used=[],
                    query=query_text
                )
                log_performance("rag_query", time.time() - start_time, {"chunks_found": 0})
                return response

            # Step 4: Generate response using LLM with context
            # When chunks are found, the LLM service is configured to use them and avoid fallback responses
            result = await llm_service.generate_response_with_citations(query_text, relevant_chunks)

            # Detect if this is a definition query to control citation output
            is_definition_query = self._is_definition_query(query_text.lower())

            # Step 4a: Ensure the response contains proper citations and chunks when chunks were provided
            # CRITICAL FIX: If the LLM returned "I don't know" despite having chunks, we should NOT return that
            # Instead, we should either try to generate a better response or at least acknowledge the chunks exist
            if result["answer"] == "I don't know based on the textbook." and relevant_chunks:
                self.logger.warning(f"LLM returned fallback response despite having {len(relevant_chunks)} relevant chunks")
                # IMPORTANT: When chunks are available, we should NOT return "I don't know"
                # Instead, we'll provide a response that acknowledges the available content
                self.logger.info("Overriding fallback response because chunks were found")

                # Create a response that acknowledges the available chunks but indicates lack of specific info
                chunk_titles = [chunk.get('metadata', {}).get('title', 'Unknown') for chunk in relevant_chunks[:3]]  # First 3 titles
                available_content = ", ".join(set(chunk_titles))  # Remove duplicates

                result = {
                    "answer": f"Based on the available textbook content ({available_content}), I cannot find specific information to answer your question: '{query_text}'. The system found {len(relevant_chunks)} relevant chunks but they don't contain the specific information needed.",
                    "citations": result["citations"]  # Keep the citations that were generated
                }

            # Step 5: Format the response to match the required schema exactly
            # Ensure we maintain the exact structure expected by the frontend
            chunks_used = []
            citations = []

            # For definition queries, exclude citations and chunks to keep response concise
            if not is_definition_query:
                for chunk in relevant_chunks:
                    # Create TextChunk objects ensuring all required fields are present
                    text_chunk = TextChunk(
                        id=chunk.get('id', ''),
                        content=chunk.get('content', ''),
                        metadata=chunk.get('metadata', {}),
                        similarity_score=chunk.get('similarity_score', 0.0)
                    )
                    chunks_used.append(text_chunk)

                    # Create citations with title + url + chunk_index + module/section info as required
                    metadata = chunk.get('metadata', {})
                    module_info = []
                    if metadata.get('module'):
                        module_info.append(metadata['module'])
                    if metadata.get('section'):
                        module_info.append(metadata['section'])
                    if metadata.get('subsection'):
                        module_info.append(metadata['subsection'])

                    source_parts = [metadata.get('title', chunk.get('title', ''))]
                    if module_info:
                        source_parts.append(f"({' | '.join(module_info)})")
                    source_parts.append(f"Chunk {metadata.get('chunk_index', 'N/A')}")

                    citation_obj = Citation(
                        text=chunk.get('content', '')[:100] + "...",  # Use content preview as text
                        source=" ".join(source_parts),
                        url=chunk.get('metadata', {}).get('url', chunk.get('url', ''))
                    )
                    citations.append(citation_obj)

            response = RAGResponseModel(
                answer=result["answer"],
                citations=citations,
                chunks_used=chunks_used,
                query=query_text
            )

            log_performance("rag_query", time.time() - start_time, {
                "chunks_found": len(relevant_chunks),
                "response_length": len(result["answer"])
            })

            return response

        except Exception as e:
            self.logger.error(f"Error in RAG query processing: {e}", exc_info=True)
            log_performance("rag_query", time.time() - start_time, {"error": str(e)})

            # Return fallback response in case of error
            return RAGResponseModel(
                answer="I don't know based on the textbook.",
                citations=[],
                chunks_used=[],
                query=query_text
            )

    async def verify_textbook_content_exists(self) -> bool:
        """
        Verify that textbook content exists in the Qdrant collection.
        """
        try:
            return await qdrant_service.verify_collection_exists()
        except Exception as e:
            self.logger.error(f"Error verifying textbook content: {e}", exc_info=True)
            return False

    async def get_collection_info(self) -> Optional[Dict[str, Any]]:
        """
        Get information about the textbook collection in Qdrant.
        """
        try:
            return await qdrant_service.get_collection_info()
        except Exception as e:
            self.logger.error(f"Error getting collection info: {e}", exc_info=True)
            return None


    def _is_definition_query(self, prompt: str) -> bool:
        """
        Detect if the query is asking for a definition or explanation.

        Args:
            prompt: The user's question in lowercase

        Returns:
            True if the query is likely asking for a definition, False otherwise
        """
        # Common phrases that indicate definition queries
        definition_indicators = [
            "what is ",
            "what's ",
            "define ",
            "definition of ",
            "explain ",
            "what does ",
            "meaning of ",
            "describe ",
            "what are ",
            "what was ",
            "what were ",
            "what do you mean by ",
            "define what ",
            "what exactly is ",
            "what is the definition of ",
            "what is meant by ",
            "can you explain ",
            "tell me about ",
            "what is a ",
            "what is an ",
        ]

        prompt_lower = prompt.strip()

        # Check if the prompt starts with or contains any definition indicators
        for indicator in definition_indicators:
            if indicator in prompt_lower:
                return True

        # Additional check for simple "what is X" patterns at the beginning
        if prompt_lower.startswith("what is ") or prompt_lower.startswith("what's "):
            # Make sure it's not asking about a specific module or section
            if not any(keyword in prompt_lower for keyword in ["module", "section", "chapter", "page", "figure", "table"]):
                return True

        return False

    def _is_greeting_or_social_message(self, query: str) -> bool:
        """
        Detect if the query is a greeting or very short social message.

        Args:
            query: The user's input

        Returns:
            True if the query is likely a greeting or social message, False otherwise
        """
        # Normalize the query
        query_lower = query.strip().lower()

        # Common greetings and social messages
        greetings = [
            "hello", "hi", "hey", "greetings", "good morning", "good afternoon",
            "good evening", "good day", "howdy", "hi there", "hello there"
        ]

        # Check for exact matches or simple greetings
        if query_lower in greetings:
            return True

        # Check for very short messages that are likely greetings
        words = query_lower.split()
        if len(words) <= 2:
            # Check if the short message contains greeting words
            for greeting in greetings:
                if greeting in query_lower:
                    return True

        # Additional check for common short social phrases
        if query_lower in ["hello!", "hi!", "hey!", "hello.", "hi.", "hey.", "ok", "thanks", "thank you"]:
            return True

        return False


# Singleton instance
rag_service = RAGService()


# Compatibility classes for existing API
from abc import ABC, abstractmethod
from src.models.api_models import QueryResponse as APIQueryResponse, CitationReference


class RAGServiceInterface(ABC):
    """Abstract base class for RAG agent services."""

    @abstractmethod
    async def query(self, query_text: str, user_id: Optional[str] = None, session_id: Optional[str] = None) -> APIQueryResponse:
        """
        Process a query and return a grounded response with citations.
        """
        pass

    @abstractmethod
    async def validate_query(self, query_text: str) -> bool:
        """
        Validate the query before processing.
        """
        pass


class SimpleRAGService(RAGServiceInterface):
    """Simple implementation of RAG service for compatibility with existing API."""

    def __init__(self):
        self._rag_service = RAGService()

    async def query(self, query_text: str, user_id: Optional[str] = None, session_id: Optional[str] = None) -> APIQueryResponse:
        """
        Process a query and return a grounded response with citations.
        """
        # Use the existing RAG pipeline implementation
        response = await self._rag_service.query_textbook(query_text)

        # Convert from our internal model to the expected API model
        import uuid
        from datetime import datetime

        # Generate IDs
        response_id = str(uuid.uuid4())
        query_id = str(uuid.uuid4())

        # Convert citations to the expected format
        citations = []
        for citation in response.citations:
            citations.append(CitationReference(
                id=f"cit-{uuid.uuid4()}",
                text=citation.text,
                sourceUrl=citation.url or "",
                pageReference=citation.source,  # This now includes title and chunk index
                context="Relevant textbook content used to answer the query"
            ))

        # Create the API response
        api_response = APIQueryResponse(
            id=response_id,
            queryId=query_id,
            content=response.answer,
            citations=citations,
            confidence=0.85,  # Default confidence
            timestamp=datetime.utcnow().isoformat() + "Z"
        )

        return api_response

    async def validate_query(self, query_text: str) -> bool:
        """
        Validate the query before processing.
        """
        if not query_text or len(query_text.strip()) < 3:
            return False
        if len(query_text) > 1000:
            return False
        return True