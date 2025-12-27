import asyncio
import logging
import re
from typing import List, Dict, Any
from openai import OpenAI
from config.settings import settings
from models.text_query import TextQuery
from models.retrieved_chunk import RetrievedChunk
from models.grounded_response import GroundedResponse
from services.retrieval_service import retrieval_service
from utils.embedding_utils import get_embedding
from utils.error_handler import QueryProcessingError, RetrievalError, ResponseGenerationError

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RAGAgent:
    """
    RAG (Retrieval-Augmented Generation) Agent that uses OpenAI-compatible SDK to connect to
    Gemini API to answer user questions by retrieving relevant chunks from the Qdrant rag_embedding collection.
    """

    def __init__(self, top_k: int = 5, min_score: float = 0.3):
        """
        Initialize the RAG Agent.

        Args:
            top_k: Number of chunks to retrieve (default: 5)
            min_score: Minimum similarity score for retrieved chunks (default: 0.3)
        """
        self.top_k = top_k
        self.min_score = min_score
        self.client = OpenAI(
            api_key=settings.gemini_api_key,
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
        )
        logger.info("RAG Agent initialized")

    async def invoke(self, query: str) -> str:
        """
        Primary method for interacting with the RAG agent.

        Args:
            query: Text query from user

        Returns:
            str: Grounded response based on retrieved content

        Raises:
            QueryProcessingError: If query processing fails
            RetrievalError: If document retrieval fails
            ResponseGenerationError: If response generation fails
        """
        try:
            logger.info(f"Processing query: {query[:50]}...")

            # Validate the query
            if not query or not query.strip():
                raise QueryProcessingError("Query cannot be empty")

            if len(query.strip()) < 3:
                raise QueryProcessingError("Query is too short, must be at least 3 characters")

            # Create a TextQuery model
            text_query = TextQuery(query_text=query)

            # Retrieve relevant chunks from Qdrant
            retrieved_data = await retrieval_service.retrieve_chunks(
                query_text=text_query.query_text,
                top_k=self.top_k,
                min_score=self.min_score
            )

            # Convert retrieved data to RetrievedChunk models
            retrieved_chunks = []
            for data in retrieved_data:
                chunk = RetrievedChunk(
                    id=data["id"],
                    content=data["content"],
                    score=data["score"],
                    metadata=data.get("metadata", {}),
                    source=data.get("metadata", {}).get("source")
                )
                retrieved_chunks.append(chunk)

            logger.info(f"Retrieved {len(retrieved_chunks)} chunks for query")

            # Handle case where no relevant documents are found
            if not retrieved_chunks:
                logger.info("No relevant documents found in Qdrant for the given query")
                return "I couldn't find any relevant information in the knowledge base to answer your question. The query may be too specific or the information may not be available in the current knowledge base."

            # Generate response using OpenAI based on retrieved chunks
            response_text = await self._generate_response(text_query.query_text, retrieved_chunks)

            # Validate that the response is grounded in the retrieved content
            is_grounded = await self._validate_response_grounding(response_text, retrieved_chunks)
            if not is_grounded and retrieved_chunks:
                logger.warning("Response may not be fully grounded in retrieved content")

            # Create and return the grounded response
            grounded_response = GroundedResponse(
                response_text=response_text,
                source_chunks=retrieved_chunks
            )

            logger.info("Successfully generated grounded response")
            return grounded_response.response_text

        except QueryProcessingError:
            raise
        except RetrievalError as e:
            logger.error(f"Retrieval error: {str(e)}")
            return "I'm having trouble accessing the knowledge base right now. Please try again later."
        except Exception as e:
            logger.error(f"Error in agent.invoke: {str(e)}")
            raise QueryProcessingError(f"Failed to process query: {str(e)}")

    async def _generate_response(self, query: str, retrieved_chunks: List[RetrievedChunk]) -> str:
        """
        Generate a response using OpenAI based on the query and retrieved chunks.

        Args:
            query: Original user query
            retrieved_chunks: List of retrieved chunks to use as context

        Returns:
            str: Generated response grounded in the retrieved content
        """
        try:
            if not retrieved_chunks:
                return "I couldn't find any relevant information to answer your question."

            # Format the context from retrieved chunks
            context = "\n\n".join([f"Context {i+1}: {chunk.content}" for i, chunk in enumerate(retrieved_chunks)])

            # Create the prompt for OpenAI
            prompt = f"""
            Answer the user's question based ONLY on the provided context.
            Do not use any external knowledge or make up information.
            If the context doesn't contain enough information to answer the question, say so.
            Ensure your answer is factual and directly based on the provided context.

            Context:
            {context}

            Question: {query}

            Answer:
            """

            # Call Gemini API to generate the response
            response = self.client.chat.completions.create(
                model="gemini-2.0-flash",  # Using Gemini model via OpenAI-compatible endpoint
                messages=[
                    {"role": "system", "content": "You are a helpful assistant that answers questions based only on the provided context. Do not make up information or use external knowledge. Ensure all claims are directly supported by the provided context."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.3,  # Lower temperature for more consistent answers
                max_tokens=500
            )

            return response.choices[0].message.content.strip()

        except Exception as e:
            logger.error(f"Error generating response: {str(e)}")
            raise ResponseGenerationError(f"Failed to generate response: {str(e)}")

    async def _validate_response_grounding(self, response: str, retrieved_chunks: List[RetrievedChunk]) -> bool:
        """
        Validate that the response is grounded in the retrieved content.

        This is a basic implementation that checks if key terms from the chunks
        appear in the response. A more sophisticated implementation would use
        semantic similarity or other NLP techniques.

        Args:
            response: The generated response to validate
            retrieved_chunks: The chunks that were used as context

        Returns:
            bool: True if the response appears to be grounded in the chunks
        """
        if not retrieved_chunks:
            return True  # If no chunks, we can't validate grounding

        # Extract key terms/phrases from the retrieved chunks
        chunk_content = " ".join([chunk.content for chunk in retrieved_chunks])

        # Simple validation: check if response contains terms from the chunks
        response_lower = response.lower()
        chunk_lower = chunk_content.lower()

        # Extract words that appear in both response and chunks (excluding common stop words)
        response_words = set(re.findall(r'\b\w+\b', response_lower))
        chunk_words = set(re.findall(r'\b\w+\b', chunk_lower))

        # Define common stop words to exclude from the comparison
        stop_words = {
            'the', 'a', 'an', 'and', 'or', 'but', 'in', 'on', 'at', 'to', 'for',
            'of', 'with', 'by', 'is', 'are', 'was', 'were', 'be', 'been', 'being',
            'have', 'has', 'had', 'do', 'does', 'did', 'will', 'would', 'could',
            'should', 'may', 'might', 'must', 'can', 'this', 'that', 'these', 'those'
        }

        # Find intersection of words, excluding stop words
        content_words_in_response = response_words - stop_words
        content_words_in_chunks = chunk_words - stop_words

        # Calculate overlap
        common_words = content_words_in_response.intersection(content_words_in_chunks)
        total_content_words_in_response = len(content_words_in_response)

        if total_content_words_in_response == 0:
            # If response has no content words, it's not grounded
            return len(common_words) > 0

        # Consider it grounded if at least 30% of content words in response appear in chunks
        grounding_ratio = len(common_words) / total_content_words_in_response if total_content_words_in_response > 0 else 0
        is_grounded = grounding_ratio >= 0.3  # 30% threshold

        logger.debug(f"Grounding validation: {grounding_ratio:.2%} of response content words found in chunks")

        return is_grounded

    async def validate_connection(self) -> bool:
        """
        Validate that all required services are available.

        Returns:
            True if all services are available, False otherwise
        """
        try:
            # Validate Qdrant connection
            qdrant_ok = await retrieval_service.validate_connection()

            # Validate Gemini connection by making a simple call
            try:
                test_response = self.client.chat.completions.create(
                    model="gemini-2.0-flash",
                    messages=[{"role": "user", "content": "test"}],
                    max_tokens=5
                )
                gemini_ok = True
            except:
                gemini_ok = False

            return qdrant_ok and gemini_ok

        except Exception as e:
            logger.error(f"Error validating connections: {str(e)}")
            return False


if __name__ == "__main__":
    import asyncio
    import os
    from dotenv import load_dotenv

    # Load environment variables
    load_dotenv()

    async def test_agent():
        # Create an instance of the RAG agent
        agent = RAGAgent(top_k=5, min_score=0.3)

        # Test query
        query = "Explain Gazebo physics simulation in Module 3"

        print(f"Query: {query}")
        print("Processing...\n")

        try:
            # Call the invoke method
            result = await agent.invoke(query)
            print("Result:")
            print(result)
        except Exception as e:
            print(f"Error occurred: {e}")

    # Run the test
    asyncio.run(test_agent())