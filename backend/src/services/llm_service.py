from openai import OpenAI
from typing import List, Dict, Any, Optional
from src.config import settings


class LLMService:
    """
    Service class to handle interactions with the LLM (OpenRouter via OpenAI Agents SDK).
    """

    def __init__(self):
        """
        Initialize OpenAI client with OpenRouter configuration.
        """
        self.client = OpenAI(
            api_key=settings.openrouter_api_key,
            base_url="https://openrouter.ai/api/v1"
        )
        self.model = "mistralai/devstral-2512:free"

    async def generate_response(self, prompt: str, context_chunks: List[Dict[str, Any]] = None) -> str:
        """
        Generate a response from the LLM based on the prompt and context.

        Args:
            prompt: The user's question or prompt
            context_chunks: List of text chunks to provide as context

        Returns:
            Generated response from the LLM
        """
        try:
            # Build the full prompt with context
            full_prompt = self._build_prompt_with_context(prompt, context_chunks)

            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {
                        "role": "system",
                        "content": "You are an educational assistant that answers questions based on provided textbook content. Answer questions using ONLY the provided context. Do not use any outside knowledge, general knowledge, or make assumptions. Do not hallucinate or create information not present in the context. If the context doesn't contain specific information about modules (e.g. Module 2, Module 3) that are asked about, respond with exactly: 'I don't know based on the textbook.' If the context doesn't contain relevant information at all, respond with exactly: 'I don't know based on the textbook.' NEVER make up information or cite sources that aren't in the provided context."
                    },
                    {
                        "role": "user",
                        "content": full_prompt
                    }
                ],
                max_tokens=1000,
                temperature=0.1  # Even lower temperature for more consistent, fact-based responses
            )

            return response.choices[0].message.content
        except Exception as e:
            print(f"Error generating response from LLM: {e}")
            return "I don't know based on the textbook."

    async def generate_response_with_citations(self, prompt: str, context_chunks: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Generate a response with proper citations based on the context.
        When chunks are available, the model MUST use them to answer the question.
        Only return 'I don't know based on the textbook.' when chunks are found but don't contain relevant information.

        Args:
            prompt: The user's question
            context_chunks: List of text chunks to provide as context

        Returns:
            Dictionary with answer and citations
        """
        try:
            # Build the prompt with instructions to include citations
            context_text = self._format_context_for_prompt(context_chunks)

            # Detect if this is a definition query (asking for definitions, explanations, or what something is)
            is_definition_query = self._is_definition_query(prompt.lower())

            # Adjust the prompt based on query type
            if is_definition_query:
                # For definition queries, ask for concise answers
                full_prompt = f"Provide a concise 2-3 sentence definition/explanation for: '{prompt}'\n\nTextbook Content:\n{context_text}\n\nIMPORTANT: Provide only a short, clear definition in 2-3 sentences. Do not include code, framework details, or long citations unless specifically asked. Use the provided context to answer directly and concisely."

                system_prompt = "You are an educational assistant that provides concise, clear definitions and explanations based on provided textbook content. For definition/explanation questions, respond with only 2-3 sentences. Do not include code, framework details, or long citations unless specifically asked. Answer questions using ONLY the provided context. Do not use any outside knowledge. When relevant context is provided, synthesize it into a concise answer."
            else:
                # For non-definition queries, maintain detailed behavior
                full_prompt = f"Based on the following textbook content, answer the question: '{prompt}'\n\nTextbook Content:\n{context_text}\n\nIMPORTANT: Since you have been provided with relevant textbook content, you MUST answer the question based on this content. Do not respond with 'I don't know based on the textbook.' unless the provided content specifically does not contain information relevant to the question. Answer the question directly using the provided context."

                system_prompt = "You are an educational assistant that answers questions based on provided textbook content. Answer questions using ONLY the provided context. Do not use any outside knowledge, general knowledge, or make assumptions. If the context doesn't contain specific information about modules (e.g. Module 2, Module 3) that are asked about, respond with exactly: 'I don't know based on the textbook.' If the context doesn't contain relevant information at all, respond with exactly: 'I don't know based on the textbook.' Include relevant citations from the context in your response. Format your response as a direct answer to the user's question. When relevant context is provided, you MUST use that context to answer the question."

            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {
                        "role": "system",
                        "content": system_prompt
                    },
                    {
                        "role": "user",
                        "content": full_prompt
                    }
                ],
                max_tokens=500 if is_definition_query else 1000,  # Lower token limit for definitions
                temperature=0.1  # Lower temperature for more consistent, fact-based responses
            )

            answer = response.choices[0].message.content

            # Extract citations from the context chunks
            citations = self._extract_citations(context_chunks)

            return {
                "answer": answer,
                "citations": citations
            }
        except Exception as e:
            print(f"Error generating response with citations: {e}")
            return {
                "answer": "I don't know based on the textbook.",
                "citations": []
            }

    def _build_prompt_with_context(self, prompt: str, context_chunks: Optional[List[Dict[str, Any]]]) -> str:
        """
        Build a prompt that includes the context chunks.
        """
        if not context_chunks:
            return f"Answer the following question: {prompt}\n\nNote: No relevant context was found in the textbook."

        context_text = self._format_context_for_prompt(context_chunks)
        return f"Based on the following textbook content, answer the question: '{prompt}'\n\nTextbook Content:\n{context_text}"

    def _format_context_for_prompt(self, context_chunks: List[Dict[str, Any]]) -> str:
        """
        Format context chunks for inclusion in the prompt.
        Include title, URL, and chunk_index as required for citations.
        """
        formatted_chunks = []
        for i, chunk in enumerate(context_chunks):
            content = chunk.get('content', '')
            title = chunk.get('metadata', {}).get('title', 'Untitled')
            url = chunk.get('metadata', {}).get('url', 'No URL')
            chunk_index = chunk.get('metadata', {}).get('chunk_index', 'N/A')

            formatted_chunk = f"Source {i+1}: {content}\n(Citation: {title} | URL: {url} | Chunk Index: {chunk_index})"
            formatted_chunks.append(formatted_chunk)

        return "\n\n".join(formatted_chunks)

    def _extract_citations(self, context_chunks: List[Dict[str, Any]]) -> List[Dict[str, str]]:
        """
        Extract citations from context chunks with title, URL, and chunk_index.
        """
        citations = []
        unique_citations = set()

        for chunk in context_chunks:
            # Create a unique identifier for this chunk based on URL and chunk_index
            url = chunk.get('metadata', {}).get('url', '')
            chunk_index = chunk.get('metadata', {}).get('chunk_index', 'N/A')
            title = chunk.get('metadata', {}).get('title', 'Untitled')

            # Create a unique key for deduplication
            unique_key = f"{url}_{chunk_index}"

            if unique_key not in unique_citations:
                citation_obj = {
                    "text": chunk.get('content', '')[:100] + "...",  # Use content preview as text
                    "source": f"{title} (Chunk {chunk_index})",  # Include title and chunk index in source
                    "url": url
                }
                citations.append(citation_obj)
                unique_citations.add(unique_key)

        return citations

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


# Singleton instance
llm_service = LLMService()