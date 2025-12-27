import asyncio
from typing import List
import numpy as np
from config.settings import settings

async def get_embedding(text: str, model: str = "text-embedding-ada-002") -> List[float]:
    """
    Generate embedding for the given text using OpenAI's embedding model.

    Args:
        text: Input text to embed
        model: OpenAI embedding model to use (default: text-embedding-ada-002)

    Returns:
        List of floats representing the embedding vector
    """
    try:
        # Use the new OpenAI SDK format
        from openai import OpenAI
        client = OpenAI(api_key=settings.openai_api_key)

        response = client.embeddings.create(
            input=text,
            model=model
        )

        return response.data[0].embedding
    except Exception as e:
        print(f"Error generating embedding: {str(e)}")
        raise

def get_embedding_sync(text: str, model: str = "text-embedding-ada-002") -> List[float]:
    """
    Synchronous version of get_embedding for use in non-async contexts.
    """
    return asyncio.run(get_embedding(text, model))

async def get_embeddings(texts: List[str], model: str = "text-embedding-ada-002") -> List[List[float]]:
    """
    Generate embeddings for multiple texts.

    Args:
        texts: List of input texts to embed
        model: OpenAI embedding model to use (default: text-embedding-ada-002)

    Returns:
        List of embedding vectors (each vector is a list of floats)
    """
    try:
        from openai import OpenAI
        client = OpenAI(api_key=settings.openai_api_key)

        response = client.embeddings.create(
            input=texts,
            model=model
        )

        return [data.embedding for data in response.data]
    except Exception as e:
        print(f"Error generating embeddings: {str(e)}")
        raise

async def find_similar_texts(
    query_text: str,
    candidate_texts: List[str],
    top_k: int = 5,
    model: str = "text-embedding-ada-002"
) -> List[tuple]:
    """
    Find the most similar texts to the query from a list of candidates.

    Args:
        query_text: The text to compare against
        candidate_texts: List of candidate texts to compare with
        top_k: Number of most similar texts to return
        model: OpenAI embedding model to use

    Returns:
        List of tuples (text, similarity_score) sorted by similarity (highest first)
    """
    # Get embeddings for query and all candidates
    all_texts = [query_text] + candidate_texts
    embeddings = await get_embeddings(all_texts, model)

    # Extract query embedding and candidate embeddings
    query_embedding = embeddings[0]
    candidate_embeddings = embeddings[1:]

    # Calculate cosine similarity
    similarities = []
    for i, candidate_embedding in enumerate(candidate_embeddings):
        similarity = cosine_similarity(query_embedding, candidate_embedding)
        similarities.append((candidate_texts[i], similarity))

    # Sort by similarity score (descending)
    similarities.sort(key=lambda x: x[1], reverse=True)

    # Return top_k results
    return similarities[:top_k]

def cosine_similarity(vec1: List[float], vec2: List[float]) -> float:
    """
    Calculate cosine similarity between two vectors.

    Args:
        vec1: First vector
        vec2: Second vector

    Returns:
        Cosine similarity score between -1 and 1
    """
    # Convert to numpy arrays for calculation
    v1 = np.array(vec1)
    v2 = np.array(vec2)

    # Calculate cosine similarity
    dot_product = np.dot(v1, v2)
    norm_v1 = np.linalg.norm(v1)
    norm_v2 = np.linalg.norm(v2)

    if norm_v1 == 0 or norm_v2 == 0:
        return 0.0

    return float(dot_product / (norm_v1 * norm_v2))

async def preprocess_query(query: str) -> str:
    """
    Preprocess a query string before embedding.

    Args:
        query: Raw query string

    Returns:
        Preprocessed query string
    """
    # Basic preprocessing: strip whitespace and normalize
    processed = query.strip()

    # Additional preprocessing could include:
    # - Removing special characters
    # - Handling contractions
    # - Normalizing case (depending on use case)

    return processed