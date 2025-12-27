#!/usr/bin/env python3
"""
Retrieval Validation Script

This script validates that all embeddings stored in Qdrant can be successfully retrieved
and that the ingestion pipeline works end-to-end. It connects to the Qdrant collection,
fetches stored points and metadata, runs similarity searches, validates chunk integrity,
and generates comprehensive retrieval reports.
"""

import os
import sys
import logging
import argparse
from typing import Dict, List, Optional, Any, Tuple
from datetime import datetime
import json
import time
import hashlib
from dataclasses import dataclass, asdict
from pathlib import Path

# Import required dependencies
try:
    from qdrant_client import QdrantClient
    from qdrant_client.http import models
    import cohere
    import requests
    from dotenv import load_dotenv
    from agent import RAGAgent
    import asyncio
except ImportError as e:
    print(f"Missing required dependency: {e}")
    print("Please install required dependencies: pip install qdrant-client cohere requests python-dotenv openai")
    sys.exit(1)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('retrieval_validation.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# Load environment variables
load_dotenv()


@dataclass
class StoredVector:
    """Represents a vector stored in Qdrant with its metadata"""
    id: str
    vector: List[float]
    payload: Dict[str, Any]
    validation_status: str = "unknown"


@dataclass
class ValidationResult:
    """Represents the result of a validation operation"""
    test_id: str
    timestamp: str
    vector_count: int
    success_count: int
    error_count: int
    integrity_score: float
    retrieval_accuracy: Optional[float] = None
    processing_time: Optional[float] = None


@dataclass
class SearchQuery:
    """Represents a search query for validation"""
    query_text: str
    expected_results: Optional[List[str]] = None
    actual_results: Optional[List[Dict[str, Any]]] = None
    precision_score: Optional[float] = None
    confidence_threshold: float = 0.7


@dataclass
class ChunkValidation:
    """Represents validation of a specific chunk"""
    chunk_id: str
    original_content: Optional[str] = None
    retrieved_content: Optional[str] = None
    url_match: bool = False
    content_similarity: float = 0.0
    validation_timestamp: str = ""


@dataclass
class RetrievalReport:
    """Represents a comprehensive retrieval validation report"""
    report_id: str
    timestamp: str
    validation_results: List[ValidationResult]
    sample_queries: List[SearchQuery]
    sample_results: List[Dict[str, Any]]
    performance_metrics: Dict[str, float]
    integrity_summary: Dict[str, Any]
    recommendations: List[str]


def load_config() -> Dict[str, str]:
    """
    Load configuration from environment variables.

    Returns:
        Dictionary containing configuration values
    """
    config = {
        'cohere_api_key': os.getenv('COHERE_API_KEY', ''),
        'qdrant_url': os.getenv('QDRANT_URL', ''),
        'qdrant_api_key': os.getenv('QDRANT_API_KEY', ''),
        'target_website_url': os.getenv('TARGET_WEBSITE_URL', 'https://physical-ai-humanoid-robotics-textb-nu.vercel.app/'),
        'sitemap_url': os.getenv('SITEMAP_URL', 'https://physical-ai-humanoid-robotics-textb-nu.vercel.app/sitemap.xml'),
        'collection_name': os.getenv('QDRANT_COLLECTION_NAME', 'rag_embedding')
    }

    # Validate required configuration
    required_keys = ['cohere_api_key', 'qdrant_url', 'qdrant_api_key']
    missing_keys = [key for key in required_keys if not config[key]]

    if missing_keys:
        raise ValueError(f"Missing required configuration: {', '.join(missing_keys)}")

    logger.info("Configuration loaded successfully")
    return config


def connect_to_qdrant(config: Dict[str, str]) -> QdrantClient:
    """
    Create and return a Qdrant client connection.

    Args:
        config: Configuration dictionary containing Qdrant connection parameters

    Returns:
        QdrantClient instance
    """
    try:
        client = QdrantClient(
            url=config['qdrant_url'],
            api_key=config['qdrant_api_key'],
            prefer_grpc=False  # Using REST API
        )

        # Test the connection
        client.get_collections()
        logger.info("Successfully connected to Qdrant")
        return client
    except Exception as e:
        logger.error(f"Failed to connect to Qdrant: {e}")
        raise


def initialize_cohere_client(config: Dict[str, str]) -> cohere.Client:
    """
    Initialize and return a Cohere client.

    Args:
        config: Configuration dictionary containing Cohere API key

    Returns:
        Cohere Client instance
    """
    try:
        client = cohere.Client(api_key=config['cohere_api_key'])
        logger.info("Cohere client initialized successfully")
        return client
    except Exception as e:
        logger.error(f"Failed to initialize Cohere client: {e}")
        raise


def embed_texts(texts: List[str], cohere_client: cohere.Client) -> List[List[float]]:
    """
    Generate embeddings for a list of texts using Cohere.

    Args:
        texts: List of text strings to embed
        cohere_client: Initialized Cohere client

    Returns:
        List of embedding vectors
    """
    try:
        response = cohere_client.embed(
            texts=texts,
            model='embed-english-v3.0',  # Using a standard embedding model
            input_type='search_document'  # Appropriate for document search
        )

        embeddings = [embedding for embedding in response.embeddings]
        logger.info(f"Generated embeddings for {len(texts)} text(s)")
        return embeddings
    except Exception as e:
        logger.error(f"Failed to generate embeddings: {e}")
        raise


def fetch_all_vectors(qdrant_client: QdrantClient, collection_name: str, limit: int = 1000) -> List[StoredVector]:
    """
    Fetch all vectors from the specified Qdrant collection.

    Args:
        qdrant_client: Initialized Qdrant client
        collection_name: Name of the collection to fetch from
        limit: Maximum number of vectors to fetch (default: 1000)

    Returns:
        List of StoredVector objects
    """
    try:
        # Use scroll to fetch all points efficiently
        all_vectors = []
        offset = None
        total_fetched = 0

        while total_fetched < limit:
            # Fetch a batch of points
            points, next_offset = qdrant_client.scroll(
                collection_name=collection_name,
                limit=min(100, limit - total_fetched),  # Fetch up to 100 at a time or remaining
                offset=offset,
                with_payload=True,
                with_vectors=True
            )

            for point in points:
                stored_vector = StoredVector(
                    id=str(point.id),
                    vector=point.vector if hasattr(point, 'vector') else [],
                    payload=point.payload if point.payload else {}
                )
                all_vectors.append(stored_vector)

            total_fetched += len(points)

            # If no more points or reached limit, break
            if not points or next_offset is None or total_fetched >= limit:
                break

            offset = next_offset

        logger.info(f"Fetched {len(all_vectors)} vectors from collection '{collection_name}'")
        return all_vectors
    except Exception as e:
        logger.error(f"Failed to fetch vectors from Qdrant: {e}")
        raise


def get_collection_info(qdrant_client: QdrantClient, collection_name: str) -> Dict[str, Any]:
    """
    Get information about a Qdrant collection.

    Args:
        qdrant_client: Initialized Qdrant client
        collection_name: Name of the collection to get info for

    Returns:
        Dictionary with collection information
    """
    try:
        collection_info = qdrant_client.get_collection(collection_name=collection_name)

        # Extract relevant information from the collection
        # Safely access the collection info attributes
        config = getattr(collection_info, 'config', None)
        if config and hasattr(config, 'params'):
            params = getattr(config, 'params', {})
            # Handle both dict and object access for vectors_count
            if isinstance(params, dict):
                vectors_count = params.get('vectors_count', 0)
            else:
                vectors_count = getattr(params, 'vectors_count', 0)
        else:
            # Try to get vectors_count directly from collection_info if available
            vectors_count = getattr(collection_info, 'vectors_count', 0)

        # Get other counts
        indexed_vectors_count = getattr(collection_info, 'indexed_vectors_count', 0)
        points_count = getattr(collection_info, 'points_count', 0)

        info = {
            'name': collection_name,
            'vectors_count': vectors_count,
            'indexed_vectors_count': indexed_vectors_count,
            'points_count': points_count
        }

        logger.info(f"Collection '{collection_name}' info: {info}")
        return info
    except Exception as e:
        logger.error(f"Failed to get collection info: {e}")
        raise


def connect_to_qdrant_with_validation(config: Dict[str, str]) -> Tuple[QdrantClient, Dict[str, Any]]:
    """
    Connect to Qdrant and validate the connection by checking collection existence.

    Args:
        config: Configuration dictionary containing Qdrant connection parameters

    Returns:
        Tuple of (QdrantClient instance, collection info dictionary)
    """
    client = connect_to_qdrant(config)

    # Validate that the collection exists
    collection_name = config['collection_name']
    try:
        collection_info = get_collection_info(client, collection_name)
        logger.info(f"Successfully connected to Qdrant collection '{collection_name}'")
        return client, collection_info
    except Exception as e:
        logger.error(f"Collection '{collection_name}' does not exist or is not accessible: {e}")
        raise


def validate_vector_structure(stored_vector: StoredVector) -> bool:
    """
    Validate the structure of a stored vector including its metadata.

    Args:
        stored_vector: StoredVector object to validate

    Returns:
        Boolean indicating if the vector structure is valid
    """
    try:
        # Check if required fields exist
        if not stored_vector.id:
            logger.warning(f"Vector with empty ID: {stored_vector}")
            return False

        if not stored_vector.vector or len(stored_vector.vector) == 0:
            logger.warning(f"Vector with empty embedding: ID {stored_vector.id}")
            return False

        if not stored_vector.payload:
            logger.warning(f"Vector with empty payload: ID {stored_vector.id}")
            return False

        # Check for required payload fields
        required_payload_fields = ['content', 'url']
        missing_fields = []
        for field in required_payload_fields:
            if field not in stored_vector.payload:
                missing_fields.append(field)

        if missing_fields:
            logger.warning(f"Vector missing payload fields {missing_fields}: ID {stored_vector.id}")
            return False

        # Validate content field
        content = stored_vector.payload.get('content', '')
        if not content or not isinstance(content, str):
            logger.warning(f"Vector has invalid content in payload: ID {stored_vector.id}")
            return False

        # Validate url field
        url = stored_vector.payload.get('url', '')
        if not url or not isinstance(url, str):
            logger.warning(f"Vector has invalid URL in payload: ID {stored_vector.id}")
            return False

        # Validate optional fields if present
        chunk_index = stored_vector.payload.get('chunk_index')
        if chunk_index is not None and not isinstance(chunk_index, (int, float)):
            logger.warning(f"Vector has invalid chunk_index in payload: ID {stored_vector.id}")
            return False

        source_title = stored_vector.payload.get('source_title')
        if source_title is not None and not isinstance(source_title, str):
            logger.warning(f"Vector has invalid source_title in payload: ID {stored_vector.id}")
            return False

        logger.debug(f"Vector structure validation passed: ID {stored_vector.id}")
        return True

    except Exception as e:
        logger.error(f"Error validating vector structure: {e}")
        return False


def validate_vector_list_structure(vectors: List[StoredVector]) -> ValidationResult:
    """
    Validate the structure of a list of stored vectors.

    Args:
        vectors: List of StoredVector objects to validate

    Returns:
        ValidationResult object with validation metrics
    """
    start_time = time.time()

    total_vectors = len(vectors)
    success_count = 0
    error_count = 0

    for vector in vectors:
        if validate_vector_structure(vector):
            success_count += 1
        else:
            error_count += 1

    processing_time = time.time() - start_time
    integrity_score = success_count / total_vectors if total_vectors > 0 else 0

    validation_result = ValidationResult(
        test_id=f"vector_structure_validation_{int(time.time())}",
        timestamp=datetime.now().isoformat(),
        vector_count=total_vectors,
        success_count=success_count,
        error_count=error_count,
        integrity_score=integrity_score,
        processing_time=processing_time
    )

    logger.info(f"Vector structure validation completed: {success_count}/{total_vectors} valid ({integrity_score:.2%})")
    return validation_result


def run_vector_retrieval_validation(config: Dict[str, str], limit: int = 1000) -> ValidationResult:
    """
    Execute the main validation workflow for vector retrieval.

    Args:
        config: Configuration dictionary
        limit: Maximum number of vectors to validate

    Returns:
        ValidationResult object with validation results
    """
    start_time = time.time()
    logger.info(f"Starting vector retrieval validation with limit: {limit}")

    try:
        # Connect to Qdrant and validate collection
        qdrant_client, collection_info = connect_to_qdrant_with_validation(config)
        logger.info(f"Collection info: {collection_info}")

        # Fetch vectors from Qdrant
        logger.info("Fetching vectors from Qdrant...")
        stored_vectors = fetch_all_vectors(qdrant_client, config['collection_name'], limit=limit)

        if not stored_vectors:
            logger.warning("No vectors found in the collection")
            return ValidationResult(
                test_id=f"vector_retrieval_validation_{int(time.time())}",
                timestamp=datetime.now().isoformat(),
                vector_count=0,
                success_count=0,
                error_count=0,
                integrity_score=0.0,
                processing_time=time.time() - start_time
            )

        # Validate vector structures
        logger.info(f"Validating structure of {len(stored_vectors)} vectors...")
        validation_result = validate_vector_list_structure(stored_vectors)

        processing_time = time.time() - start_time
        validation_result.processing_time = processing_time

        logger.info(f"Vector retrieval validation completed in {processing_time:.2f} seconds")
        return validation_result

    except Exception as e:
        logger.error(f"Error during vector retrieval validation: {e}")
        processing_time = time.time() - start_time
        return ValidationResult(
            test_id=f"vector_retrieval_validation_{int(time.time())}",
            timestamp=datetime.now().isoformat(),
            vector_count=0,
            success_count=0,
            error_count=1,
            integrity_score=0.0,
            processing_time=processing_time
        )


def generate_test_embeddings(queries: List[str], cohere_client: cohere.Client) -> List[List[float]]:
    """
    Generate embeddings for test queries using Cohere.

    Args:
        queries: List of query strings to embed
        cohere_client: Initialized Cohere client

    Returns:
        List of embedding vectors for the queries
    """
    try:
        embeddings = embed_texts(queries, cohere_client)
        logger.info(f"Generated embeddings for {len(queries)} test queries")
        return embeddings
    except Exception as e:
        logger.error(f"Failed to generate test embeddings: {e}")
        raise


def run_similarity_search(qdrant_client: QdrantClient, collection_name: str, query_embedding: List[float],
                         top_k: int = 5, query_filter: Optional[models.Filter] = None) -> List[Dict[str, Any]]:
    """
    Execute a similarity search against the Qdrant collection.

    Args:
        qdrant_client: Initialized Qdrant client
        collection_name: Name of the collection to search in
        query_embedding: Embedding vector to search for similar items
        top_k: Number of top results to return (default: 5)
        query_filter: Optional filter to apply to the search

    Returns:
        List of result dictionaries with id, score, and payload
    """
    try:
        # Perform the search using query_points method (correct Qdrant API)
        search_results = qdrant_client.query_points(
            collection_name=collection_name,
            query=query_embedding,
            limit=top_k,
            query_filter=query_filter,
            with_payload=True
        )

        # Process and return results
        results = []
        for result in search_results:
            # Handle different result formats depending on Qdrant client version
            if hasattr(result, 'id'):  # Object with attributes
                result_dict = {
                    'id': result.id,
                    'score': getattr(result, 'score', 0),
                    'payload': getattr(result, 'payload', {})
                }
            else:  # Different format, try to handle as object
                result_dict = {
                    'id': getattr(result, 'id', None),
                    'score': getattr(result, 'score', 0),
                    'payload': getattr(result, 'payload', {})
                }

            results.append(result_dict)

        logger.info(f"Similarity search returned {len(results)} results")
        return results

    except Exception as e:
        logger.error(f"Error performing similarity search: {e}")
        raise


def run_similarity_validation(qdrant_client: QdrantClient, cohere_client: cohere.Client,
                             collection_name: str, test_queries: List[str], top_k: int = 5) -> List[SearchQuery]:
    """
    Execute similarity validation by running searches with test queries.

    Args:
        qdrant_client: Initialized Qdrant client
        cohere_client: Initialized Cohere client
        collection_name: Name of the collection to search in
        test_queries: List of test query strings
        top_k: Number of top results to return (default: 5)

    Returns:
        List of SearchQuery objects with results
    """
    search_queries = []

    for query_text in test_queries:
        try:
            # Generate embedding for the query
            query_embedding = generate_test_embeddings([query_text], cohere_client)[0]

            # Run similarity search
            results = run_similarity_search(qdrant_client, collection_name, query_embedding, top_k)

            # Create SearchQuery object with results
            search_query = SearchQuery(
                query_text=query_text,
                actual_results=results,
                expected_results=None  # For validation, we might not have expected results
            )

            search_queries.append(search_query)
            logger.info(f"Similarity search completed for query: '{query_text[:50]}...'")
        except Exception as e:
            logger.error(f"Error running similarity search for query '{query_text}': {e}")
            # Add query with error result
            search_query = SearchQuery(
                query_text=query_text,
                actual_results=[],
                expected_results=None
            )
            search_queries.append(search_query)

    return search_queries


def generate_sample_queries_from_vectors(stored_vectors: List[StoredVector], num_queries: int = 5) -> List[str]:
    """
    Generate sample test queries based on the content of stored vectors.

    Args:
        stored_vectors: List of StoredVector objects to generate queries from
        num_queries: Number of sample queries to generate (default: 5)

    Returns:
        List of sample query strings
    """
    if not stored_vectors:
        logger.warning("No stored vectors to generate sample queries from")
        return []

    sample_queries = []

    # Extract content from vectors to generate sample queries
    for i, vector in enumerate(stored_vectors[:num_queries]):
        try:
            content = vector.payload.get('content', '')
            if content:
                # Create a query based on the first 100 characters of content
                content_snippet = content[:100].strip()
                if len(content) > 100:
                    content_snippet += "..."

                query = f"What is {content_snippet}?"
                sample_queries.append(query)
        except Exception as e:
            logger.warning(f"Error generating query from vector {i}: {e}")
            continue

    # If we don't have enough queries from content, add some generic ones
    while len(sample_queries) < num_queries:
        generic_queries = [
            "What is Physical AI and Humanoid Robotics?",
            "Explain digital twin technology",
            "How does Gazebo physics simulation work?",
            "What are VLA integration methods?",
            "Describe ROS2 basics"
        ]
        # Add a different generic query each time
        query_idx = len(sample_queries) % len(generic_queries)
        sample_queries.append(generic_queries[query_idx])

    logger.info(f"Generated {len(sample_queries)} sample queries for testing")
    return sample_queries


def calculate_precision_score(actual_results: List[Dict[str, Any]], expected_results: List[str], top_k: int = 5) -> float:
    """
    Calculate precision score for search results.

    Args:
        actual_results: List of actual search results
        expected_results: List of expected result identifiers/URLs
        top_k: Number of top results to consider for precision calculation

    Returns:
        Precision score (0.0 to 1.0)
    """
    if not expected_results:
        logger.warning("No expected results provided for precision calculation")
        return 0.0

    # Get the top_k actual results
    top_results = actual_results[:top_k] if actual_results else []

    # Extract URLs or IDs from actual results for comparison
    actual_ids = set()
    for result in top_results:
        # Try to get URL from payload, or use the ID if available
        payload = result.get('payload', {})
        url = payload.get('url', '')
        if url:
            actual_ids.add(url)
        else:
            result_id = result.get('id', '')
            if result_id:
                actual_ids.add(result_id)

    # Count how many expected results are in the actual results
    relevant_retrieved = 0
    for expected in expected_results:
        if expected in actual_ids:
            relevant_retrieved += 1

    precision = relevant_retrieved / len(top_results) if top_results else 0.0
    logger.debug(f"Precision calculation: {relevant_retrieved}/{len(top_results)} = {precision:.2f}")
    return precision


def validate_chunk_integrity(stored_vector: StoredVector) -> ChunkValidation:
    """
    Validate the integrity of a stored chunk including text content, URL, and metadata.

    Args:
        stored_vector: StoredVector object to validate

    Returns:
        ChunkValidation object with validation results
    """
    chunk_id = stored_vector.id
    validation_time = datetime.now().isoformat()

    # Validate content
    content = stored_vector.payload.get('content', '')
    original_content = content  # In this context, we don't have original vs retrieved separately

    # Validate URL
    url = stored_vector.payload.get('url', '')
    url_match = bool(url) and isinstance(url, str) and url.startswith(('http://', 'https://'))

    # Calculate content similarity (in this case, we just validate that content exists)
    content_similarity = 1.0 if content and len(content.strip()) > 0 else 0.0

    chunk_validation = ChunkValidation(
        chunk_id=chunk_id,
        original_content=original_content if content_similarity > 0 else None,
        retrieved_content=content if content_similarity > 0 else None,
        url_match=url_match,
        content_similarity=content_similarity,
        validation_timestamp=validation_time
    )

    logger.debug(f"Chunk validation for ID {chunk_id}: URL match={url_match}, content similarity={content_similarity:.2f}")
    return chunk_validation


def validate_chunks_list_integrity(stored_vectors: List[StoredVector]) -> List[ChunkValidation]:
    """
    Validate the integrity of a list of stored chunks.

    Args:
        stored_vectors: List of StoredVector objects to validate

    Returns:
        List of ChunkValidation objects
    """
    chunk_validations = []

    for stored_vector in stored_vectors:
        try:
            chunk_validation = validate_chunk_integrity(stored_vector)
            chunk_validations.append(chunk_validation)
        except Exception as e:
            logger.error(f"Error validating chunk integrity for vector {stored_vector.id}: {e}")
            # Create a failed validation record
            chunk_validation = ChunkValidation(
                chunk_id=stored_vector.id,
                original_content=None,
                retrieved_content=None,
                url_match=False,
                content_similarity=0.0,
                validation_timestamp=datetime.now().isoformat()
            )
            chunk_validations.append(chunk_validation)

    logger.info(f"Completed integrity validation for {len(chunk_validations)} chunks")
    return chunk_validations


def validate_embedding_chunk_url_relationships(stored_vectors: List[StoredVector]) -> Dict[str, Any]:
    """
    Validate the relationships between embeddings, chunks, and source URLs.

    Args:
        stored_vectors: List of StoredVector objects to validate

    Returns:
        Dictionary with validation results
    """
    total_chunks = len(stored_vectors)
    valid_relationships = 0
    invalid_relationships = 0
    errors = []

    for vector in stored_vectors:
        try:
            # Check if the vector has valid embedding
            has_embedding = vector.vector and len(vector.vector) > 0

            # Check if the payload has required fields
            payload = vector.payload
            has_content = bool(payload.get('content', '').strip())
            has_url = bool(payload.get('url', '').strip())

            # Validate URL format
            url = payload.get('url', '')
            has_valid_url = bool(url) and isinstance(url, str) and url.startswith(('http://', 'https://'))

            # Check if all elements of the relationship are present
            has_complete_relationship = has_embedding and has_content and has_valid_url

            if has_complete_relationship:
                valid_relationships += 1
            else:
                invalid_relationships += 1

        except Exception as e:
            errors.append(f"Error validating relationship for vector {vector.id}: {e}")
            invalid_relationships += 1

    validation_results = {
        'total_chunks': total_chunks,
        'valid_relationships': valid_relationships,
        'invalid_relationships': invalid_relationships,
        'relationship_completeness_ratio': valid_relationships / total_chunks if total_chunks > 0 else 0,
        'errors': errors
    }

    logger.info(f"Relationship validation: {valid_relationships}/{total_chunks} valid relationships ({validation_results['relationship_completeness_ratio']:.2%})")

    return validation_results


def generate_validation_report(validation_results: List[ValidationResult],
                              sample_queries: List[SearchQuery],
                              sample_results: List[Dict[str, Any]],
                              integrity_summary: Dict[str, Any],
                              performance_metrics: Dict[str, float]) -> RetrievalReport:
    """
    Generate a comprehensive validation report.

    Args:
        validation_results: List of validation results
        sample_queries: List of sample search queries
        sample_results: List of example search results
        integrity_summary: Summary of data integrity validation
        performance_metrics: Performance metrics dictionary

    Returns:
        RetrievalReport object with comprehensive validation report
    """
    report_id = f"validation_report_{int(time.time())}"
    timestamp = datetime.now().isoformat()

    # Create recommendations based on validation results
    recommendations = []

    # Analyze validation results for recommendations
    if validation_results:
        avg_integrity = sum([vr.integrity_score for vr in validation_results]) / len(validation_results)
        if avg_integrity < 0.9:
            recommendations.append("Integrity score below threshold, investigate data corruption issues")
        elif avg_integrity < 0.95:
            recommendations.append("Consider improving data integrity validation procedures")

    # Analyze performance metrics for recommendations
    if performance_metrics:
        if performance_metrics.get('avg_processing_time', 0) > 30:  # seconds
            recommendations.append("Processing time is high, consider optimizing queries or infrastructure")
        if performance_metrics.get('throughput', 0) < 10:  # queries per minute
            recommendations.append("Throughput is low, investigate performance bottlenecks")

    # Create the report
    report = RetrievalReport(
        report_id=report_id,
        timestamp=timestamp,
        validation_results=validation_results,
        sample_queries=sample_queries,
        sample_results=sample_results,
        performance_metrics=performance_metrics,
        integrity_summary=integrity_summary,
        recommendations=recommendations
    )

    logger.info(f"Generated validation report: {report_id}")
    return report


def save_report_to_file(report: RetrievalReport, output_path: str = "validation_report.json"):
    """
    Save the validation report to a JSON file.

    Args:
        report: RetrievalReport object to save
        output_path: Path to save the report (default: "validation_report.json")
    """
    try:
        # Convert the report to a dictionary for JSON serialization
        report_dict = {
            'report_id': report.report_id,
            'timestamp': report.timestamp,
            'validation_results': [asdict(vr) for vr in report.validation_results],
            'sample_queries': [
                {
                    'query_text': sq.query_text,
                    'expected_results': sq.expected_results,
                    'actual_results': sq.actual_results,
                    'precision_score': sq.precision_score,
                    'confidence_threshold': sq.confidence_threshold
                } for sq in report.sample_queries
            ],
            'sample_results': report.sample_results,
            'performance_metrics': report.performance_metrics,
            'integrity_summary': report.integrity_summary,
            'recommendations': report.recommendations
        }

        # Write to file
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(report_dict, f, indent=2, ensure_ascii=False)

        logger.info(f"Validation report saved to: {output_path}")
    except Exception as e:
        logger.error(f"Error saving validation report to {output_path}: {e}")
        raise


async def test_rag_agent_module2():
    """
    Test the RAG agent with Module 2 content specifically.
    Query: "Explain Gazebo physics simulation in Module 2"
    """
    logger.info("Starting RAG agent test for Module 2 content...")

    # Create an instance of the RAG agent
    agent = RAGAgent(top_k=5, min_score=0.3)

    # Test query for Module 2
    query = "Explain Gazebo physics simulation in Module 2"

    logger.info(f"Query: {query}")
    print(f"Query: {query}")
    print("Processing...\n")

    try:
        # Call the invoke method
        result = await agent.invoke(query)
        print("Result:")
        print(result)
        logger.info("RAG agent test completed successfully")
        return result
    except Exception as e:
        error_msg = f"Error occurred during RAG agent test: {e}"
        logger.error(error_msg)
        print(error_msg)
        return None


def main():
    """Main entry point for the retrieval validation script"""
    parser = argparse.ArgumentParser(description='Validate retrieval functionality of stored embeddings')
    parser.add_argument('--limit', type=int, default=1000, help='Number of vectors to validate (default: 1000)')
    parser.add_argument('--test', choices=['connection', 'similarity', 'integrity', 'report', 'all'],
                       default='all', help='Specific test to run (default: all)')
    parser.add_argument('--output', type=str, default='validation_report.json',
                       help='Output file for validation report (default: validation_report.json)')
    parser.add_argument('--rag-test', action='store_true',
                       help='Run RAG agent test for Module 2 content')

    args = parser.parse_args()

    # If rag-test flag is provided, run only the RAG agent test
    if args.rag_test:
        logger.info("Running RAG agent test for Module 2 content")
        result = asyncio.run(test_rag_agent_module2())
        if result:
            print("\nRAG agent test completed successfully!")
        else:
            print("\nRAG agent test failed!")
        return

    logger.info(f"Starting retrieval validation with test: {args.test}")
    logger.info(f"Limit: {args.limit}, Output: {args.output}")

    try:
        # Load configuration
        config = load_config()

        # Initialize clients
        cohere_client = initialize_cohere_client(config)
        qdrant_client, collection_info = connect_to_qdrant_with_validation(config)

        # Initialize results containers
        validation_results = []
        sample_queries = []
        sample_results = []
        integrity_summary = {}
        performance_metrics = {}

        # Run the requested tests
        start_time = time.time()

        if args.test in ['connection', 'all']:
            logger.info("Running connection validation...")
            connection_result = run_vector_retrieval_validation(config, limit=args.limit)
            validation_results.append(connection_result)

        if args.test in ['similarity', 'all']:
            logger.info("Running similarity validation...")
            # Fetch a sample of vectors to generate test queries from
            sample_vectors = fetch_all_vectors(qdrant_client, config['collection_name'], limit=5)
            test_queries = generate_sample_queries_from_vectors(sample_vectors, num_queries=3)
            similarity_results = run_similarity_validation(qdrant_client, cohere_client, config['collection_name'], test_queries)
            sample_queries.extend(similarity_results)

        if args.test in ['integrity', 'all']:
            logger.info("Running integrity validation...")
            # Fetch vectors to validate integrity
            vectors_for_integrity = fetch_all_vectors(qdrant_client, config['collection_name'], limit=args.limit)
            chunk_validations = validate_chunks_list_integrity(vectors_for_integrity)
            relationship_validation = validate_embedding_chunk_url_relationships(vectors_for_integrity)

            # Add to sample results
            sample_results.append({
                'chunk_validations': len([cv for cv in chunk_validations if cv.url_match]),
                'total_chunks': len(chunk_validations),
                'relationship_validation': relationship_validation
            })

        if args.test in ['report', 'all']:
            logger.info("Generating comprehensive validation report...")

            # Collect integrity summary
            integrity_summary = {
                'total_validated': args.limit,
                'validation_passed': len([vr for vr in validation_results if vr.integrity_score >= 0.9]),
                'validation_failed': len([vr for vr in validation_results if vr.integrity_score < 0.9])
            }

            # Collect performance metrics
            total_time = time.time() - start_time
            performance_metrics = {
                'total_execution_time': total_time,
                'vectors_per_second': args.limit / total_time if total_time > 0 else 0,
                'avg_processing_time': total_time / args.limit if args.limit > 0 else 0
            }

            # Generate and save the report
            report = generate_validation_report(
                validation_results=validation_results,
                sample_queries=sample_queries,
                sample_results=sample_results,
                integrity_summary=integrity_summary,
                performance_metrics=performance_metrics
            )

            save_report_to_file(report, args.output)
            logger.info(f"Validation report saved to {args.output}")

        logger.info("Retrieval validation completed successfully")

    except Exception as e:
        logger.error(f"Error during retrieval validation: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()