#!/usr/bin/env python3
"""
Script to check if chunk indices are now globally sequential in Qdrant
"""
import sys
import os

# Add the backend directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

from src.config import settings
from qdrant_client import QdrantClient
from qdrant_client.http import models

def check_chunk_indices():
    """Check if chunk indices are globally sequential in Qdrant"""
    print("Connecting to Qdrant...")

    # Initialize Qdrant client
    client = QdrantClient(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key,
        prefer_grpc=False
    )

    collection_name = settings.qdrant_collection_name
    print(f"Connected to collection: {collection_name}")

    # Fetch all points from the collection
    points = client.scroll(
        collection_name=collection_name,
        limit=1000  # Get up to 1000 points
    )

    all_points = []
    points_data, next_offset = points

    # Get all points (scroll through all of them)
    all_points.extend(points_data)

    while next_offset is not None:
        points = client.scroll(
            collection_name=collection_name,
            limit=1000,
            offset=next_offset
        )
        points_data, next_offset = points
        all_points.extend(points_data)
        if len(points_data) == 0:
            break

    print(f"Fetched {len(all_points)} points from Qdrant")

    # Extract chunk indices from payloads
    chunk_indices = []
    for point in all_points:
        chunk_index = point.payload.get('chunk_index', 0)
        chunk_indices.append(chunk_index)
        print(f"Point ID: {point.id}, Chunk Index: {chunk_index}, URL: {point.payload.get('url', 'N/A')[:50]}...")

    # Sort the chunk indices to check if they're sequential
    sorted_indices = sorted(chunk_indices)
    print(f"\nChunk indices (sorted): {sorted_indices[:20]}{'...' if len(sorted_indices) > 20 else ''}")

    # Check if they form a sequential sequence starting from 0
    expected_sequence = list(range(len(chunk_indices)))
    is_sequential = sorted_indices == expected_sequence

    print(f"\nTotal chunks: {len(chunk_indices)}")
    print(f"Expected sequence [0 to {len(chunk_indices)-1}]: {is_sequential}")

    if is_sequential:
        print("SUCCESS: Chunk indices are globally sequential (0, 1, 2, 3, ...)!")
    else:
        print("ISSUE: Chunk indices are NOT globally sequential")
        # Show first few non-sequential values
        unique_indices = sorted(list(set(chunk_indices)))
        print(f"Unique chunk indices (first 20): {unique_indices[:20]}")

        # Check for duplicates
        if len(chunk_indices) != len(set(chunk_indices)):
            print("WARNING: There are duplicate chunk indices!")
            from collections import Counter
            index_counts = Counter(chunk_indices)
            duplicates = {k: v for k, v in index_counts.items() if v > 1}
            if duplicates:
                print(f"Duplicate indices: {duplicates}")

    return is_sequential

if __name__ == "__main__":
    try:
        success = check_chunk_indices()
        if success:
            print("\nRAG retrieval fix verification: PASSED")
            exit(0)
        else:
            print("\nRAG retrieval fix verification: FAILED")
            exit(1)
    except Exception as e:
        print(f"Error during verification: {e}")
        import traceback
        traceback.print_exc()
        exit(1)