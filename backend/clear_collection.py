#!/usr/bin/env python3
"""
Script to clear the Qdrant collection and verify it's empty
"""
import sys
import os

# Add the backend directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

from src.config import settings
from qdrant_client import QdrantClient
from qdrant_client.http import models

def clear_collection():
    """Clear the entire Qdrant collection"""
    print("Connecting to Qdrant...")

    # Initialize Qdrant client
    client = QdrantClient(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key,
        prefer_grpc=False
    )

    collection_name = settings.qdrant_collection_name
    print(f"Connected to collection: {collection_name}")

    # Get current count
    collection_info = client.get_collection(collection_name)
    current_count = collection_info.points_count
    print(f"Current points in collection: {current_count}")

    if current_count > 0:
        print(f"Deleting all {current_count} points from collection...")

        # Delete all points
        client.delete(
            collection_name=collection_name,
            points_selector=models.FilterSelector(
                filter=models.Filter(
                    must=[]
                )
            )
        )

        # Verify deletion
        collection_info = client.get_collection(collection_name)
        new_count = collection_info.points_count
        print(f"Points after deletion: {new_count}")

        if new_count == 0:
            print("SUCCESS: Collection cleared successfully!")
            return True
        else:
            print(f"ERROR: Expected 0 points, but found {new_count}")
            return False
    else:
        print("Collection is already empty.")
        return True

if __name__ == "__main__":
    try:
        success = clear_collection()
        if success:
            print("Collection clear operation: PASSED")
            exit(0)
        else:
            print("Collection clear operation: FAILED")
            exit(1)
    except Exception as e:
        print(f"Error during collection clear: {e}")
        import traceback
        traceback.print_exc()
        exit(1)