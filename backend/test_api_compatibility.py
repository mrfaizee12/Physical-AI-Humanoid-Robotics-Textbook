"""
Test script to verify API compatibility between frontend and backend
"""
import requests
import json

# Test the current API endpoints with different payload formats

BASE_URL = "http://127.0.0.1:8000"

def test_new_format():
    """Test the new format that the frontend actually sends"""
    print("Testing new format (text, max_chunks, similarity_threshold)...")

    payload = {
        "text": "What is the main concept of Module 1?",
        "max_chunks": 5,
        "similarity_threshold": 0.25
    }

    try:
        response = requests.post(f"{BASE_URL}/api/rag/query",
                                json=payload,
                                headers={"Content-Type": "application/json"})
        print(f"Status: {response.status_code}")
        if response.status_code == 200:
            print("SUCCESS: New format works correctly")
            print(f"Response preview: {response.json().get('answer', '')[:100]}...")
        else:
            print(f"ERROR: {response.status_code} - {response.text}")
        return response.status_code == 200
    except Exception as e:
        print(f"ERROR: {e}")
        return False

def test_legacy_format():
    """Test the legacy format you mentioned"""
    print("\nTesting legacy format (query, book_id, user_id)...")

    payload = {
        "query": "What is the main concept of Module 1?",
        "book_id": "some-book-uuid",
        "user_id": "some-user-uuid"
    }

    try:
        response = requests.post(f"{BASE_URL}/api/query",
                                json=payload,
                                headers={"Content-Type": "application/json"})
        print(f"Status: {response.status_code}")
        if response.status_code == 200:
            print("SUCCESS: Legacy format works correctly")
            print(f"Response preview: {response.json().get('answer', '')[:100]}...")
        else:
            print(f"ERROR: {response.status_code} - {response.text}")
        return response.status_code == 200
    except Exception as e:
        print(f"ERROR: {e}")
        return False

def test_legacy_format_old_endpoint():
    """Test the legacy format on the old endpoint"""
    print("\nTesting legacy format on old endpoint (should fail)...")

    payload = {
        "query": "What is the main concept of Module 1?",
        "book_id": "some-book-uuid",
        "user_id": "some-user-uuid"
    }

    try:
        response = requests.post(f"{BASE_URL}/api/rag/query",
                                json=payload,
                                headers={"Content-Type": "application/json"})
        print(f"Status: {response.status_code}")
        if response.status_code == 422:
            print("EXPECTED: Legacy format fails on new endpoint (422 validation error)")
        else:
            print(f"UNEXPECTED: Got {response.status_code} instead of 422")
        return response.status_code == 422
    except Exception as e:
        print(f"ERROR: {e}")
        return False

if __name__ == "__main__":
    print("Testing API compatibility...")

    # Make sure the server is running first
    try:
        health_response = requests.get(f"{BASE_URL}/health")
        print(f"Backend health check: {health_response.status_code}")
    except:
        print(f"ERROR: Cannot connect to backend at {BASE_URL}. Make sure the server is running.")
        exit(1)

    results = []
    results.append(test_new_format())
    results.append(test_legacy_format())
    results.append(test_legacy_format_old_endpoint())

    print(f"\nResults: {sum(results)}/{len(results)} tests behaved as expected")

    if results[0]:  # New format should work
        print("✅ Frontend format is compatible with backend")
    else:
        print("❌ Frontend format is NOT compatible with backend")

    if results[1]:  # Legacy format should work on new endpoint
        print("✅ Legacy format is now supported")
    else:
        print("❌ Legacy format is NOT supported (may need to start server)")

    if results[2]:  # Legacy on old endpoint should fail
        print("✅ Correctly rejects invalid format")
    else:
        print("❌ Should have rejected invalid format")