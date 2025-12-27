"""
Test script to debug the 422 error issue
"""
import requests
import json

# Test the exact format that the frontend sends
BASE_URL = "http://127.0.0.1:8000"

def test_frontend_format():
    """Test the format that the actual frontend sends"""
    print("Testing frontend format (text, max_chunks, similarity_threshold)...")

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
        print(f"Response: {response.text[:200]}...")

        if response.status_code == 200:
            print("SUCCESS: Frontend format works correctly")
            return True
        else:
            print(f"ERROR: {response.status_code} - {response.text}")
            # Try to parse the error details
            try:
                error_detail = response.json()
                print(f"Error details: {error_detail}")
            except:
                pass
            return False
    except Exception as e:
        print(f"ERROR: {e}")
        return False

def test_with_validation_errors():
    """Test various validation edge cases"""
    print("\nTesting validation edge cases...")

    # Test 1: Empty text
    payload1 = {
        "text": "",
        "max_chunks": 5,
        "similarity_threshold": 0.5
    }

    try:
        response = requests.post(f"{BASE_URL}/api/rag/query",
                                json=payload1,
                                headers={"Content-Type": "application/json"})
        print(f"Empty text - Status: {response.status_code} (expected 422)")
    except Exception as e:
        print(f"Empty text test error: {e}")

    # Test 2: Text too long
    payload2 = {
        "text": "a" * 1001,  # Exceeds max length of 1000
        "max_chunks": 5,
        "similarity_threshold": 0.5
    }

    try:
        response = requests.post(f"{BASE_URL}/api/rag/query",
                                json=payload2,
                                headers={"Content-Type": "application/json"})
        print(f"Long text - Status: {response.status_code} (expected 422)")
    except Exception as e:
        print(f"Long text test error: {e}")

    # Test 3: Invalid max_chunks
    payload3 = {
        "text": "Test query",
        "max_chunks": 0,  # Below minimum of 1
        "similarity_threshold": 0.5
    }

    try:
        response = requests.post(f"{BASE_URL}/api/rag/query",
                                json=payload3,
                                headers={"Content-Type": "application/json"})
        print(f"Invalid max_chunks - Status: {response.status_code} (expected 422)")
    except Exception as e:
        print(f"Invalid max_chunks test error: {e}")

    # Test 4: Invalid similarity_threshold
    payload4 = {
        "text": "Test query",
        "max_chunks": 5,
        "similarity_threshold": -0.1  # Below minimum of 0.0
    }

    try:
        response = requests.post(f"{BASE_URL}/api/rag/query",
                                json=payload4,
                                headers={"Content-Type": "application/json"})
        print(f"Invalid similarity_threshold - Status: {response.status_code} (expected 422)")
    except Exception as e:
        print(f"Invalid similarity_threshold test error: {e}")

if __name__ == "__main__":
    print("Testing API to debug 422 errors...")

    # First, check if server is running
    try:
        health_response = requests.get(f"{BASE_URL}/health")
        print(f"Backend health check: {health_response.status_code}")
        if health_response.status_code != 200:
            print(f"ERROR: Backend is not running or not accessible at {BASE_URL}")
            exit(1)
    except Exception as e:
        print(f"ERROR: Cannot connect to backend at {BASE_URL}. Make sure the server is running.")
        print(f"Error: {e}")
        exit(1)

    # Test the frontend format
    success = test_frontend_format()

    # Test validation edge cases
    test_with_validation_errors()

    if success:
        print("\n✅ The API is working correctly with the frontend format!")
        print("If you're still getting 422 errors, the issue might be:")
        print("- Network connectivity issues")
        print("- Incorrect API URL in the frontend")
        print("- CORS issues (less likely since it's a 422, not 403/405)")
        print("- Request timeout or other network issues")
    else:
        print("\n❌ The API is not working with the frontend format.")
        print("There may be validation or configuration issues.")