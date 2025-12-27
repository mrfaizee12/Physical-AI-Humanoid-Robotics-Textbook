/**
 * API service for RAG agent integration
 */

// Base API configuration
const API_BASE_URL = process.env.REACT_APP_RAG_API_URL || 'http://localhost:8000';

/**
 * Query the RAG agent with a user question
 * @param {string} query - The user's question
 * @param {string} userId - Optional user identifier
 * @param {string} sessionId - Optional session identifier
 * @param {number} timeout - Request timeout in milliseconds (default: 30000ms)
 * @returns {Promise<Object>} The response from the RAG agent
 */
export const queryRAG = async (query, userId = null, sessionId = null, timeout = 30000) => {
  try {
    const requestBody = {
      query: query,
      userId: userId,
      sessionId: sessionId || generateSessionId()
    };

    // Create a timeout promise
    const timeoutPromise = new Promise((_, reject) => {
      setTimeout(() => {
        reject(new Error('Request timed out'));
      }, timeout);
    });

    // Create the fetch promise
    const fetchPromise = fetch(`${API_BASE_URL}/api/rag/query`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(requestBody),
    });

    // Race between fetch and timeout
    const response = await Promise.race([fetchPromise, timeoutPromise]);

    if (!response.ok) {
      const errorData = await response.json();
      throw new Error(errorData.message || `HTTP error! status: ${response.status}`);
    }

    return await response.json();
  } catch (error) {
    console.error('Error querying RAG agent:', error);
    throw error;
  }
};

/**
 * Health check for the RAG API
 * @returns {Promise<Object>} Health status
 */
export const checkHealth = async () => {
  try {
    const response = await fetch(`${API_BASE_URL}/health`);

    if (!response.ok) {
      throw new Error(`Health check failed with status: ${response.status}`);
    }

    return await response.json();
  } catch (error) {
    console.error('Error checking RAG API health:', error);
    throw error;
  }
};

/**
 * Generate a session ID if one is not provided
 * @returns {string} A unique session identifier
 */
const generateSessionId = () => {
  return 'session_' + Date.now() + '_' + Math.random().toString(36).substr(2, 9);
};

export default {
  queryRAG,
  checkHealth
};