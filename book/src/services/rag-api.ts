/**
 * API service for RAG agent integration in Docusaurus
 */

// Base API configuration
// For browser-safe API URL configuration in Docusaurus
// Using a default that works for development
// For production deployment, this can be overridden by setting window.RAG_API_URL

// Define types for window object extension
declare global {
  interface Window {
    RAG_API_URL?: string;
  }
}

// Get API base URL from window object (set by client module) or default
const API_BASE_URL = typeof window !== 'undefined' && window.RAG_API_URL
  ? window.RAG_API_URL
  : 'https://faizananjum-rag-chatbot.hf.space';

// Define types
export type CitationReference = {
  id: string;
  text: string;
  sourceUrl: string;
  pageReference: string;
  context: string;
};

export type QueryResponse = {
  id: string;
  queryId: string;
  content: string;
  citations: CitationReference[];
  confidence: number;
  timestamp: string;
  sessionId?: string;
};

export type ErrorResponse = {
  error: string;
  message: string;
  timestamp: string;
};

/**
 * Query the RAG agent with a user question
 * @param {string} query - The user's question
 * @param {string | null} userId - Optional user identifier
 * @param {string | null} sessionId - Optional session identifier
 * @param {number} timeout - Request timeout in milliseconds (default: 30000ms)
 * @returns {Promise<QueryResponse>} The response from the RAG agent
 */
export const queryRAG = async (
  query: string,
  userId: string | null = null,
  sessionId: string | null = null,
  timeout: number = 30000
): Promise<QueryResponse> => {
  try {
    const requestBody = {
      query: query,  // Backend expects 'query', not 'text'
      userId: userId || null,
      sessionId: sessionId || null
    };

    // Create a timeout promise
    const timeoutPromise = new Promise<never>((_, reject) => {
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
      const errorData: ErrorResponse = await response.json();
      throw new Error(errorData.message || `HTTP error! status: ${response.status}`);
    }

    const backendResponse = await response.json();

    // Convert backend response format to frontend expected format
    const frontendResponse: QueryResponse = {
      id: backendResponse.id || 'response_' + Date.now() + '_' + Math.random().toString(36).substr(2, 9),
      queryId: backendResponse.queryId || 'query_' + Date.now() + '_' + Math.random().toString(36).substr(2, 9),
      content: backendResponse.content,
      citations: (backendResponse.citations || []).map((citation: any) => ({
        id: citation.id || 'cit_' + Math.random().toString(36).substr(2, 9),
        text: citation.text,
        sourceUrl: citation.sourceUrl,
        pageReference: citation.pageReference,
        context: citation.context || 'Relevant textbook content used to answer the query'
      })),
      confidence: backendResponse.confidence || 0.85, // Use backend confidence if available
      timestamp: backendResponse.timestamp || new Date().toISOString(),
      sessionId: sessionId || backendResponse.sessionId || generateSessionId()
    };

    return frontendResponse;
  } catch (error: any) {
    console.error('Error querying RAG agent:', error);
    throw error;
  }
};

/**
 * Health check for the RAG API
 * @returns {Promise<{status: string, service: string}>} Health status
 */
export const checkHealth = async (): Promise<{status: string, service: string}> => {
  try {
    const response = await fetch(`${API_BASE_URL}/health`);

    if (!response.ok) {
      throw new Error(`Health check failed with status: ${response.status}`);
    }

    const result = await response.json();
    return result;
  } catch (error: any) {
    console.error('Error checking RAG API health:', error);
    throw error;
  }
};

/**
 * Generate a session ID if one is not provided
 * @returns {string} A unique session identifier
 */
const generateSessionId = (): string => {
  return 'session_' + Date.now() + '_' + Math.random().toString(36).substr(2, 9);
};

export default {
  queryRAG,
  checkHealth
};