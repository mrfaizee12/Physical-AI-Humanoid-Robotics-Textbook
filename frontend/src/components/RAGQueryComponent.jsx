import React, { useState } from 'react';
import { queryRAG } from '../services/rag-api';
import CitationList from './CitationList';
import ErrorDisplay from './ErrorDisplay';

const RAGQueryComponent = () => {
  const [query, setQuery] = useState('');
  const [response, setResponse] = useState(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);
  const [sessionId, setSessionId] = useState(null);

  const handleSubmit = async (e) => {
    e.preventDefault();

    if (!query.trim()) {
      setError({ message: 'Query cannot be empty' });
      return;
    }

    setLoading(true);
    setError(null);

    try {
      const result = await queryRAG(query, null, sessionId);
      setResponse(result);
      setSessionId(result.sessionId || result.queryId);
    } catch (err) {
      setError({
        message: err.message || 'An error occurred while processing your query',
        originalError: err
      });
      setResponse(null);
    } finally {
      setLoading(false);
    }
  };

  const handleRetry = () => {
    setError(null);
    handleSubmit({ preventDefault: () => {} });
  };

  return (
    <div className="rag-query-component">
      <h2>Ask about the Textbook</h2>
      <form onSubmit={handleSubmit} className="query-form">
        <div className="query-input-container">
          <input
            type="text"
            value={query}
            onChange={(e) => setQuery(e.target.value)}
            placeholder="Enter your question about the textbook content..."
            className="query-input"
            disabled={loading}
          />
          <button type="submit" disabled={loading} className="query-submit-btn">
            {loading ? 'Processing...' : 'Ask'}
          </button>
        </div>
      </form>

      {loading && (
        <div className="loading-indicator">
          <p>Processing your query...</p>
        </div>
      )}

      {error && (
        <ErrorDisplay error={error} onRetry={handleRetry} />
      )}

      {response && !loading && (
        <div className="response-container">
          <h3>Response:</h3>
          <div className="response-content">
            <p>{response.content}</p>
          </div>

          <CitationList citations={response.citations} />
        </div>
      )}
    </div>
  );
};

export default RAGQueryComponent;