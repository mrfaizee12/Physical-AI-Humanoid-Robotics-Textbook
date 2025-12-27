import React, { useState } from 'react';
import { queryRAG } from '@site/src/services/rag-api';
import CitationList from '@site/src/components/CitationList';
import ErrorDisplay from '@site/src/components/ErrorDisplay';

type CitationReference = {
  id: string;
  text: string;
  sourceUrl: string;
  pageReference: string;
  context: string;
};

type QueryResponse = {
  id: string;
  queryId: string;
  content: string;
  citations: CitationReference[];
  confidence: number;
  timestamp: string;
  sessionId?: string;
};

const RAGQueryComponent: React.FC = () => {
  const [query, setQuery] = useState('');
  const [response, setResponse] = useState<QueryResponse | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<{ message: string; originalError?: any } | null>(null);
  const [sessionId, setSessionId] = useState<string | null>(null);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!query.trim()) {
      setError({ message: 'Query cannot be empty' });
      return;
    }

    setLoading(true);
    setError(null);

    try {
      const result = await queryRAG(query, null, sessionId || null);
      setResponse(result);
      setSessionId(result.sessionId || result.queryId);
    } catch (err: any) {
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
    handleSubmit({ preventDefault: () => {}, currentTarget: undefined } as React.FormEvent);
  };

  return (
    <div className="rag-query-component" style={{
      maxWidth: '800px',
      margin: '2rem auto',
      padding: '0 1rem',
      border: '1px solid #ddd',
      borderRadius: '8px',
      backgroundColor: '#fff'
    }}>
      <div style={{ padding: '1.5rem' }}>
        <h2 style={{ marginTop: 0, marginBottom: '1.5rem' }}>Ask about the Textbook</h2>
        <form onSubmit={handleSubmit} className="query-form" style={{ marginBottom: '1.5rem' }}>
          <div className="query-input-container" style={{ display: 'flex', gap: '0.5rem', marginBottom: '1rem' }}>
            <input
              type="text"
              value={query}
              onChange={(e) => setQuery(e.target.value)}
              placeholder="Enter your question about the textbook content..."
              className="query-input"
              disabled={loading}
              style={{
                flex: 1,
                padding: '0.75rem',
                border: '1px solid #ccc',
                borderRadius: '4px',
                fontSize: '1rem'
              }}
            />
            <button
              type="submit"
              disabled={loading}
              className="query-submit-btn"
              style={{
                padding: '0.75rem 1.5rem',
                backgroundColor: loading ? '#ccc' : '#007cba',
                color: 'white',
                border: 'none',
                borderRadius: '4px',
                cursor: loading ? 'not-allowed' : 'pointer',
                fontSize: '1rem'
              }}
            >
              {loading ? 'Processing...' : 'Ask'}
            </button>
          </div>
        </form>

        {loading && (
          <div className="loading-indicator" style={{ marginBottom: '1.5rem' }}>
            <p>Processing your query...</p>
          </div>
        )}

        {error && (
          <ErrorDisplay error={error} onRetry={handleRetry} />
        )}

        {response && !loading && (
          <div className="response-container">
            <h3>Response:</h3>
            <div className="response-content" style={{ marginBottom: '1.5rem' }}>
              <p>{response.content}</p>
            </div>
            <CitationList citations={response.citations} />
          </div>
        )}
      </div>
    </div>
  );
};

export default RAGQueryComponent;