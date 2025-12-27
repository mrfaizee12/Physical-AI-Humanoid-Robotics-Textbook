import React from 'react';

type CitationReference = {
  id: string;
  text: string;
  sourceUrl: string;
  pageReference: string;
  context: string;
};

const CitationList: React.FC<{ citations: CitationReference[] }> = ({ citations = [] }) => {
  if (!citations || citations.length === 0) {
    return (
      <div className="no-citations" style={{ padding: '1rem', backgroundColor: '#f9f9f9', borderRadius: '4px' }}>
        <p>No citations available for this response.</p>
      </div>
    );
  }

  return (
    <div className="citations-container" style={{ marginTop: '1.5rem' }}>
      <h4 className="citations-title" style={{ marginBottom: '1rem' }}>Citations:</h4>
      <ul className="citations-list" style={{ listStyle: 'none', padding: 0 }}>
        {citations.map((citation, index) => (
          <li
            key={citation.id || `cit-${index}`}
            className="citation-item"
            style={{
              marginBottom: '1rem',
              padding: '1rem',
              border: '1px solid #eee',
              borderRadius: '4px'
            }}
          >
            <div className="citation-content">
              <p className="citation-text" style={{ fontStyle: 'italic', marginBottom: '0.5rem' }}>
                "{citation.text}"
              </p>
              <div className="citation-details">
                <p className="citation-source" style={{ marginBottom: '0.25rem' }}>
                  <strong>Source:</strong> {citation.sourceUrl}
                </p>
                <p className="citation-page" style={{ marginBottom: '0.25rem' }}>
                  <strong>Reference:</strong> {citation.pageReference}
                </p>
                {citation.context && (
                  <p className="citation-context" style={{ marginBottom: 0 }}>
                    <strong>Context:</strong> {citation.context}
                  </p>
                )}
              </div>
            </div>
          </li>
        ))}
      </ul>
    </div>
  );
};

export default CitationList;