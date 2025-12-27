import React from 'react';

const CitationList = ({ citations = [] }) => {
  if (!citations || citations.length === 0) {
    return (
      <div className="no-citations">
        <p>No citations available for this response.</p>
      </div>
    );
  }

  return (
    <div className="citations-container">
      <h4 className="citations-title">Citations:</h4>
      <ul className="citations-list">
        {citations.map((citation, index) => (
          <li key={citation.id || `cit-${index}`} className="citation-item">
            <div className="citation-content">
              <p className="citation-text">
                <em>"{citation.text}"</em>
              </p>
              <div className="citation-details">
                <p className="citation-source">
                  <strong>Source:</strong> {citation.sourceUrl}
                </p>
                <p className="citation-page">
                  <strong>Reference:</strong> {citation.pageReference}
                </p>
                {citation.context && (
                  <p className="citation-context">
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