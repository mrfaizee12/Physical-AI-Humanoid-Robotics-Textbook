import React from 'react';

const ErrorDisplay = ({ error, onRetry = null }) => {
  if (!error) {
    return null;
  }

  const handleRetry = () => {
    if (onRetry) {
      onRetry();
    }
  };

  // Determine error type and message
  let displayMessage = error.message || 'An unknown error occurred';
  let errorType = 'error';

  // Check for specific error types
  if (error.message && error.message.toLowerCase().includes('query')) {
    displayMessage = 'Your query appears to be invalid. Please make sure it is at least 3 characters long.';
    errorType = 'query-error';
  } else if (error.message && (error.message.toLowerCase().includes('timeout') || error.message.includes('408'))) {
    displayMessage = 'The request timed out. Please try again.';
    errorType = 'timeout-error';
  } else if (error.message && (error.message.toLowerCase().includes('service') || error.message.toLowerCase().includes('503'))) {
    displayMessage = 'The RAG service is temporarily unavailable. Please try again later.';
    errorType = 'service-error';
  }

  return (
    <div className={`error-display error-${errorType}`}>
      <div className="error-content">
        <h4>Error</h4>
        <p>{displayMessage}</p>
        {onRetry && (
          <button onClick={handleRetry} className="retry-button">
            Try Again
          </button>
        )}
      </div>
    </div>
  );
};

export default ErrorDisplay;