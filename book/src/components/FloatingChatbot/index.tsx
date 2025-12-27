import React, { useState, useEffect, useRef } from 'react';
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

type Message = {
  id: string;
  content: string;
  role: 'user' | 'assistant';
  citations?: CitationReference[];
};

const FloatingChatbot: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<{ message: string; originalError?: any } | null>(null);
  const [sessionId, setSessionId] = useState<string | null>(null);
  const [showSuggestions, setShowSuggestions] = useState(true);
  const messagesEndRef = useRef<null | HTMLDivElement>(null);

  const suggestedQuestions = [
    "What is Physical AI?",
    "What is this book about?",
    "Explain Module 1",
    "Explain Module 2"
  ];

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSendMessage = async (message: string) => {
    if (!message.trim()) return;

    // Add user message to chat
    const userMessage: Message = {
      id: Date.now().toString(),
      content: message,
      role: 'user'
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setShowSuggestions(false);
    setLoading(true);
    setError(null);

    try {
      const result = await queryRAG(message, null, sessionId || null);

      const assistantMessage: Message = {
        id: result.id,
        content: result.content,
        role: 'assistant',
        citations: result.citations
      };

      setMessages(prev => [...prev, assistantMessage]);
      setSessionId(result.sessionId || result.queryId);
    } catch (err: any) {
      setError({
        message: err.message || 'An error occurred while processing your query',
        originalError: err
      });
    } finally {
      setLoading(false);
    }
  };

  const handleRetry = () => {
    setError(null);
    if (messages.length > 0) {
      const lastUserMessage = [...messages].reverse().find(msg => msg.role === 'user');
      if (lastUserMessage) {
        handleSendMessage(lastUserMessage.content);
      }
    }
  };

  const handleSuggestionClick = (question: string) => {
    handleSendMessage(question);
  };

  return (
    <>
      {/* Floating chat icon */}
      <button
        className="floating-chat-icon"
        onClick={() => setIsOpen(!isOpen)}
        style={{
          position: 'fixed',
          bottom: '20px',
          right: '20px',
          width: '60px',
          height: '60px',
          borderRadius: '50%',
          backgroundColor: '#007cba',
          color: 'white',
          border: 'none',
          cursor: 'pointer',
          fontSize: '24px',
          zIndex: 1000,
          boxShadow: '0 4px 12px rgba(0, 0, 0, 0.15)',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          transition: 'all 0.3s ease'
        }}
      >
        💬
      </button>

      {/* Chat panel */}
      {isOpen && (
        <div
          className="floating-chat-panel"
          style={{
            position: 'fixed',
            bottom: '90px',
            right: '20px',
            width: '400px',
            height: '500px',
            backgroundColor: 'white',
            borderRadius: '12px',
            boxShadow: '0 8px 30px rgba(0, 0, 0, 0.2)',
            zIndex: 1000,
            display: 'flex',
            flexDirection: 'column',
            overflow: 'hidden',
            fontFamily: 'system-ui, -apple-system, sans-serif'
          }}
        >
          {/* Header */}
          <div
            style={{
              backgroundColor: '#007cba',
              color: 'white',
              padding: '16px',
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center'
            }}
          >
            <h3 style={{ margin: 0, fontSize: '16px' }}>Ask about the Textbook</h3>
            <button
              onClick={() => setIsOpen(false)}
              style={{
                background: 'none',
                border: 'none',
                color: 'white',
                fontSize: '20px',
                cursor: 'pointer',
                padding: '0',
                width: '24px',
                height: '24px',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center'
              }}
            >
              ×
            </button>
          </div>

          {/* Messages container */}
          <div
            style={{
              flex: 1,
              overflowY: 'auto',
              padding: '16px',
              backgroundColor: '#f9f9f9',
              display: 'flex',
              flexDirection: 'column'
            }}
          >
            {messages.length === 0 && showSuggestions && (
              <div style={{ marginBottom: '16px' }}>
                <p style={{ color: '#666', fontSize: '14px', marginBottom: '12px' }}>Try asking:</p>
                <div style={{ display: 'flex', flexDirection: 'column', gap: '8px' }}>
                  {suggestedQuestions.map((question, index) => (
                    <button
                      key={index}
                      onClick={() => handleSuggestionClick(question)}
                      style={{
                        textAlign: 'left',
                        padding: '10px 12px',
                        border: '1px solid #ddd',
                        borderRadius: '6px',
                        backgroundColor: 'white',
                        cursor: 'pointer',
                        fontSize: '14px',
                        color: '#333',
                        transition: 'background-color 0.2s'
                      }}
                      onMouseOver={(e) => {
                        (e.target as HTMLButtonElement).style.backgroundColor = '#f0f0f0';
                      }}
                      onMouseOut={(e) => {
                        (e.target as HTMLButtonElement).style.backgroundColor = 'white';
                      }}
                    >
                      {question}
                    </button>
                  ))}
                </div>
              </div>
            )}

            {messages.map((message) => (
              <div
                key={message.id}
                style={{
                  marginBottom: '16px',
                  display: 'flex',
                  justifyContent: message.role === 'user' ? 'flex-end' : 'flex-start'
                }}
              >
                <div
                  style={{
                    maxWidth: '85%',
                    padding: '12px 16px',
                    borderRadius: '18px',
                    backgroundColor: message.role === 'user' ? '#007cba' : '#e9ecef',
                    color: message.role === 'user' ? 'white' : '#333',
                    fontSize: '14px',
                    lineHeight: '1.4'
                  }}
                >
                  {message.content}
                  {message.role === 'assistant' && message.citations && message.citations.length > 0 && (
                    <div style={{ marginTop: '8px' }}>
                      <CitationList citations={message.citations} />
                    </div>
                  )}
                </div>
              </div>
            ))}

            {loading && (
              <div style={{ marginBottom: '16px', display: 'flex', justifyContent: 'flex-start' }}>
                <div
                  style={{
                    maxWidth: '85%',
                    padding: '12px 16px',
                    borderRadius: '18px',
                    backgroundColor: '#e9ecef',
                    color: '#333',
                    fontSize: '14px'
                  }}
                >
                  Processing your query...
                </div>
              </div>
            )}

            {error && (
              <div style={{ marginBottom: '16px', display: 'flex', justifyContent: 'flex-start' }}>
                <div
                  style={{
                    maxWidth: '85%',
                    padding: '12px 16px',
                    borderRadius: '18px',
                    backgroundColor: '#f8d7da',
                    color: '#721c24',
                    fontSize: '14px'
                  }}
                >
                  <ErrorDisplay error={error} onRetry={handleRetry} />
                </div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Input area */}
          <div
            style={{
              padding: '12px',
              backgroundColor: 'white',
              borderTop: '1px solid #eee',
              display: 'flex',
              gap: '8px'
            }}
          >
            <input
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={(e) => {
                if (e.key === 'Enter' && !e.shiftKey) {
                  e.preventDefault();
                  handleSendMessage(inputValue);
                }
              }}
              placeholder="Type your question..."
              disabled={loading}
              style={{
                flex: 1,
                padding: '10px 12px',
                border: '1px solid #ddd',
                borderRadius: '20px',
                fontSize: '14px',
                outline: 'none'
              }}
            />
            <button
              onClick={() => handleSendMessage(inputValue)}
              disabled={loading || !inputValue.trim()}
              style={{
                padding: '10px 16px',
                backgroundColor: loading || !inputValue.trim() ? '#ccc' : '#007cba',
                color: 'white',
                border: 'none',
                borderRadius: '20px',
                cursor: loading || !inputValue.trim() ? 'not-allowed' : 'pointer',
                fontSize: '14px'
              }}
            >
              Send
            </button>
          </div>
        </div>
      )}
    </>
  );
};

export default FloatingChatbot;