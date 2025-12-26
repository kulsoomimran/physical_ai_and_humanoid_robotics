import React, { useState, useEffect, useRef } from 'react';
import useChatAPI from './hooks/useChatAPI';
import ChatMessage from './ChatMessage';

const Chatbot = () => {
  const [messages, setMessages] = useState(() => {
    if (typeof window !== 'undefined') {
      const saved = localStorage.getItem('chatbotMessages');
      return saved ? JSON.parse(saved) : [];
    }
    return [];
  });

  const [currentQuestion, setCurrentQuestion] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  const { sendQuestion } = useChatAPI();

  // Persist messages
  useEffect(() => {
    if (typeof window !== 'undefined' && messages.length > 0) {
      localStorage.setItem('chatbotMessages', JSON.stringify(messages));
    }
  }, [messages]);

  // Auto-scroll to bottom
  useEffect(() => {
    scrollToBottom();
  }, [messages, isLoading]);

  // Auto-focus input on mount
  useEffect(() => {
    inputRef.current?.focus();
  }, []);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  // Get selected text from page
  const getSelectedText = () => {
    if (typeof window !== 'undefined' && window.getSelection) {
      return window.getSelection().toString().trim();
    }
    return '';
  };

  const handleSendQuestion = async () => {
    const question = currentQuestion.trim();
    if (!question) return;

    const userMessage = {
      id: Date.now(),
      text: question,
      sender: 'user',
      timestamp: new Date()
    };
    
    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);
    setError(null);
    setCurrentQuestion('');

    try {
      const context = selectedText ? { selected_text: selectedText } : {};
      const response = await sendQuestion(question, context);

      if (response) {
        const botMessage = {
          id: Date.now() + 1,
          text: response.response || response.answer || '',
          sender: 'bot',
          timestamp: new Date(),
          sourceDocuments: response.source_documents || response.sources || []
        };
        setMessages(prev => [...prev, botMessage]);
      }
    } catch (err) {
      console.error('Chatbot error:', err);
      const errorMessage = {
        id: Date.now() + 2,
        text: `Error: ${err.message || 'An error occurred while getting the response. Please try again.'}`,
        sender: 'error',
        timestamp: new Date()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
      setSelectedText('');
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendQuestion();
    }
    
    // Auto-resize textarea
    if (e.key === 'Enter' && e.shiftKey) {
      // Allow new line
      setTimeout(() => {
        const textarea = e.target;
        textarea.style.height = 'auto';
        textarea.style.height = Math.min(textarea.scrollHeight, 120) + 'px';
      }, 0);
    }
  };

  const handleInputChange = (e) => {
    setCurrentQuestion(e.target.value);
    
    // Auto-resize textarea
    setTimeout(() => {
      const textarea = e.target;
      textarea.style.height = 'auto';
      textarea.style.height = Math.min(textarea.scrollHeight, 120) + 'px';
    }, 0);
  };

  const handleInputFocus = () => {
    const selected = getSelectedText();
    if (selected) {
      setSelectedText(selected);
    }
  };

  const clearChat = () => {
    if (window.confirm('Are you sure you want to clear the chat history?')) {
      setMessages([]);
      if (typeof window !== 'undefined') {
        localStorage.removeItem('chatbotMessages');
      }
    }
  };

  const handleSelectionChange = () => {
    // Update selected text when user selects text on the page
    if (document.activeElement !== inputRef.current) {
      const selected = getSelectedText();
      if (selected) {
        setSelectedText(selected);
      }
    }
  };

  // Listen for text selection changes
  useEffect(() => {
    document.addEventListener('selectionchange', handleSelectionChange);
    return () => {
      document.removeEventListener('selectionchange', handleSelectionChange);
    };
  }, []);

  return (
    <div className="chatbot-container">
      {/* Chat header */}
      <div className="chatbot-header">
        <div style={{ display: 'flex', alignItems: 'center', gap: '12px', flex: 1 }}>
          <div className="chatbot-header-icon">🤖</div>
          <h3 className="chatbot-header-title">AI Assistant</h3>
        </div>
        <button
          onClick={clearChat}
          style={{
            background: 'rgba(255, 255, 255, 0.2)',
            border: 'none',
            color: 'white',
            width: '28px',
            height: '28px',
            borderRadius: '50%',
            cursor: 'pointer',
            fontSize: '0.9em',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            transition: 'background 0.2s ease',
            flexShrink: 0
          }}
          onMouseEnter={(e) => e.target.style.background = 'rgba(255, 255, 255, 0.3)'}
          onMouseLeave={(e) => e.target.style.background = 'rgba(255, 255, 255, 0.2)'}
          aria-label="Clear chat history"
        >
          🗑️
        </button>
      </div>

      <div className="chatbot-messages" role="log" aria-live="polite" aria-label="Chat messages">
        {messages.length === 0 ? (
          <ChatMessage
            message={"**Hello!** Ask me anything about this content. 💬\n\n*Tip:* Select text on the page and ask a question about it!"}
            sender="bot"
            aria-label="bot welcome message"
          />
        ) : (
          <>
            {messages.map(msg => (
              <ChatMessage
                key={msg.id}
                message={msg.text}
                sender={msg.sender}
                sourceDocuments={msg.sourceDocuments}
                timestamp={msg.timestamp}
                aria-label={`${msg.sender} message: ${msg.text.substring(0, 50)}...`}
              />
            ))}
          </>
        )}

        {isLoading && (
          <div className="chat-message bot-message" role="status" aria-label="bot thinking">
            <div className="message-text">
              <span className="loading-dots" aria-hidden="true">Thinking...</span>
            </div>
          </div>
        )}

        <div ref={messagesEndRef} />
      </div>

      <div className="chatbot-input-area" role="form" aria-label="Chat input area">
        {selectedText && (
          <div className="selected-text-context" role="status">
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'flex-start', gap: '8px' }}>
              <div style={{ flex: 1 }}>
                <small>
                  <strong>📝 Context:</strong>
                </small>
                <p style={{
                  margin: '4px 0 0 0',
                  fontStyle: 'italic',
                  fontSize: '0.85em',
                  lineHeight: 1.4,
                  overflow: 'hidden',
                  textOverflow: 'ellipsis',
                  display: '-webkit-box',
                  WebkitLineClamp: 2,
                  WebkitBoxOrient: 'vertical'
                }}>
                  "{selectedText}"
                </p>
              </div>
              <button
                onClick={() => setSelectedText('')}
                style={{
                  background: 'transparent',
                  border: 'none',
                  color: 'var(--ifm-color-gray-500)',
                  cursor: 'pointer',
                  fontSize: '1.1em',
                  padding: '2px 6px',
                  borderRadius: '50%',
                  flexShrink: 0,
                  lineHeight: 1
                }}
                aria-label="Remove selected text context"
                onMouseEnter={(e) => e.target.style.color = 'var(--ifm-color-danger)'}
                onMouseLeave={(e) => e.target.style.color = 'var(--ifm-color-gray-500)'}
              >
                ×
              </button>
            </div>
          </div>
        )}

        <div className="chatbot-input-container">
          <textarea
            ref={inputRef}
            className="chatbot-input"
            value={currentQuestion}
            onChange={handleInputChange}
            onKeyDown={handleKeyPress}
            onFocus={handleInputFocus}
            placeholder="Message AI Assistant..."
            rows="1"
            aria-label="Type your message here"
            disabled={isLoading}
            style={{
              color: 'var(--ifm-color-gray-900)',
              minHeight: '40px',
              fontFamily: 'inherit'
            }}
          />
          <button
            className="chatbot-send-button"
            onClick={handleSendQuestion}
            disabled={isLoading || !currentQuestion.trim()}
            aria-label="Send message"
          >
            {isLoading ? (
              <span style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
                <span className="loading-dots-mini" style={{ fontSize: '0.9em' }}>●</span>
              </span>
            ) : (
              '➤'
            )}
          </button>
        </div>
      </div>
    </div>
  );
};

export default Chatbot;