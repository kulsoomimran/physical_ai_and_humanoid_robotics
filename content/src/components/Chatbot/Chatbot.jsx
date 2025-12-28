import React, { useState, useEffect, useRef } from 'react';
import ReactMarkdown from 'react-markdown';
import rehypeHighlight from 'rehype-highlight';
import remarkGfm from 'remark-gfm';
import useChatAPI from './hooks/useChatAPI';
import icon from "../../../static/img/icon.png";
import 'highlight.js/styles/github.css';
import '../../css/highlight-dark.css';
import './Chatbot.css';

// Book Icon Component
const BookIcon = () => (
  <img src={icon} alt="Book Icon" style={{ width: '18px', height: '18px' }} />
);

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
  const [isCollapsed, setIsCollapsed] = useState(() => {
    if (typeof window !== 'undefined') {
      const saved = localStorage.getItem('chatbotCollapsed');
      return saved === 'true';
    }
    return false;
  });
  const messagesEndRef = useRef(null);
  const messagesContainerRef = useRef(null);
  const inputRef = useRef(null);

  const { sendQuestion } = useChatAPI();

  // Persist messages
  useEffect(() => {
    if (typeof window !== 'undefined' && messages.length > 0) {
      localStorage.setItem('chatbotMessages', JSON.stringify(messages));
    }
  }, [messages]);

  // Listen for clear history event from Layout
  useEffect(() => {
    const handleClearHistory = () => {
      setMessages([]);
    };
    
    if (typeof window !== 'undefined') {
      window.addEventListener('chatbotClearHistory', handleClearHistory);
      return () => {
        window.removeEventListener('chatbotClearHistory', handleClearHistory);
      };
    }
  }, []);

  // Persist collapsed state
  useEffect(() => {
    if (typeof window !== 'undefined') {
      localStorage.setItem('chatbotCollapsed', isCollapsed.toString());
    }
  }, [isCollapsed]);

  // Auto-scroll to bottom
  useEffect(() => {
    scrollToBottom();
  }, [messages, isLoading]);

  // Auto-focus input on mount
  useEffect(() => {
    inputRef.current?.focus();
  }, []);

  const scrollToBottom = () => {
    if (messagesContainerRef.current) {
      // Smooth scroll to bottom within container
      requestAnimationFrame(() => {
        if (messagesContainerRef.current) {
          messagesContainerRef.current.scrollTo({
            top: messagesContainerRef.current.scrollHeight,
            behavior: 'smooth'
          });
        }
      });
    }
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

  // ChatMessage component - merged into Chatbot
  const ChatMessage = ({ message, sender, sourceDocuments = [], timestamp, ...props }) => {
    const messageClass = `chat-message ${sender}-message`;

    const formatTime = (date) => {
      if (!date) return '';
      const d = new Date(date);
      return d.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
    };

    // Markdown components configuration
    const markdownComponents = {
      p: ({ children }) => <p style={{ margin: '0 0 8px 0' }}>{children}</p>,
      ul: ({ children }) => <ul style={{ margin: '8px 0', paddingLeft: '20px' }}>{children}</ul>,
      ol: ({ children }) => <ol style={{ margin: '8px 0', paddingLeft: '20px' }}>{children}</ol>,
      li: ({ children }) => <li style={{ marginBottom: '4px' }}>{children}</li>,
      code: ({ node, inline, className, children, ...props }) => {
        const match = /language-(\w+)/.exec(className || '');
        return !inline && match ? (
          <pre style={{
            background: 'linear-gradient(145deg, var(--ifm-color-emphasis-100), var(--ifm-color-emphasis-200))',
            padding: '16px',
            borderRadius: '12px',
            overflowX: 'auto',
            margin: '14px 0',
            fontSize: '0.85em',
            border: '1px solid var(--ifm-color-emphasis-300)',
            boxShadow: 'inset 0 2px 4px rgba(0, 0, 0, 0.05)',
            position: 'relative',
            fontFamily: "'SFMono-Regular', Consolas, 'Liberation Mono', Menlo, monospace"
          }}>
            <code className={className} {...props}>
              {children}
            </code>
          </pre>
        ) : (
          <code style={{
            background: 'linear-gradient(145deg, rgba(0, 0, 0, 0.08), rgba(0, 0, 0, 0.05))',
            padding: '4px 8px',
            borderRadius: '6px',
            fontSize: '0.85em',
            fontFamily: "'SFMono-Regular', Consolas, 'Liberation Mono', Menlo, monospace",
            border: '1px solid rgba(0, 0, 0, 0.1)',
            boxShadow: 'inset 0 1px 2px rgba(0, 0, 0, 0.05)'
          }} {...props}>
            {children}
          </code>
        );
      },
      a: ({ href, children }) => (
        <a
          href={href}
          target="_blank"
          rel="noopener noreferrer"
          style={{
            color: 'var(--ifm-color-primary)',
            textDecoration: 'none',
            fontWeight: 500,
            borderBottom: '1px solid transparent',
            transition: 'all 0.2s ease',
            borderRadius: '2px'
          }}
          onMouseEnter={(e) => {
            e.target.style.textDecoration = 'none';
            e.target.style.borderBottom = `1px solid var(--ifm-color-primary)`;
            e.target.style.background = 'rgba(var(--ifm-color-primary-rgb), 0.1)';
            e.target.style.padding = '1px 2px';
            e.target.style.borderRadius = '3px';
          }}
          onMouseLeave={(e) => {
            e.target.style.textDecoration = 'none';
            e.target.style.borderBottom = '1px solid transparent';
            e.target.style.background = 'transparent';
            e.target.style.padding = '0';
            e.target.style.borderRadius = '2px';
          }}
        >
          {children}
        </a>
      )
    };

    return (
      <div className={`message-wrapper ${sender}-message-wrapper`} {...props}>
        {sender === 'bot' && (
          <div className="message-avatar">
            <BookIcon />
          </div>
        )}
        <div className={messageClass}>
          <div className="message-text">
            {message ? (
              <ReactMarkdown
                remarkPlugins={[remarkGfm]}
                rehypePlugins={[rehypeHighlight]}
                components={markdownComponents}
              >
                {message}
              </ReactMarkdown>
            ) : (
              <em>No content</em>
            )}
          </div>

          {timestamp && (
            <div className="message-timestamp">
              {formatTime(timestamp)}
            </div>
          )}

          {sourceDocuments.length > 0 && (
            <div className="source-documents" aria-label="Source documents">
              <small>📚 Sources:</small>
              <ul>
                {sourceDocuments.map((doc, index) => (
                  <li key={index}>
                    {doc.url ? (
                      <a
                        href={doc.url}
                        target="_blank"
                        rel="noopener noreferrer"
                        aria-label={`Source document: ${doc.title || doc.url}`}
                      >
                        {doc.title || 'Untitled Document'}
                        {doc.page && <span style={{ fontSize: '0.9em', marginLeft: '4px' }}>(p. {doc.page})</span>}
                      </a>
                    ) : (
                      <span>
                        {doc.title || 'Untitled source'}
                        {doc.page && <span style={{ fontSize: '0.9em', marginLeft: '4px' }}>(p. {doc.page})</span>}
                      </span>
                    )}
                  </li>
                ))}
              </ul>
            </div>
          )}
        </div>
        {sender === 'user' && <div className="message-spacer"></div>}
      </div>
    );
  };

  return (
    <div className={`chatbot-wrapper ${isCollapsed ? 'collapsed' : ''}`}>
      <div className="chatbot-container">
        {/* Chat header */}
        <div className="chatbot-header">
          <div style={{ display: 'flex', alignItems: 'center', gap: '10px', flex: 1 }}>
            <div className="chatbot-header-icon">
              <BookIcon />
            </div>
            <h3 className="chatbot-header-title">Humanoid AI</h3>
          </div>
          <div style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
            <button
              onClick={clearChat}
              className="chatbot-header-button"
              aria-label="Clear chat history"
              title="Clear chat history"
            >
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M3 6h18M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6m3 0V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2"/>
              </svg>
            </button>
            <button
              onClick={() => setIsCollapsed(true)}
              className="chatbot-header-button"
              aria-label="Close chatbot"
              title="Close chatbot"
            >
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                <line x1="18" y1="6" x2="6" y2="18"></line>
                <line x1="6" y1="6" x2="18" y2="18"></line>
              </svg>
            </button>
          </div>
        </div>

      <div 
        ref={messagesContainerRef}
        className="chatbot-messages" 
        role="log" 
        aria-live="polite" 
        aria-label="Chat messages"
      >
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
          <div className="message-wrapper bot-message-wrapper" role="status" aria-label="bot thinking">
            <div className="message-avatar">
              <BookIcon />
            </div>
            <div className="chat-message bot-message">
              <div className="message-text">
                <span className="loading-dots" aria-hidden="true">Thinking...</span>
              </div>
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
            placeholder="Type your message..."
            rows="1"
            aria-label="Type your message here"
            disabled={isLoading}
          />
          <button
            className="chatbot-send-button"
            onClick={handleSendQuestion}
            disabled={isLoading || !currentQuestion.trim()}
            aria-label="Send message"
          >
            {isLoading ? (
              <span className="loading-dots-mini">●</span>
            ) : (
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                <line x1="22" y1="2" x2="11" y2="13"></line>
                <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
              </svg>
            )}
          </button>
        </div>
      </div>
      </div>
      
      {/* Toggle button */}
      {/* <button
        className="chatbot-toggle"
        onClick={() => setIsCollapsed(!isCollapsed)}
        aria-label={isCollapsed ? 'Expand chatbot' : 'Collapse chatbot'}
      >
        <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
          {isCollapsed ? (
            <path d="M5 12h14M12 5l7 7-7 7"/>
          ) : (
            <path d="M19 12H5M12 19l-7-7 7-7"/>
          )}
        </svg>
      </button> */}
    </div>
  );
};

export default Chatbot;