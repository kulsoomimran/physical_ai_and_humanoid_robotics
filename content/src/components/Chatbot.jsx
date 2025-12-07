import React, { useState, useEffect, useRef } from 'react';
import { useColorMode } from '@docusaurus/theme-common';
import './Chatbot.css';

const Chatbot = ({ contextMode = 'book_content' }) => {
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState(null);
  const [selectedText, setSelectedText] = useState('');
  const [responseStyle, setResponseStyle] = useState('standard'); // standard, simplified, detailed, examples
  const messagesEndRef = useRef(null);
  const { colorMode } = useColorMode();

  // Initialize chat session
  useEffect(() => {
    const initializeSession = async () => {
      try {
        const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000';
    const response = await fetch(`${API_BASE_URL}/chat/start`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({}),
        });

        if (response.ok) {
          const data = await response.json();
          setSessionId(data.session_token);
        } else {
          console.error('Failed to initialize chat session');
        }
      } catch (error) {
        console.error('Error initializing chat session:', error);
      }
    };

    initializeSession();
  }, []);

  // Auto-scroll to bottom of messages
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Function to get selected text
  const getSelectedText = () => {
    const selection = window.getSelection();
    return selection.toString().trim();
  };

  // Handle text selection
  useEffect(() => {
    const handleSelection = () => {
      const text = getSelectedText();
      if (text) {
        setSelectedText(text);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  const sendMessage = async () => {
    if (!inputValue.trim() || !sessionId || isLoading) return;

    const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000';
    const userMessage = { id: Date.now(), text: inputValue, sender: 'user', timestamp: new Date() };
    setMessages(prev => [...prev, userMessage]);
    const tempId = Date.now();
    setIsLoading(true);

    try {
      // Add a temporary loading message
      setMessages(prev => [...prev, { id: tempId, text: '', sender: 'bot', loading: true, timestamp: new Date() }]);

      const response = await fetch(`${API_BASE_URL}/chat/${sessionId}/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          session_token: sessionId,
          query: inputValue,
          context_mode: selectedText ? 'user_text' : contextMode,
          response_style: responseStyle,
        }),
      });

      if (response.ok) {
        const data = await response.json();

        // Remove the loading message
        setMessages(prev => prev.filter(msg => msg.id !== tempId));

        // Add the actual response
        const botMessage = {
          id: Date.now(),
          text: data.response,
          sender: 'bot',
          timestamp: new Date(),
          citations: data.source_citations || [],
        };
        setMessages(prev => [...prev, botMessage]);
      } else {
        // Remove the loading message
        setMessages(prev => prev.filter(msg => msg.id !== tempId));

        // Add error message
        const errorMessage = {
          id: Date.now(),
          text: 'Sorry, I encountered an error processing your request. Please try again.',
          sender: 'bot',
          timestamp: new Date(),
          error: true,
        };
        setMessages(prev => [...prev, errorMessage]);
      }
    } catch (error) {
      // Remove the loading message
      setMessages(prev => prev.filter(msg => msg.id !== tempId));

      // Add error message
      const errorMessage = {
        id: Date.now(),
        text: 'Sorry, I encountered a network error. Please check your connection and try again.',
        sender: 'bot',
        timestamp: new Date(),
        error: true,
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
      setInputValue('');
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  const handleNewChat = async () => {
    try {
      const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000';
    const response = await fetch(`${API_BASE_URL}/chat/start`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({}),
      });

      if (response.ok) {
        const data = await response.json();
        setSessionId(data.session_token);
        setMessages([]);
      }
    } catch (error) {
      console.error('Error starting new chat:', error);
    }
  };

  const handleUseSelectedText = () => {
    if (selectedText) {
      setInputValue(selectedText);
    }
  };

  return (
    <div className={`chatbot-container ${colorMode}`}>
      <div className="chatbot-header">
        <h3>Physical AI & Robotics Assistant</h3>
        <div className="header-actions">
          <select
            value={responseStyle}
            onChange={(e) => setResponseStyle(e.target.value)}
            className="response-style-select"
          >
            <option value="standard">Standard Response</option>
            <option value="simplified">Simplified</option>
            <option value="detailed">Detailed</option>
            <option value="examples">With Examples</option>
          </select>
          <button onClick={handleNewChat} className="new-chat-button">
            New Chat
          </button>
        </div>
      </div>

      {selectedText && (
        <div className="selected-text-preview">
          <p><strong>Selected text:</strong> "{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"</p>
          <button onClick={handleUseSelectedText} className="use-selected-text-button">
            Use Selected Text
          </button>
        </div>
      )}

      <div className="chatbot-messages">
        {messages.length === 0 ? (
          <div className="welcome-message">
            <p>Hello! I'm your Physical AI & Humanoid Robotics assistant.</p>
            <p>Ask me questions about the book content or paste your own text to analyze.</p>
          </div>
        ) : (
          messages.map((message) => (
            <div
              key={message.id}
              className={`message ${message.sender} ${message.error ? 'error' : ''}`}
            >
              {message.loading ? (
                <div className="loading-indicator">
                  <div className="loading-dot"></div>
                  <div className="loading-dot"></div>
                  <div className="loading-dot"></div>
                </div>
              ) : (
                <>
                  <div className="message-text">{message.text}</div>
                  {message.citations && message.citations.length > 0 && (
                    <div className="citations">
                      <strong>Sources:</strong>
                      <ul>
                        {message.citations.map((citation, index) => (
                          <li key={index}>
                            {citation.source_location || `Source ${index + 1}`}
                          </li>
                        ))}
                      </ul>
                    </div>
                  )}
                </>
              )}
            </div>
          ))
        )}
        <div ref={messagesEndRef} />
      </div>

      <div className="chatbot-input-area">
        <textarea
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          onKeyPress={handleKeyPress}
          placeholder="Ask a question about the book content..."
          className="chat-input"
          rows="3"
        />
        <button
          onClick={sendMessage}
          disabled={!inputValue.trim() || !sessionId || isLoading}
          className="send-button"
        >
          {isLoading ? 'Sending...' : 'Send'}
        </button>
      </div>
    </div>
  );
};

export default Chatbot;