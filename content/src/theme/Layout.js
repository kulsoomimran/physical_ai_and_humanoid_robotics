import React, { useState } from 'react';
import OriginalLayout from '@theme-original/Layout';
import Chatbot from '@site/src/components/Chatbot/Chatbot';
import { useLocation } from '@docusaurus/router';

export default function Layout(props) {
  const location = useLocation();
  const [isChatbotOpen, setIsChatbotOpen] = useState(false);

  // Only show chatbot on docs pages, not on the homepage or other pages
  const shouldShowChatbot = location.pathname.startsWith('/docs/');

  const toggleChatbot = () => {
    setIsChatbotOpen(!isChatbotOpen);
  };

  const clearChatHistory = () => {
    if (window.confirm('Are you sure you want to clear the chat history?')) {
      if (typeof window !== 'undefined') {
        localStorage.removeItem('chatbotMessages');
        // Dispatch a custom event that Chatbot can listen to
        window.dispatchEvent(new CustomEvent('chatbotClearHistory'));
      }
    }
  };

  return (
    <OriginalLayout {...props}>
      {props.children}
      {shouldShowChatbot && (
        <>
          <button
            className={`chatbot-toggle-button ${isChatbotOpen ? 'hidden' : ''}`}
            onClick={toggleChatbot}
            aria-label={isChatbotOpen ? "Close chatbot" : "Open chatbot"}
            title={isChatbotOpen ? "Close chatbot" : "Open chatbot"}
          >
            💬
          </button>
          <div className={`chatbot-overlay ${isChatbotOpen ? 'open' : 'closed'}`}>
            <div className="chatbot-header-bar">
              <span className="chatbot-title">Humanoid AI</span>
              <div style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
                <button
                  className="chatbot-clear-button"
                  onClick={clearChatHistory}
                  aria-label="Clear chat history"
                  title="Clear chat history"
                  style={{
                    background: 'rgba(255, 255, 255, 0.15)',
                    border: 'none',
                    color: 'white',
                    width: '32px',
                    height: '32px',
                    borderRadius: '50%',
                    cursor: 'pointer',
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'center',
                    transition: 'background 0.2s ease',
                    padding: 0
                  }}
                  onMouseEnter={(e) => e.target.style.background = 'rgba(255, 255, 255, 0.25)'}
                  onMouseLeave={(e) => e.target.style.background = 'rgba(255, 255, 255, 0.15)'}
                >
                  <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                    <path d="M3 6h18M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6m3 0V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2"/>
                  </svg>
                </button>
                <button
                  className="chatbot-close-button"
                  onClick={toggleChatbot}
                  aria-label="Close chatbot"
                >
                  ✕
                </button>
              </div>
            </div>
            <Chatbot />
          </div>
        </>
      )}
    </OriginalLayout>
  );
}