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
              <span className="chatbot-title">RAG Chatbot</span>
              <button
                className="chatbot-close-button"
                onClick={toggleChatbot}
                aria-label="Close chatbot"
              >
                ✕
              </button>
            </div>
            <Chatbot />
          </div>
        </>
      )}
    </OriginalLayout>
  );
}