import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import Chatbot from '../components/Chatbot';
import { useLocation } from '@docusaurus/router';

export default function Layout(props) {
  const { pathname } = useLocation();

  // Don't show chatbot on the homepage to avoid clutter
  const showChatbot = !pathname.match(/^\/?$/) && !pathname.includes('/blog');

  return (
    <OriginalLayout {...props}>
      {props.children}
      {showChatbot && (
        <div className="chatbot-fixed">
          <Chatbot />
        </div>
      )}
    </OriginalLayout>
  );
}