import React from 'react';
import ReactMarkdown from 'react-markdown';
import rehypeHighlight from 'rehype-highlight';
import remarkGfm from 'remark-gfm';
import './ChatMessage.css';
import 'highlight.js/styles/github.css';
import '../../css/highlight-dark.css'; // Optional: Add a dark theme for highlight.js

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
    <div className={messageClass} {...props}>
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
  );
};

export default ChatMessage;