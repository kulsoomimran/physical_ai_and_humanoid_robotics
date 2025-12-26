# RAG Chatbot Frontend Integration

This document describes the RAG (Retrieval Augmented Generation) chatbot frontend component that has been integrated into the Docusaurus documentation site.

## Overview

The RAG chatbot allows users to ask questions about the documentation content directly from any page. It sends questions to the backend RAG system and displays relevant responses with source references.

## Features

- **Context-aware Q&A**: Ask questions about the current page content
- **Selected Text Context**: Automatically includes selected text as context for questions
- **Real-time Responses**: Get answers from the RAG backend
- **Source References**: Responses include links to source documents
- **Persistent Chat History**: Chat history preserved between page visits
- **Responsive Design**: Works on desktop and mobile devices
- **Accessibility Support**: Keyboard navigation and screen reader support

## Configuration

The chatbot can be configured using environment variables in the `.env` file:

```
REACT_APP_BACKEND_URL=https://kulsoomimran-rag-chatbot.hf.space/
REACT_APP_API_TIMEOUT=30000
REACT_APP_API_RETRY_ATTEMPTS=3
```

## How It Works

1. The chatbot is integrated into the Docusaurus layout via `src/theme/Layout.js`
2. It only appears on documentation pages (URLs starting with `/docs/`)
3. When a user types a question, it's sent to the backend `/chat` endpoint
4. The response is displayed in the chat interface
5. If text is selected on the page, it's automatically included as context

## API Communication

The `useChatAPI` hook handles communication with the backend:

- **Endpoint**: POST `/chat`
- **Request**: `{ question: string, context: { selected_text?: string } }`
- **Response**: `{ answer: string, source_documents: [] }`

## Components

- `Chatbot.jsx`: Main chat interface component
- `ChatMessage.jsx`: Individual message display component
- `useChatAPI.js`: API communication hook with timeout/retry logic
- `Chatbot.css`: Main styling
- `ChatMessage.css`: Message-specific styling

## Accessibility

- Keyboard navigation support (Enter to send, arrow keys for message navigation)
- Proper ARIA labels and roles
- Screen reader support
- Focus management

## Development

To modify the chatbot:

1. Components are located in `content/src/components/Chatbot/`
2. Layout integration is in `content/src/theme/Layout.js`
3. Styling is in `content/src/css/custom.css`
4. The component uses React, ReactMarkdown for formatting, and localStorage for persistence