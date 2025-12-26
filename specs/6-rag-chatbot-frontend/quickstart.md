# Quickstart: Frontend Integration for RAG Chatbot

## Overview
This guide provides instructions for integrating the RAG chatbot into Docusaurus book pages.

## Prerequisites
- Docusaurus project set up
- Running FastAPI backend with `/chat` endpoint
- Node.js and npm installed

## Installation Steps

### 1. Add the Chat Component
Create a new React component `Chatbot.jsx` in your Docusaurus `src/components` directory:

```jsx
import React, { useState, useEffect } from 'react';
import './Chatbot.css';

const Chatbot = ({ backendUrl = 'https://kulsoomimran-rag-chatbot.hf.space/' }) => {
  // Component implementation will go here
};

export default Chatbot;
```

### 2. Environment Configuration
Add the backend URL to your environment configuration:

```bash
# For local development
BACKEND_URL=http://localhost:8000

# For production
BACKEND_URL=https://kulsoomimran-rag-chatbot.hf.space/
```

### 3. Integrate into Layout
Add the chatbot component to your Docusaurus layout, typically in `src/pages/Layout.js` or as a plugin in `docusaurus.config.js`.

### 4. Run the Application
```bash
npm run start
```

## Usage
- Type your question in the input field
- Optionally select text on the page before asking a question to include it as context
- View responses in the chat interface
- Loading indicators will show during API communication
- Error messages will display if the backend is unavailable

## Configuration Options
- `backendUrl`: The URL of your FastAPI backend
- `timeout`: Request timeout in milliseconds
- `showOnAllPages`: Whether to show the chatbot on all pages (default: true)