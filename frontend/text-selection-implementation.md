# Frontend Implementation Guide: Text Selection and Paste Functionality

## Overview
This document outlines the frontend implementation required to support User Story 2: Query User-Selected Text via RAG Chatbot.

## Required Frontend Features

### 1. Text Selection Functionality
- Enable users to select text within the book content
- Capture the selected text and its context/location
- Provide a clear UI indicator for selected text
- Allow users to trigger a query based on selected text

### 2. Text Paste Functionality
- Provide a text input area for users to paste external content
- Implement character limits and validation
- Include a "Submit for Analysis" button
- Show feedback when text is being processed

### 3. Context Mode Switching
- UI controls to switch between context modes:
  - `book_content`: Search only book content
  - `user_text`: Search only user-provided text
  - `mixed`: Search both book and user content

### 4. API Integration
The frontend needs to integrate with the following backend endpoints:

#### For Text Ingestion:
- **POST** `/documents/ingest-user-text`
  - Request body: `{"session_token": "string", "text_content": "string", "user_id": "string"}`
  - Response: Confirmation of text ingestion

#### For Queries:
- **POST** `/chat/{session_token}/query`
  - Include `context_mode` parameter in request
  - Example: `{"session_token": "token", "query": "question", "context_mode": "user_text"}`

### 5. UI Components to Implement

#### Text Selection Component
```javascript
// Example implementation approach
document.addEventListener('mouseup', function() {
  const selectedText = window.getSelection().toString().trim();
  if (selectedText) {
    showQueryButton(selectedText);
  }
});
```

#### Text Paste Modal/Panel
- Text area with character counter
- Submit button
- Validation messages
- Loading states

#### Context Mode Selector
- Toggle buttons or dropdown for context modes
- Visual indication of current mode
- Mode persistence across sessions

## Implementation Notes

1. **Security**: Sanitize user input before sending to backend
2. **Performance**: Consider limiting text length to prevent API overload
3. **User Experience**: Provide clear instructions and feedback
4. **Accessibility**: Ensure all components are accessible

## Integration Points

- Connect to the backend RAG API
- Maintain session state between components
- Handle API errors gracefully
- Implement proper loading states