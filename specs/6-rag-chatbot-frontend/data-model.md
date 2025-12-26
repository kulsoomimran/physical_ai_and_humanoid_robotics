# Data Model: Frontend Integration for RAG Chatbot

## Entities

### UserQuestion
- **id**: string (auto-generated)
- **text**: string (required, the question text from user)
- **selectedTextContext**: string (optional, text selected on the page when question was asked)
- **timestamp**: datetime (when the question was submitted)
- **status**: enum (pending, sent, error)

### ChatResponse
- **id**: string (auto-generated, matches the corresponding question id)
- **questionId**: string (reference to the UserQuestion)
- **text**: string (required, the response from the backend)
- **timestamp**: datetime (when the response was received)
- **status**: enum (success, error, partial)
- **sourceDocuments**: array of objects (optional, references to source documents if provided by backend)

### ChatSession
- **id**: string (auto-generated)
- **userId**: string (optional, for tracking if needed)
- **messages**: array of objects (list of UserQuestion and ChatResponse pairs)
- **createdAt**: datetime
- **updatedAt**: datetime

### APIConfig
- **backendUrl**: string (URL for the FastAPI backend)
- **timeout**: number (API request timeout in milliseconds)
- **retryAttempts**: number (number of retry attempts for failed requests)

## State Models

### ChatUIState
- **isLoading**: boolean (indicates if a request is in progress)
- **error**: string (error message if any)
- **currentQuestion**: string (the current question being typed)
- **messages**: array of objects (history of questions and responses)
- **selectedText**: string (currently selected text on the page)