# Data Model: FastAPI Backend for RAG Chatbot

## ChatRequest Entity

**Purpose**: Represents a user query with optional selected text context

**Fields**:
- `question` (string, required): The user's question or query
- `selected_text` (string, optional): Additional context from selected text
- `thread_id` (string, optional): ID to maintain conversation context
- `metadata` (object, optional): Additional metadata for the request

**Validation Rules**:
- `question` must be between 1 and 2000 characters
- `selected_text` if provided, must be between 1 and 10000 characters
- `thread_id` if provided, must be a valid UUID format

## ChatResponse Entity

**Purpose**: Represents the response from the RAG agent

**Fields**:
- `response` (string, required): The agent's response to the question
- `thread_id` (string, required): ID of the conversation thread
- `metadata` (object, optional): Additional metadata about the response
- `timestamp` (string, required): ISO 8601 formatted timestamp

**Validation Rules**:
- `response` must be between 1 and 10000 characters
- `thread_id` must be a valid UUID format
- `timestamp` must be in ISO 8601 format

## HealthCheck Entity

**Purpose**: Represents the health status of the backend service

**Fields**:
- `status` (string, required): "healthy" or "unhealthy"
- `timestamp` (string, required): ISO 8601 formatted timestamp
- `version` (string, optional): API version information

**Validation Rules**:
- `status` must be either "healthy" or "unhealthy"
- `timestamp` must be in ISO 8601 format

## Error Entity

**Purpose**: Represents error responses from the API

**Fields**:
- `error` (string, required): Error message
- `code` (string, required): Error code
- `details` (object, optional): Additional error details

**Validation Rules**:
- `error` must be between 1 and 500 characters
- `code` must be a valid HTTP status code