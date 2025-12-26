# Data Model: Agent with Retrieval-Augmented Generation

## Overview

This document defines the data models for the AI agent with retrieval-augmented generation capabilities. It outlines the entities, their attributes, and relationships within the system.

## Key Entities

### 1. RAGAgent

The main agent class that orchestrates the interaction between the OpenAI Assistant API and the Qdrant retrieval system.

**Attributes:**
- `client`: OpenAI API client instance
- `assistant`: OpenAI Assistant object
- `retrieval_tool`: Instance of QdrantRetrievalTool
- `instructions`: Instructions for the assistant's behavior

**Methods:**
- `chat(user_message, thread_id)`: Process user message and return response
- `get_assistant_id()`: Return the assistant ID
- `cleanup()`: Clean up resources

### 2. QdrantRetrievalTool

A tool class that handles the retrieval of relevant document chunks from Qdrant.

**Attributes:**
- `qdrant_client`: Qdrant client instance
- `cohere_client`: Cohere client instance
- `collection_name`: Name of the Qdrant collection to search
- `top_k`: Default number of results to retrieve

**Methods:**
- `generate_query_embedding(query_text)`: Generate embedding for a query
- `search(query, top_k)`: Search Qdrant for relevant chunks
- `format_retrieved_content(chunks)`: Format chunks for AI context

### 3. RetrievedChunk

Represents a single chunk of text retrieved from Qdrant.

**Attributes:**
- `id`: Unique identifier for the chunk
- `text`: The actual text content of the chunk
- `score`: Relevance score from the search
- `vector_id`: ID in the vector database
- `metadata`: Dictionary containing additional information

**Metadata Attributes:**
- `source_url`: URL where the content originated
- `document_id`: ID of the original document
- `chunk_index`: Index of this chunk within the document
- `page_number`: Page number if applicable
- `section_title`: Title of the section if applicable

### 4. ChatResponse

The response structure returned by the agent's chat method.

**Attributes:**
- `response`: The text response from the AI
- `thread_id`: ID of the conversation thread
- `status`: Status of the operation (completed, error, etc.)

### 5. ToolCallResult

The result structure returned by the retrieval tool.

**Attributes:**
- `retrieved_content`: Formatted content from search results
- `chunk_count`: Number of chunks retrieved
- `query`: The original search query

## Data Flow

### Query Processing Flow
1. User provides `user_message` to `RAGAgent.chat()`
2. Message is added to OpenAI thread
3. Assistant determines if `search_book_content` tool is needed
4. Tool call parameters are passed to `QdrantRetrievalTool.search()`
5. Query embedding is generated using Cohere
6. Qdrant search returns `RetrievedChunk` objects
7. Chunks are formatted and returned to Assistant
8. Assistant generates response based on retrieved content
9. `ChatResponse` is returned to user

## Validation Rules

### RetrievedChunk Validation
- Text must not be empty
- Score must be between 0 and 1
- Metadata must contain required source information (source_url, document_id)

### ChatResponse Validation
- Response must not be empty
- Thread ID must be valid if provided
- Status must be one of: "completed", "error", "requires_action"

## State Management

### Conversation State
- Thread ID is maintained between calls for conversation continuity
- Assistant maintains conversation context internally
- No explicit state management needed in the agent class

### Tool State
- Qdrant and Cohere clients are initialized once per tool instance
- Configuration parameters are loaded from environment variables
- No dynamic state changes during operation

## API Contracts

### Tool Function Definition
```json
{
  "type": "function",
  "function": {
    "name": "search_book_content",
    "description": "Search through book content to find relevant information for answering user questions",
    "parameters": {
      "type": "object",
      "properties": {
        "query": {
          "type": "string",
          "description": "The search query to find relevant information in the book content"
        },
        "top_k": {
          "type": "integer",
          "description": "Number of top results to retrieve (default: 5)",
          "default": 5
        }
      },
      "required": ["query"]
    }
  }
}
```

### Internal Data Structures
- RetrievedChunk: Maps to Qdrant search results with additional formatting
- ToolCallResult: JSON serializable for OpenAI API compatibility
- ChatResponse: Contains all necessary information for client applications