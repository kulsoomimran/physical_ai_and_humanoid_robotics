# API Reference

## Chat Endpoints

### POST /chat/start
Initialize a new chat session.

**Request:**
```json
{}
```

**Response:**
```json
{
  "session_token": "string",
  "created_at": "timestamp"
}
```

### POST /chat/{session_id}/query
Process a query in a chat session.

**Path Parameters:**
- `session_id`: The session identifier

**Request:**
```json
{
  "session_token": "string",
  "query": "string",
  "context_mode": "book_content|user_text",
  "response_style": "standard|simplified|detailed|examples"
}
```

**Response:**
```json
{
  "response": "string",
  "source_citations": [
    {
      "source_location": "string",
      "content_snippet": "string"
    }
  ]
}
```

## Document Endpoints

### POST /documents/ingest
Ingest documents for RAG processing.

**Request:**
```json
{
  "content": "string",
  "source_url": "string",
  "metadata": {}
}
```

**Response:**
```json
{
  "document_id": "string",
  "chunks_processed": "number"
}
```

### GET /documents/{doc_id}
Retrieve document information.

**Path Parameters:**
- `doc_id`: The document identifier

**Response:**
```json
{
  "id": "string",
  "source_url": "string",
  "created_at": "timestamp",
  "metadata": {}
}
```

## Error Handling

All endpoints return appropriate HTTP status codes:
- `200`: Success
- `400`: Bad request
- `404`: Resource not found
- `500`: Internal server error