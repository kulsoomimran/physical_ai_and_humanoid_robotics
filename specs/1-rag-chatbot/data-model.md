# Data Model: Embedded RAG Chatbot

**Feature**: 1-rag-chatbot
**Date**: 2025-12-07

## Entity: Query
**Description**: A user's question or request for information, including metadata about context and user preferences

**Fields**:
- `id` (UUID): Unique identifier for the query
- `session_id` (UUID): Reference to the conversation session
- `content` (Text): The actual text of the user's query
- `query_type` (Enum): Type of query (book_content, user_text, mixed)
- `created_at` (DateTime): Timestamp when the query was submitted
- `processed_at` (DateTime): Timestamp when the query was processed
- `user_preferences` (JSON): User preferences for response style/level

**Validation**:
- `content` must not be empty
- `query_type` must be one of the defined enum values
- `session_id` must reference an existing Conversation

## Entity: Document Chunk
**Description**: Segments of book content or user-provided text that have been processed for vector storage

**Fields**:
- `id` (UUID): Unique identifier for the chunk
- `source_id` (UUID): Reference to the original document/source
- `source_type` (Enum): Type of source (book_content, user_text)
- `content` (Text): The actual text content of the chunk
- `chunk_order` (Integer): Order of this chunk within the original document
- `embedding_vector` (Vector): The vector representation of the content
- `metadata` (JSON): Additional metadata about the chunk
- `created_at` (DateTime): Timestamp when the chunk was created

**Validation**:
- `content` must not be empty
- `source_type` must be one of the defined enum values
- `embedding_vector` must be a valid vector representation

## Entity: Embedding Vector
**Description**: Reference to the numerical representation of text content used for similarity matching in the vector database (stored in Qdrant, with reference here)

**Fields**:
- `id` (UUID): Unique identifier for the embedding
- `chunk_id` (UUID): Reference to the source document chunk
- `vector_db_id` (String): ID in the vector database (Qdrant)
- `model_used` (String): Name of the model used to generate the embedding
- `created_at` (DateTime): Timestamp when the embedding was generated

**Validation**:
- `chunk_id` must reference an existing Document Chunk
- `vector_db_id` must be unique within the system

## Entity: Conversation
**Description**: A sequence of related queries and responses that maintains context for the user session

**Fields**:
- `id` (UUID): Unique identifier for the conversation
- `session_token` (String): Token for identifying the session (for web use)
- `user_id` (UUID): Reference to the user (optional, for registered users)
- `created_at` (DateTime): Timestamp when the conversation started
- `updated_at` (DateTime): Timestamp when the conversation was last updated
- `metadata` (JSON): Additional metadata about the conversation
- `active` (Boolean): Whether the conversation is currently active

**Validation**:
- `session_token` must be unique
- `active` defaults to true when conversation is created

## Entity: Source Citation
**Description**: Reference to the specific part of the book or user text that contributed to the response

**Fields**:
- `id` (UUID): Unique identifier for the citation
- `response_id` (UUID): Reference to the response containing this citation
- `chunk_id` (UUID): Reference to the document chunk that was cited
- `relevance_score` (Float): Score indicating how relevant this chunk was to the response
- `citation_text` (Text): The specific text that was cited
- `source_location` (String): Location in the original source (page, section, etc.)

**Validation**:
- `response_id` and `chunk_id` must reference existing records
- `relevance_score` must be between 0 and 1

## Entity: Response
**Description**: The system's response to a user query

**Fields**:
- `id` (UUID): Unique identifier for the response
- `query_id` (UUID): Reference to the original query
- `content` (Text): The actual text of the response
- `response_type` (Enum): Type of response (factual, explanatory, comparative)
- `created_at` (DateTime): Timestamp when the response was generated
- `processing_time` (Float): Time taken to generate the response in seconds

**Validation**:
- `query_id` must reference an existing Query
- `content` must not be empty

## Relationships

- Conversation (1) ←→ Query (Many): A conversation contains multiple queries
- Query (1) ←→ Response (1): Each query has one response
- Response (1) ←→ Source Citation (Many): A response can cite multiple sources
- Document Chunk (1) ←→ Source Citation (Many): A document chunk can be cited in multiple responses
- Document Chunk (1) ←→ Embedding Vector (1): Each document chunk has one embedding vector