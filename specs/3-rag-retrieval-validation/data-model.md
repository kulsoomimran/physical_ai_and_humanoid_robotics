# Data Model: RAG Retrieval Validation

## Entities

### QueryVector
**Description**: The embedded representation of user queries used for similarity matching
- **Fields**:
  - `id`: Unique identifier for the query
  - `vector`: The embedding vector representation
  - `text`: Original query text
  - `timestamp`: When the query was created
- **Validation Rules**:
  - Vector must have the same dimension as stored embeddings
  - Text must not be empty

### RetrievedChunk
**Description**: Text chunks returned from the vector database that match the query semantics
- **Fields**:
  - `id`: Unique identifier for the chunk
  - `text`: The actual text content of the chunk
  - `score`: Similarity score from the retrieval process
  - `vector_id`: ID of the vector in Qdrant
  - `metadata`: Dictionary containing source information
- **Validation Rules**:
  - Text must not be empty
  - Score must be between 0 and 1
  - Metadata must contain required source information

### SourceMetadata
**Description**: Information linking retrieved content back to original URLs and document identifiers
- **Fields**:
  - `source_url`: URL of the original document
  - `document_id`: Unique identifier of the source document
  - `chunk_index`: Position of the chunk in the original document
  - `page_number`: Page number if applicable
  - `section_title`: Title of the section containing the chunk
- **Validation Rules**:
  - Source URL must be a valid URL
  - Document ID must exist and be non-empty

### ValidationResult
**Description**: Output representing the result of a validation check
- **Fields**:
  - `query_id`: ID of the query being validated
  - `retrieved_chunks`: List of RetrievedChunk objects
  - `is_valid`: Boolean indicating if validation passed
  - `error_message`: Description of any validation errors
  - `validation_timestamp`: When validation was performed
- **Validation Rules**:
  - Retrieved chunks list must not be empty
  - Validation timestamp must be current

### ValidationReport
**Description**: Output document summarizing the results of the retrieval validation tests
- **Fields**:
  - `report_id`: Unique identifier for the report
  - `total_queries`: Number of queries tested
  - `successful_queries`: Number of queries that passed validation
  - `failed_queries`: Number of queries that failed validation
  - `success_rate`: Percentage of successful queries
  - `details`: List of ValidationResult objects
  - `execution_time`: Time taken to complete validation
- **Validation Rules**:
  - Success rate must be between 0 and 100
  - Total queries must equal successful + failed queries
  - Details list must match the number of queries tested

## Relationships

- QueryVector → (0..n) RetrievedChunk (via similarity search)
- RetrievedChunk → (1) SourceMetadata (embedded in metadata field)
- (0..n) ValidationResult → (1) ValidationReport