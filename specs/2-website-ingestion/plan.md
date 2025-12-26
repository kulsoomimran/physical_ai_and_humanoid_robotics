# Implementation Plan: Website Ingestion, Embedding Generation, and Vector Storage for RAG Chatbot

**Feature**: 2-website-ingestion
**Created**: 2025-12-18
**Status**: Draft
**Author**: Claude Code Assistant

## Technical Context

This plan outlines the implementation of a RAG (Retrieval Augmented Generation) system that ingests content from published Docusaurus books, generates embeddings using Cohere, and stores them in Qdrant vector database. The system will be implemented in a single `ingestion.py` file with the following key functions:
- `get_all_urls`: Retrieve all URLs from the target Docusaurus site
- `extract_text_from_url`: Extract clean text content from each URL
- `chunk_text`: Split content into logical sections suitable for embeddings
- `embed`: Generate embeddings with Cohere
- `create_collection`: Create a Qdrant collection for storing vectors
- `save_chunk_to_qdrant`: Store embeddings with metadata in Qdrant

### Known Dependencies
- Python 3.8+
- Cohere Python SDK
- Qdrant Python client
- Requests library
- BeautifulSoup for HTML parsing
- Sentence-transformers or similar for text chunking

### Unknown Dependencies/Requirements (NEEDS CLARIFICATION)
- Specific Cohere embedding model to use
- Target Docusaurus URL for ingestion (https://ai-and-humanoid-robotics-book.vercel.app/)
- Exact chunk size requirements
- Collection name for Qdrant
- Specific metadata fields to store alongside embeddings

## Constitution Check

### Alignment Assessment
- **Practical & Hands-on**: This implementation provides a working RAG system that engineers can use to query documentation
- **Comprehensive Modules**: Enables querying of the robotics book content across all modules
- **Clear & Accessible**: Implementation will follow clean code practices with clear documentation
- **Future-Oriented**: Uses modern vector databases and embedding techniques
- **Ethical Robotics**: N/A for this backend infrastructure component

### Potential Violations
None identified - this implementation aligns with project principles.

## Gates

### Gate 1: Technical Feasibility ✓
- All required services (Cohere, Qdrant) are available and have Python SDKs
- Target website (Docusaurus) is publicly accessible
- All dependencies are standard Python packages

### Gate 2: Resource Availability ✓
- Qdrant cloud instance is already configured in .env
- Cohere API access can be configured
- Target website is published and accessible

### Gate 3: Architecture Alignment ✓
- Implementation follows single-file approach as requested
- Separation of concerns with distinct functions
- Follows ETL (Extract, Transform, Load) pattern

## Phase 0: Research

### Research Tasks Completed

#### 1. Target Website Structure Analysis
- **Decision**: The target website is a Docusaurus-based documentation site at https://ai-and-humanoid-robotics-book.vercel.app/
- **Rationale**: This is the deployed book URL provided in the requirements
- **Alternatives considered**: None provided - this is the specific URL to be ingested

#### 2. Cohere Embedding Model Selection
- **Decision**: Use Cohere's multilingual-22-12 embedding model (recommended for documentation)
- **Rationale**: Performs well on technical documentation and supports multiple languages
- **Alternatives considered**:
  - embed-english-v2.0: Good for English content but less suitable for technical docs
  - embed-multilingual-v2.0: Better for mixed-language content and technical documentation

#### 3. Text Chunking Strategy
- **Decision**: Use recursive character-based chunking with overlap
- **Rationale**: Preserves semantic coherence while managing token limits
- **Alternatives considered**:
  - Sentence-based: May create chunks that are too small
  - Fixed token length: May split semantically related content

#### 4. Qdrant Collection Configuration
- **Decision**: Create collection named "rag-embeddings" with 1024 dimensions (for Cohere embeddings)
- **Rationale**: Standard naming convention and matches Cohere's embedding dimensions
- **Alternatives considered**: Generic names like "documents" - decided on specific name for clarity

#### 5. Metadata Schema
- **Decision**: Store URL, page title, content section, and timestamp with each embedding
- **Rationale**: Provides rich context for retrieval and enables proper citation
- **Alternatives considered**: Minimal metadata - decided rich metadata improves retrieval quality

## Phase 1: Design

### Data Model

#### Document Chunk Entity
- **id**: Unique identifier for the chunk (UUID)
- **content**: The text content of the chunk
- **embedding**: Vector representation of the content
- **metadata**: Dictionary containing:
  - source_url: Original URL of the document
  - page_title: Title of the page
  - section_hierarchy: Document section/path information
  - created_at: Timestamp of ingestion
  - chunk_index: Position of chunk within original document

#### Vector Database Record
- **vector_id**: Unique identifier in Qdrant
- **vector**: The embedding vector (768 dimensions for Cohere multilingual model)
- **payload**: Contains content and metadata as defined above

### Dependencies and Configuration

#### Required Python Packages
- `cohere`: For generating text embeddings
- `qdrant-client`: For interacting with Qdrant vector database
- `requests`: For HTTP requests to fetch web content
- `beautifulsoup4`: For parsing and extracting content from HTML
- `python-dotenv`: For loading environment variables
- `urllib3`: For URL manipulation

#### Environment Variables Required
- `COHERE_API_KEY`: API key for accessing Cohere embedding services
- `QDRANT_URL`: URL endpoint for the Qdrant cloud instance
- `QDRANT_API_KEY`: API key for authenticating with Qdrant

#### Configuration Parameters
- `CHUNK_SIZE`: Maximum size of text chunks (default: 1000 characters)
- `CHUNK_OVERLAP`: Overlap between chunks to maintain context (default: 100 characters)
- `BATCH_SIZE`: Number of texts to process in each Cohere API call (default: 96)
- `COLLECTION_NAME`: Name of the Qdrant collection (default: "rag-embeddings")
- `REQUEST_TIMEOUT`: Timeout for HTTP requests (default: 10 seconds)

### API Contract

The ingestion.py file will expose a single entry point that orchestrates the entire ingestion process:

```python
def main():
    """
    Main entry point for the website ingestion pipeline
    Orchestrates: URL retrieval -> Content extraction -> Chunking -> Embedding -> Storage
    """
```

### Quickstart Guide

1. Install dependencies: `pip install cohere qdrant-client requests beautifulsoup4 python-dotenv`
2. Configure environment variables in `.env` (COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY)
3. Run the ingestion: `python ingestion.py`
4. Verify successful ingestion in Qdrant dashboard

## Phase 2: Implementation Strategy

### Function Breakdown

1. **`get_all_urls(base_url)`**
   - Input: Base URL of Docusaurus site
   - Output: List of all accessible page URLs
   - Responsibility: Crawl the site and collect all documentation URLs
   - Implementation details:
     - Use breadth-first search to discover all internal links
     - Filter out non-documentation pages (images, PDFs, external links)
     - Respect robots.txt and implement polite crawling with delays
     - Handle pagination and navigation structures specific to Docusaurus

2. **`extract_text_from_url(url)`**
   - Input: Single URL
   - Output: Clean text content from the page
   - Responsibility: Parse HTML, extract main content, remove navigation elements
   - Implementation details:
     - Use BeautifulSoup to parse HTML content
     - Target Docusaurus-specific selectors for main content areas
     - Remove navigation, headers, footers, and other non-content elements
     - Extract page title for metadata
     - Handle different page layouts and content structures

3. **`chunk_text(text_content, chunk_size=1000, overlap=100)`**
   - Input: Raw text content
   - Output: List of text chunks
   - Responsibility: Split content into semantically coherent segments
   - Implementation details:
     - Use recursive character splitting with sentence boundary awareness
     - Maintain context with overlapping chunks
     - Preserve document structure information in metadata
   - Chunking Strategy:
     - Primary method: Character-based splitting with intelligent boundary detection
     - Secondary method: Sentence-aware splitting to avoid breaking sentences
     - Chunk size: 1000 characters (configurable) to balance context and efficiency
     - Overlap: 100 characters (configurable) to maintain context across chunks
     - Boundary detection: Look for sentence endings (., !, ?, ;) near chunk boundaries
     - Fallback: Hard cutoff if no good boundary found within tolerance

4. **`embed(text_chunks)`**
   - Input: List of text chunks
   - Output: List of embedding vectors
   - Responsibility: Generate embeddings using Cohere API
   - Implementation details:
     - Process in batches to respect API limits
     - Handle rate limiting and retry logic
     - Use Cohere's multilingual embedding model for technical documentation
   - Embedding Configuration:
     - Model: embed-multilingual-v2.0 (optimized for technical documentation)
     - Vector dimensions: 768 (important for Qdrant configuration)
     - Batch size: 96 (Cohere's recommended maximum)
     - Input type: "search_document" for document retrieval use case
     - Text preprocessing: Minimal cleaning to preserve technical content
   - Rate Limiting & Error Handling:
     - Implement exponential backoff for API retries
     - Track API usage to stay within quota limits
     - Cache embeddings to avoid redundant API calls
     - Fallback mechanism for API failures (e.g., return zero vectors with logging)

5. **`create_collection(collection_name="rag-embeddings")`**
   - Input: Collection name
   - Output: Confirmation of collection creation
   - Responsibility: Initialize Qdrant collection with appropriate vector size
   - Implementation details:
     - Configure for 768-dimensional vectors (Cohere multilingual model)
     - Set cosine similarity distance function
     - Handle case where collection already exists
   - Qdrant Configuration:
     - Collection name: "rag-embeddings" (default) or configurable
     - Vector size: 768 dimensions (matching Cohere embedding output)
     - Distance function: Cosine similarity (optimal for text embeddings)
     - Additional settings: HNSW index for efficient similarity search
     - Payload storage: Enable for metadata retrieval during search
     - Sharding: Default configuration for cloud tier

6. **`save_chunk_to_qdrant(chunk_data)`**
   - Input: Chunk with content, embedding, and metadata
   - Output: Confirmation of storage
   - Responsibility: Store individual chunk in Qdrant with metadata
   - Implementation details:
     - Generate unique UUID for each record
     - Store content, embedding vector, and rich metadata
     - Implement error handling for storage failures
   - Storage Schema:
     - Point ID: UUID4 string for unique identification
     - Vector: 768-dimensional embedding vector from Cohere
     - Payload fields:
       - "content": Original text content of the chunk
       - "source_url": URL where the content was found
       - "page_title": Title of the source page
       - "section_hierarchy": Path/section information from the site structure
       - "created_at": Timestamp of ingestion
       - "chunk_index": Position of this chunk in the original document
       - "start_pos": Character position where chunk starts in original text
       - "end_pos": Character position where chunk ends in original text
   - Performance Optimization:
     - Batch upsert operations for multiple records
     - Use UUID4 for distributed ID generation
     - Implement retry logic for failed storage operations
     - Monitor collection size for cloud tier limits

### Error Handling Strategy
- Graceful degradation when individual URLs fail to load
- Retry mechanism for API calls to Cohere and Qdrant
- Logging for monitoring and debugging
- Validation of embeddings before storage

### Configuration Strategy
- Environment variables for API keys and connection strings
- Configurable parameters for chunk size, overlap, and retry attempts
- Command-line arguments for specifying target URL and collection name

## Phase 3: Execution Plan

### Step 1: Environment Setup
- Create virtual environment
- Install required dependencies
- Verify environment variables

### Step 2: URL Retrieval Module
- Implement `get_all_urls()` function
- Test with target Docusaurus site

### Step 3: Content Extraction Module
- Implement `extract_text_from_url()` function
- Handle different Docusaurus page structures
- Test with sample URLs

### Step 4: Text Processing Module
- Implement `chunk_text()` function
- Test chunking logic with sample content

### Step 5: Embedding Generation Module
- Implement `embed()` function
- Integrate with Cohere API
- Handle rate limiting and errors

### Step 6: Vector Storage Module
- Implement `create_collection()` and `save_chunk_to_qdrant()` functions
- Integrate with Qdrant
- Test storage and retrieval

### Step 7: Integration and Testing
- Combine all modules in main function
- Perform end-to-end testing
- Optimize performance and error handling

## Risk Analysis

### High-Risk Items
1. **API Rate Limits**: Cohere and Qdrant have usage limits - implement proper rate limiting
2. **Large Site Processing**: Docusaurus site may have many pages - implement batching and progress tracking
3. **Network Failures**: External requests may fail - implement retries and error recovery

### Mitigation Strategies
- Implement exponential backoff for API calls
- Add progress tracking and checkpointing
- Include comprehensive logging for debugging
- Validate data at each stage before proceeding

## Success Criteria

### Functional Acceptance
- [ ] Successfully retrieve all URLs from target Docusaurus site
- [ ] Extract clean text content from each page
- [ ] Generate embeddings without exceeding API limits
- [ ] Store all embeddings in Qdrant with proper metadata
- [ ] Maintain semantic coherence during chunking

### Performance Acceptance
- [ ] Process 100-page site within 30 minutes
- [ ] Generate 1000 embeddings within 10 minutes
- [ ] Achieve >95% success rate for content storage

### Quality Acceptance
- [ ] Proper error handling without crashes
- [ ] Comprehensive logging for monitoring
- [ ] Configurable parameters for different use cases