# RAG Chatbot Backend - Website Ingestion Pipeline

This repository contains the backend implementation for a RAG (Retrieval Augmented Generation) chatbot that ingests content from Docusaurus websites, generates embeddings, and stores them in a vector database for semantic search.

## Features

- **Docusaurus Site Crawling**: Automatically discovers and extracts content from Docusaurus-based documentation sites
- **Content Extraction**: Extracts clean text content while preserving document structure and metadata
- **Intelligent Chunking**: Splits documents into semantically coherent chunks with overlap for context preservation
- **Embedding Generation**: Uses Cohere's multilingual embedding model to generate high-quality vector representations
- **Vector Storage**: Stores embeddings in Qdrant vector database with rich metadata
- **Progress Tracking**: Provides progress indicators and ETA for long-running operations
- **Error Handling**: Comprehensive error handling with retry logic and fallback mechanisms
- **Configurable**: Command-line arguments and environment variables for flexible configuration

## Prerequisites

- Python 3.8+
- Cohere API key
- Qdrant Cloud account and API credentials

## Installation

1. Clone the repository
2. Create a virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```
3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

## Configuration

Create a `.env` file in the backend directory with the following variables:

```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
CHUNK_SIZE=1000
CHUNK_OVERLAP=100
BATCH_SIZE=96
COLLECTION_NAME=rag_chatbot
REQUEST_TIMEOUT=10
```

## Usage

### Command Line Interface

The pipeline supports various command-line options:

```bash
# Process a single URL
python main.py --url "https://example.com/page"

# Crawl and process an entire Docusaurus site
python main.py --base-url "https://docusaurus-site.com"

# Process with custom configuration
python main.py --base-url "https://docusaurus-site.com" --chunk-size 1500 --overlap 200 --batch-size 5

# Enable verbose logging
python main.py --base-url "https://docusaurus-site.com" --verbose
```

### Available Options

- `--url`: Process a single URL
- `--base-url`: Crawl and process an entire site
- `--collection-name`: Qdrant collection name (default: rag_chatbot)
- `--chunk-size`: Size of text chunks (default: 1000)
- `--overlap`: Overlap between chunks (default: 100)
- `--batch-size`: Batch size for processing (default: 96)
- `--timeout`: Request timeout in seconds (default: 10)
- `--verbose`: Enable verbose logging

## Architecture

The ingestion pipeline consists of the following components:

1. **URL Discovery**: Crawls Docusaurus sites to discover all documentation pages
2. **Content Extraction**: Extracts clean text content while preserving titles and structure
3. **Text Chunking**: Splits documents into appropriately sized chunks with overlap
4. **Embedding Generation**: Creates vector representations using Cohere's API
5. **Vector Storage**: Stores embeddings in Qdrant with metadata for retrieval

## API

The `WebsiteIngestionPipeline` class provides the following methods:

### `get_all_urls(base_url)`
Crawls a Docusaurus site and returns all discovered URLs.

### `extract_text_from_url(url)`
Extracts clean text content from a single URL.

### `chunk_text(text_content, chunk_size=None, overlap=None)`
Splits text content into semantically coherent chunks.

### `embed(text_chunks)`
Generates embeddings for text chunks using Cohere's API.

### `create_collection(collection_name=None)`
Creates a Qdrant collection for storing embeddings.

### `save_chunk_to_qdrant(chunk_data, embedding, collection_name=None)`
Saves a chunk with its embedding to Qdrant.

### `process_document_with_progress(urls, progress_callback=None)`
Processes a list of URLs with progress tracking and ETA.

### `batch_process_documents(urls, batch_size=10)`
Processes documents in optimized batches for performance.

## Error Handling

The system includes comprehensive error handling:

- Network request timeouts and retries with exponential backoff
- Fallback mechanisms for API failures
- Validation of input data and configurations
- Graceful degradation when individual URLs fail

## Performance

The system is optimized to process 100 pages within 30 minutes, with:

- Batch processing of API calls
- Efficient memory usage during document processing
- Parallel processing where possible
- Progress tracking and ETA estimation

## Troubleshooting

### Common Issues

- **API Rate Limits**: The system includes retry logic with exponential backoff to handle API rate limits
- **Large Documents**: Very large documents are handled by the chunking algorithm
- **Network Issues**: Comprehensive retry mechanisms handle temporary network issues

### Logging

The system provides detailed logging at different levels. Use the `--verbose` flag for debug-level logging.

## Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| COHERE_API_KEY | Cohere API key for embedding generation | required |
| QDRANT_URL | Qdrant cloud instance URL | required |
| QDRANT_API_KEY | Qdrant API key for authentication | required |
| CHUNK_SIZE | Size of text chunks | 1000 |
| CHUNK_OVERLAP | Overlap between chunks | 100 |
| BATCH_SIZE | Batch size for API calls | 96 |
| COLLECTION_NAME | Qdrant collection name | rag_chatbot |
| REQUEST_TIMEOUT | Request timeout in seconds | 10 |