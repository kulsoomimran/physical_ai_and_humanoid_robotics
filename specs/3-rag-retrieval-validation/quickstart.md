# Quickstart: RAG Retrieval Validation

## Prerequisites

- Python 3.11 or higher
- Access to Qdrant vector database with existing embeddings
- Cohere API key for query embedding
- pip package manager

## Setup

1. **Install Dependencies**
   ```bash
   pip install qdrant-client cohere
   ```

2. **Set Environment Variables**
   ```bash
   export COHERE_API_KEY="your-cohere-api-key"
   export QDRANT_HOST="your-qdrant-host"
   export QDRANT_PORT="6333"  # Default Qdrant port
   ```

3. **Verify Qdrant Connection**
   Make sure your Qdrant instance is running and contains the expected vector collections with embeddings from the book content.

## Running the Validation Script

1. **Basic Validation**
   ```bash
   python backend/retrieve.py --query "What is humanoid robotics?" --top-k 5
   ```

2. **Full Validation Suite**
   ```bash
   python backend/retrieve.py --validate-all
   ```

3. **Custom Parameters**
   ```bash
   python backend/retrieve.py --query "Explain ROS 2 architecture" --top-k 3 --collection "book_chunks"
   ```

## Expected Output

The script will return:
- Retrieved text chunks with similarity scores
- Source metadata (URLs, document IDs)
- Validation results indicating accuracy
- Performance metrics (query time, success rate)

## Troubleshooting

- **Connection Issues**: Verify QDRANT_HOST and port settings
- **Empty Results**: Check that the collection contains vectors and the query is well-formed
- **API Errors**: Ensure COHERE_API_KEY is set correctly