# Quickstart Guide: RAG Validation and Testing

## Prerequisites

- Python 3.8 or higher
- Access to Cohere API (with valid API key)
- Access to Qdrant vector database (with valid credentials)
- Git for version control

## Setup

1. **Clone the repository** (if not already done):
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Create a virtual environment**:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   # Or install specific packages needed for validation:
   pip install cohere qdrant-client python-dotenv pytest matplotlib seaborn pandas
   ```

4. **Configure environment variables**:
   Create a `.env` file in the project root with:
   ```env
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=your_qdrant_url_here
   QDRANT_API_KEY=your_qdrant_api_key_here
   ```

## Running Validation Tests

### 1. Run All Validation Tests
```bash
python -m pytest tests/ -v
```

### 2. Run Specific Validation Components

**Unit Tests:**
```bash
python -m pytest tests/unit/ -v
```

**Integration Tests:**
```bash
python -m pytest tests/integration/ -v
```

**Specific Module Tests:**
```bash
# Embedding Validation:
python -m pytest tests/unit/test_embedding_validator.py -v

# Storage Validation:
python -m pytest tests/unit/test_storage_validator.py -v

# Retrieval Validation:
python -m pytest tests/unit/test_retrieval_validator.py -v

# Error Validation:
python -m pytest tests/unit/test_error_validator.py -v

# Performance Validation:
python -m pytest tests/unit/test_performance_validator.py -v
```

### 3. Run End-to-End Validation
```bash
python -m backend.rag_validation.main --components all
```

### 4. Run Specific Validation Components
```bash
# Run only embedding and storage validation
python -m backend.rag_validation.main --components embedding storage

# Run with verbose logging
python -m backend.rag_validation.main --components all --verbose

# Run with custom output directory
python -m backend.rag_validation.main --components all --output-dir ./my-reports
```

## Custom Validation Configuration

You can customize validation parameters by creating a `validation_config.json` file:

```json
{
  "cohere_model": "embed-multilingual-v2.0",
  "qdrant_collection": "rag-embeddings",
  "validation_thresholds": {
    "embedding_success_rate": 0.99,
    "storage_success_rate": 0.99,
    "retrieval_relevance": 0.95,
    "max_latency_ms": 500,
    "min_throughput_per_sec": 10
  },
  "test_data": {
    "sample_texts": ["Your sample text for embedding validation"],
    "test_queries": ["Your test queries for retrieval validation"]
  },
  "validation_timeout": 30,
  "retry_attempts": 3
}
```

## Validation Report Output

After running validation, reports will be generated in the `validation_reports/` directory with:
- JSON reports with detailed validation results
- CSV reports for data analysis
- Summary reports with key metrics
- Visualization charts for performance analysis

## Configuration Validation and Defaults

The system includes robust configuration validation with sensible defaults:
- Validates all required API keys are present
- Ensures proper model and collection names
- Sets appropriate performance thresholds
- Applies fallback values for missing configuration

## Input Validation and Sanitization

All modules include input validation and sanitization:
- Validates API responses and handles errors gracefully
- Sanitizes text inputs before processing
- Implements proper error handling with fallbacks
- Includes retry mechanisms for transient failures

## Error Handling and Graceful Degradation

The system implements comprehensive error handling:
- Captures and logs all errors with context
- Provides detailed error information for debugging
- Implements graceful degradation when components fail
- Includes circuit breaker patterns for external service calls

## Visualization and Export Functions

Results can be exported in multiple formats:
- JSON export for programmatic consumption
- CSV export for spreadsheet analysis
- Visualization charts for trend analysis
- Summary dashboards for executive reporting

## Troubleshooting

- **API Key Issues**: Verify your Cohere and Qdrant API keys are correct and have sufficient permissions
- **Connection Issues**: Check that your Qdrant URL is accessible and properly configured
- **Rate Limits**: If encountering rate limits, add delays between validation calls or upgrade your API plan
- **Configuration Issues**: Ensure all required environment variables and config parameters are set
- **Performance Issues**: Monitor resource usage during validation runs