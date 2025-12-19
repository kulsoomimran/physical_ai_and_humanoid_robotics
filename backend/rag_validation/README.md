# RAG Validation System

The RAG (Retrieval Augmented Generation) Validation System is a comprehensive suite of validation modules designed to verify the correctness, performance, and reliability of RAG pipeline components. This system ensures that Cohere embeddings, Qdrant storage, retrieval accuracy, error handling, and performance metrics meet the required standards.

## Overview

The RAG Validation System consists of several interconnected modules that validate different aspects of the RAG pipeline:

- **Embedding Validator**: Validates Cohere embedding generation from text chunks
- **Storage Validator**: Validates Qdrant storage with complete and accurate metadata
- **Retrieval Validator**: Validates similarity-based retrieval from Qdrant
- **Error Validator**: Validates error detection, logging, and handling mechanisms
- **Performance Validator**: Validates retrieval latency and performance metrics
- **Main Orchestrator**: Coordinates all validation components and produces comprehensive reports

## Features

- **Modular Architecture**: Each validation component can be run independently or as part of the full pipeline
- **Comprehensive Reporting**: Detailed validation reports in JSON, CSV, and summary formats
- **Performance Metrics**: Latency measurements, throughput calculations, and percentile analysis
- **Error Handling**: Robust error detection and graceful degradation mechanisms
- **Configuration Driven**: Flexible configuration through JSON files and environment variables
- **Visualization**: Charts and graphs for validation result analysis

## Installation

### Prerequisites

- Python 3.8 or higher
- pip package manager
- Access to Cohere API
- Access to Qdrant vector database

### Setup

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd <repository-directory>
   ```

2. Create a virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

4. Set up environment variables in `.env` file:
   ```env
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_api_key
   ```

## Usage

### Command Line Interface

The main validation orchestrator can be run from the command line:

```bash
python -m backend.rag_validation.main --components embedding storage retrieval error performance
```

#### Available Options

- `--components`: Specify which validation components to run (default: all)
  - `embedding`: Validate Cohere embedding generation
  - `storage`: Validate Qdrant storage and metadata
  - `retrieval`: Validate similarity-based retrieval
  - `error`: Validate error handling mechanisms
  - `performance`: Validate performance metrics
  - `all`: Run all validation components

- `--config-file`: Path to configuration file (optional)
- `--output-dir`: Directory to save validation reports (default: validation_reports)
- `--verbose`: Enable verbose logging
- `--test-data-file`: Path to file with test data for validation

#### Examples

Run all validation components:
```bash
python -m backend.rag_validation.main --components all
```

Run only embedding and storage validation:
```bash
python -m backend.rag_validation.main --components embedding storage
```

Run with verbose logging and custom output directory:
```bash
python -m backend.rag_validation.main --components all --verbose --output-dir ./custom-reports
```

### Programmatic Usage

You can also use the validation system programmatically:

```python
from backend.rag_validation.main import RAGValidationOrchestrator
from backend.rag_validation.config_loader import ConfigLoader

# Create configuration loader
config_loader = ConfigLoader()

# Create orchestrator
orchestrator = RAGValidationOrchestrator(config_loader)

# Run complete validation
results = orchestrator.run_complete_validation()

# Run specific components
results = orchestrator.run_complete_validation(components=['embedding', 'storage'])
```

## Configuration

The validation system uses a flexible configuration system that supports both JSON files and environment variables.

### Configuration File

Create a `validation_config.json` file to customize validation behavior:

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

### Environment Variables

The following environment variables can be set:

- `COHERE_API_KEY`: Cohere API key for embedding generation
- `QDRANT_URL`: URL of the Qdrant server
- `QDRANT_API_KEY`: API key for Qdrant authentication
- `COHERE_MODEL`: Cohere model to use for embeddings (default: embed-multilingual-v2.0)
- `QDRANT_COLLECTION`: Name of the Qdrant collection (default: rag-embeddings)

## Validation Modules

### Embedding Validator

Validates Cohere embedding generation pipeline:

- Verifies model name and dimensions
- Tests single and batch text chunk embedding
- Validates embedding dimensions and quality
- Includes embedding validation orchestrator

### Storage Validator

Validates Qdrant storage and metadata:

- Checks collection existence and configuration
- Validates vector dimensions match expectations
- Verifies metadata schema compliance
- Tests embedding storage with metadata
- Ensures metadata retrieval accuracy

### Retrieval Validator

Validates similarity-based retrieval:

- Tests similarity search functionality
- Calculates relevance scores between queries and results
- Verifies source URL mapping
- Validates content mapping accuracy
- Includes retrieval validation orchestrator

### Error Validator

Validates error handling mechanisms:

- Simulates Cohere API failures
- Tests Qdrant connection failures
- Validates malformed input handling
- Tests rate limit detection and handling
- Classifies error severity levels

### Performance Validator

Validates performance metrics:

- Measures retrieval latency
- Calculates p95/p99 latency percentiles
- Tests concurrent validation scenarios
- Measures throughput under load
- Validates performance against thresholds

## Reports and Visualization

### Validation Reports

The system generates comprehensive reports in multiple formats:

- **JSON Report**: Detailed validation results with all metrics
- **CSV Report**: Tabular data for analysis and import into spreadsheets
- **Summary Report**: High-level overview with key metrics and success rates

Reports are saved to the `validation_reports` directory by default.

### Visualization

The system includes visualization utilities for:

- Component success rates chart
- Latency distribution charts
- Validation timeline over multiple runs
- Comprehensive dashboard with multiple metrics

## Testing

The system includes comprehensive test suites:

- **Unit Tests**: Individual module functionality
- **Integration Tests**: Module interactions and workflows
- **End-to-End Tests**: Complete validation pipeline

Run tests with:
```bash
# Unit tests
python -m pytest tests/unit/

# Integration tests
python -m pytest tests/integration/

# All tests
python -m pytest tests/
```

## Architecture

The RAG Validation System follows a modular, extensible architecture:

```
rag_validation/
├── __init__.py
├── main.py                 # Main orchestrator
├── config_loader.py        # Configuration management
├── logger.py               # Logging utilities
├── error_handler.py        # Error handling utilities
├── test_data.py            # Test data generation
├── performance_utils.py    # Performance measurement
├── validation_report.py    # Report generation
├── visualization_utils.py  # Result visualization
├── embedding_validator.py  # Embedding validation
├── storage_validator.py    # Storage validation
├── retrieval_validator.py  # Retrieval validation
├── error_validator.py      # Error validation
└── performance_validator.py # Performance validation
```

## Best Practices

1. **Environment Isolation**: Use separate environments for development, testing, and production validation
2. **Configuration Management**: Use configuration files for non-sensitive settings and environment variables for sensitive data
3. **Regular Validation**: Schedule periodic validation runs to monitor system health
4. **Threshold Tuning**: Adjust validation thresholds based on your specific requirements and performance expectations
5. **Monitoring**: Set up alerts for validation failures or performance degradations

## Troubleshooting

### Common Issues

- **API Keys**: Ensure Cohere and Qdrant API keys are correctly set in environment variables
- **Network Connectivity**: Verify connectivity to Cohere and Qdrant services
- **Collection Names**: Check that Qdrant collection names match between validation and application
- **Resource Limits**: Monitor API usage limits and adjust validation frequency accordingly

### Logging

Enable verbose logging for detailed troubleshooting:
```bash
python -m backend.rag_validation.main --verbose --components all
```

### Error Handling

The system implements graceful error handling:
- Failed validations are logged but don't halt the entire process
- Configuration errors are caught and reported with suggestions
- Network timeouts and retries are handled automatically

## Contributing

Contributions to the RAG Validation System are welcome. Please follow these guidelines:

1. Fork the repository
2. Create a feature branch
3. Add your changes with appropriate tests
4. Submit a pull request with a clear description

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Support

For support, please create an issue in the repository or contact the development team.