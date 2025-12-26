#!/usr/bin/env python3
"""
Unit tests for the RAG retrieval validation functions
"""

import pytest
import os
from unittest.mock import Mock, patch
from backend.retrieve import (
    Config,
    QueryVector,
    RetrievedChunk,
    SourceMetadata,
    ValidationResult,
    ValidationReport,
    get_qdrant_client,
    load_collection,
    generate_query_embedding,
    validate_retrieved_chunks,
    validate_source_url_and_document_id
)


def test_config_defaults():
    """Test that Config class loads environment variables properly"""
    # Config class should load from environment variables
    assert hasattr(Config, 'QDRANT_HOST')
    assert hasattr(Config, 'QDRANT_PORT')
    assert hasattr(Config, 'COHERE_API_KEY')


def test_query_vector_creation():
    """Test QueryVector model creation and validation"""
    # Valid query vector
    query_vector = QueryVector("test_id", [0.1, 0.2, 0.3], "test query")
    assert query_vector.id == "test_id"
    assert query_vector.text == "test query"
    assert query_vector.vector == [0.1, 0.2, 0.3]

    # Test validation - empty text should raise error
    with pytest.raises(ValueError, match="Text must not be empty"):
        QueryVector("test_id", [0.1, 0.2, 0.3], "")

    # Test validation - empty vector should raise error
    with pytest.raises(ValueError, match="Vector must not be empty"):
        QueryVector("test_id", [], "test query")


def test_retrieved_chunk_creation():
    """Test RetrievedChunk model creation and validation"""
    # Valid retrieved chunk
    chunk = RetrievedChunk(
        "chunk_id",
        "test text",
        0.8,
        "vector_id",
        {"source_url": "http://example.com"}
    )
    assert chunk.id == "chunk_id"
    assert chunk.text == "test text"
    assert chunk.score == 0.8
    assert chunk.vector_id == "vector_id"

    # Test validation - empty text should raise error
    with pytest.raises(ValueError, match="Text must not be empty"):
        RetrievedChunk(
            "chunk_id",
            "",
            0.8,
            "vector_id",
            {"source_url": "http://example.com"}
        )

    # Test validation - invalid score should raise error
    with pytest.raises(ValueError, match="Score must be between 0 and 1"):
        RetrievedChunk(
            "chunk_id",
            "test text",
            1.5,
            "vector_id",
            {"source_url": "http://example.com"}
        )

    # Test validation - empty metadata should raise error
    with pytest.raises(ValueError, match="Metadata must contain required source information"):
        RetrievedChunk("chunk_id", "test text", 0.8, "vector_id", {})


def test_source_metadata_creation():
    """Test SourceMetadata model creation and validation"""
    # Valid source metadata
    metadata = SourceMetadata(
        "https://example.com",
        "doc_123",
        1,
        page_number=5,
        section_title="Introduction"
    )
    assert metadata.source_url == "https://example.com"
    assert metadata.document_id == "doc_123"
    assert metadata.chunk_index == 1

    # Test validation - invalid URL should raise error
    with pytest.raises(ValueError, match="Source URL must be a valid URL"):
        SourceMetadata("invalid_url", "doc_123", 1)

    # Test validation - empty document ID should raise error
    with pytest.raises(ValueError, match="Document ID must exist and be non-empty"):
        SourceMetadata("https://example.com", "", 1)


def test_validation_result_creation():
    """Test ValidationResult model creation and validation"""
    # Valid validation result with empty chunks list (invalid case)
    result = ValidationResult("query_1", [], False, "Test error")
    assert result.query_id == "query_1"
    assert result.is_valid == False
    assert result.error_message == "Test error"

    # Valid validation result with chunks (valid case)
    chunk = RetrievedChunk(
        "chunk_id",
        "test text",
        0.8,
        "vector_id",
        {"source_url": "http://example.com"}
    )
    result = ValidationResult("query_1", [chunk], True)
    assert result.query_id == "query_1"
    assert result.is_valid == True
    assert len(result.retrieved_chunks) == 1


def test_validation_report_creation():
    """Test ValidationReport model creation and validation"""
    chunk = RetrievedChunk(
        "chunk_id",
        "test text",
        0.8,
        "vector_id",
        {"source_url": "http://example.com"}
    )
    validation_result = ValidationResult("query_1", [chunk], True)

    # Valid validation report
    report = ValidationReport(
        "report_1",
        total_queries=1,
        successful_queries=1,
        failed_queries=0,
        details=[validation_result]
    )
    assert report.report_id == "report_1"
    assert report.total_queries == 1
    assert report.successful_queries == 1
    assert report.failed_queries == 0
    assert report.success_rate == 100.0
    assert len(report.details) == 1

    # Test validation - invalid success rate calculation
    with pytest.raises(ValueError, match="Total queries must equal successful \\+ failed queries"):
        ValidationReport(
            "report_1",
            total_queries=1,
            successful_queries=1,
            failed_queries=1,  # This makes total != successful + failed
            details=[validation_result]
        )


def test_validate_retrieved_chunks():
    """Test the validate_retrieved_chunks function"""
    # Empty list should return False
    assert validate_retrieved_chunks([]) == False

    # List with chunks that have text should return True
    chunks = [{"text": "valid text"}, {"text": "another text"}]
    assert validate_retrieved_chunks(chunks) == True

    # List with chunks that have empty text should return False
    chunks = [{"text": ""}, {"text": "valid text"}]
    assert validate_retrieved_chunks(chunks) == True  # Because one has valid text

    chunks = [{"text": ""}, {"text": ""}]
    assert validate_retrieved_chunks(chunks) == False


def test_validate_source_url_and_document_id():
    """Test the validate_source_url_and_document_id function"""
    # Valid metadata should return True
    metadata = {
        "source_url": "https://example.com",
        "document_id": "doc_123"
    }
    assert validate_source_url_and_document_id(metadata) == True

    # Invalid URL should return False
    metadata = {
        "source_url": "invalid_url",
        "document_id": "doc_123"
    }
    assert validate_source_url_and_document_id(metadata) == False

    # Empty document ID should return False
    metadata = {
        "source_url": "https://example.com",
        "document_id": ""
    }
    assert validate_source_url_and_document_id(metadata) == False


if __name__ == "__main__":
    pytest.main([__file__])