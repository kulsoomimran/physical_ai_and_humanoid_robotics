#!/usr/bin/env python3
"""
Integration tests for Qdrant connection in the RAG retrieval validation
"""

import pytest
import os
from unittest.mock import Mock, patch
from backend.retrieve import get_qdrant_client, load_collection, Config


def test_get_qdrant_client():
    """Test that get_qdrant_client returns a valid client instance"""
    # Mock the QdrantClient to avoid actual connection
    with patch('backend.retrieve.QdrantClient') as mock_client:
        mock_instance = Mock()
        mock_client.return_value = mock_instance

        client = get_qdrant_client()

        # Verify that QdrantClient was called with the right parameters
        mock_client.assert_called_once_with(
            host=Config.QDRANT_HOST,
            port=Config.QDRANT_PORT
        )
        assert client == mock_instance


def test_load_collection():
    """Test that load_collection works correctly"""
    # Mock the client and collection info
    mock_client = Mock()
    mock_collection_info = Mock()
    mock_collection_info.points_count = 100
    mock_client.get_collection.return_value = mock_collection_info

    # Test successful collection loading
    result = load_collection(mock_client, "test_collection")
    assert result == True
    mock_client.get_collection.assert_called_once_with("test_collection")

    # Test failed collection loading
    mock_client.get_collection.side_effect = Exception("Collection not found")
    result = load_collection(mock_client, "nonexistent_collection")
    assert result == False


if __name__ == "__main__":
    pytest.main([__file__])