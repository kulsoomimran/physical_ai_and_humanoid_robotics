"""
Unit tests for the storage validator.
"""
import unittest
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# Add backend directory to path to import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../'))

from backend.rag_validation.storage_validator import StorageValidator
from backend.rag_validation.config_loader import ConfigLoader


class TestStorageValidator(unittest.TestCase):
    """
    Unit tests for StorageValidator class.
    """

    def setUp(self):
        """
        Set up test fixtures before each test method.
        """
        # Create a mock config loader
        self.mock_config_loader = Mock(spec=ConfigLoader)
        self.mock_config_loader.get_config.return_value = {
            "qdrant_collection": "test-rag-embeddings",
            "validation_thresholds": {
                "embedding_success_rate": 0.99,
                "storage_success_rate": 0.99,
                "retrieval_relevance": 0.95,
                "max_latency_ms": 500
            }
        }
        self.mock_config_loader.get_qdrant_url.return_value = "https://test-qdrant.com"
        self.mock_config_loader.get_qdrant_api_key.return_value = "test-api-key"

        # Create the validator instance with mocked client
        with patch('backend.rag_validation.storage_validator.qdrant_client.QdrantClient'):
            self.validator = StorageValidator(config_loader=self.mock_config_loader)

    @patch('backend.rag_validation.storage_validator.qdrant_client.QdrantClient')
    def test_initialization(self, mock_qdrant_client_class):
        """
        Test that the StorageValidator initializes correctly.
        """
        # Setup
        mock_client = Mock()
        mock_qdrant_client_class.return_value = mock_client

        config_loader = Mock(spec=ConfigLoader)
        config_loader.get_qdrant_api_key.return_value = "test-key"
        config_loader.get_qdrant_url.return_value = "https://test.com"
        config_loader.get_config.return_value = {"qdrant_collection": "test-collection"}

        # Execute
        validator = StorageValidator(config_loader=config_loader)

        # Assert
        config_loader.get_qdrant_url.assert_called_once()
        config_loader.get_qdrant_api_key.assert_called_once()
        mock_qdrant_client_class.assert_called_once_with(url="https://test.com", api_key="test-key")
        self.assertEqual(validator.collection_name, "test-collection")

    @patch('backend.rag_validation.storage_validator.qdrant_client.QdrantClient')
    def test_validate_collection_exists_success(self, mock_qdrant_client_class):
        """
        Test collection validation when collection exists.
        """
        # Setup
        mock_client = Mock()
        mock_collection_info = Mock()
        mock_collection_info.config.params.vectors.size = 768
        mock_collection_info.config.params.vectors.distance = "Cosine"
        mock_collection_info.points_count = 100
        mock_client.get_collection.return_value = mock_collection_info
        mock_qdrant_client_class.return_value = mock_client

        config_loader = Mock(spec=ConfigLoader)
        config_loader.get_qdrant_api_key.return_value = "test-key"
        config_loader.get_qdrant_url.return_value = "https://test.com"
        config_loader.get_config.return_value = {"qdrant_collection": "test-collection"}

        validator = StorageValidator(config_loader=config_loader)

        # Execute
        result = validator.validate_collection_exists()

        # Assert
        self.assertTrue(result["success"])
        self.assertTrue(result["collection_exists"])
        self.assertEqual(result["vector_size"], 768)
        self.assertEqual(result["point_count"], 100)

    @patch('backend.rag_validation.storage_validator.qdrant_client.QdrantClient')
    def test_validate_collection_exists_failure(self, mock_qdrant_client_class):
        """
        Test collection validation when collection does not exist.
        """
        # Setup
        mock_client = Mock()
        mock_client.get_collection.side_effect = Exception("Collection not found")
        mock_qdrant_client_class.return_value = mock_client

        config_loader = Mock(spec=ConfigLoader)
        config_loader.get_qdrant_api_key.return_value = "test-key"
        config_loader.get_qdrant_url.return_value = "https://test.com"
        config_loader.get_config.return_value = {"qdrant_collection": "test-collection"}

        validator = StorageValidator(config_loader=config_loader)

        # Execute
        result = validator.validate_collection_exists()

        # Assert
        self.assertFalse(result["success"])
        self.assertFalse(result["collection_exists"])

    @patch('backend.rag_validation.storage_validator.qdrant_client.QdrantClient')
    def test_validate_vector_dimensions_correct(self, mock_qdrant_client_class):
        """
        Test vector dimension validation with correct dimensions.
        """
        # Setup
        mock_client = Mock()
        mock_collection_info = Mock()
        mock_collection_info.config.params.vectors.size = 768
        mock_collection_info.config.params.vectors.distance = "Cosine"
        mock_client.get_collection.return_value = mock_collection_info
        mock_qdrant_client_class.return_value = mock_client

        config_loader = Mock(spec=ConfigLoader)
        config_loader.get_qdrant_api_key.return_value = "test-key"
        config_loader.get_qdrant_url.return_value = "https://test.com"
        config_loader.get_config.return_value = {"qdrant_collection": "test-collection"}

        validator = StorageValidator(config_loader=config_loader)

        # Execute
        result = validator.validate_vector_dimensions(expected_dims=768)

        # Assert
        self.assertTrue(result["success"])
        self.assertTrue(result["valid"])
        self.assertEqual(result["expected_dimensions"], 768)
        self.assertEqual(result["actual_dimensions"], 768)

    @patch('backend.rag_validation.storage_validator.qdrant_client.QdrantClient')
    def test_validate_vector_dimensions_incorrect(self, mock_qdrant_client_class):
        """
        Test vector dimension validation with incorrect dimensions.
        """
        # Setup
        mock_client = Mock()
        mock_collection_info = Mock()
        mock_collection_info.config.params.vectors.size = 512
        mock_collection_info.config.params.vectors.distance = "Cosine"
        mock_client.get_collection.return_value = mock_collection_info
        mock_qdrant_client_class.return_value = mock_client

        config_loader = Mock(spec=ConfigLoader)
        config_loader.get_qdrant_api_key.return_value = "test-key"
        config_loader.get_qdrant_url.return_value = "https://test.com"
        config_loader.get_config.return_value = {"qdrant_collection": "test-collection"}

        validator = StorageValidator(config_loader=config_loader)

        # Execute
        result = validator.validate_vector_dimensions(expected_dims=768)

        # Assert
        self.assertTrue(result["success"])  # API call succeeded
        self.assertFalse(result["valid"])  # But dimensions don't match
        self.assertEqual(result["expected_dimensions"], 768)
        self.assertEqual(result["actual_dimensions"], 512)

    def test_validate_metadata_schema_valid(self):
        """
        Test metadata schema validation with valid metadata.
        """
        # Setup
        valid_metadata = {
            "source_url": "https://example.com/doc1",
            "page_title": "Example Document",
            "section_hierarchy": "section1/subsection1",
            "created_at": "2023-01-01T00:00:00Z",
            "chunk_index": 0
        }

        # Execute
        result = self.validator.validate_metadata_schema(valid_metadata)

        # Assert
        self.assertTrue(result["valid"])
        self.assertEqual(len(result["missing_fields"]), 0)

    def test_validate_metadata_schema_invalid(self):
        """
        Test metadata schema validation with invalid metadata.
        """
        # Setup
        invalid_metadata = {
            "source_url": "https://example.com/doc1",
            # Missing required fields
        }

        # Execute
        result = self.validator.validate_metadata_schema(invalid_metadata)

        # Assert
        self.assertFalse(result["valid"])
        self.assertGreater(len(result["missing_fields"]), 0)
        # Should be missing: page_title, section_hierarchy, created_at, chunk_index
        self.assertIn("page_title", result["missing_fields"])

    @patch('backend.rag_validation.storage_validator.qdrant_client.QdrantClient')
    def test_validate_store_embedding_success(self, mock_qdrant_client_class):
        """
        Test storing embedding with metadata successfully.
        """
        # Setup
        mock_client = Mock()
        mock_client.upsert.return_value = None  # Successful upsert returns None
        mock_qdrant_client_class.return_value = mock_client

        config_loader = Mock(spec=ConfigLoader)
        config_loader.get_qdrant_api_key.return_value = "test-key"
        config_loader.get_qdrant_url.return_value = "https://test.com"
        config_loader.get_config.return_value = {"qdrant_collection": "test-collection"}

        validator = StorageValidator(config_loader=config_loader)

        # Execute
        embedding = [0.1] * 768
        metadata = {
            "source_url": "https://example.com/doc1",
            "page_title": "Example Document"
        }
        result = validator.validate_store_embedding_with_metadata(embedding, metadata, "test-id")

        # Assert
        self.assertTrue(result["success"])
        self.assertEqual(result["point_id"], "test-id")
        self.assertEqual(result["embedding_length"], 768)
        mock_client.upsert.assert_called_once()

    @patch('backend.rag_validation.storage_validator.qdrant_client.QdrantClient')
    def test_validate_store_embedding_failure(self, mock_qdrant_client_class):
        """
        Test storing embedding with metadata failing.
        """
        # Setup
        mock_client = Mock()
        mock_client.upsert.side_effect = Exception("Storage failed")
        mock_qdrant_client_class.return_value = mock_client

        config_loader = Mock(spec=ConfigLoader)
        config_loader.get_qdrant_api_key.return_value = "test-key"
        config_loader.get_qdrant_url.return_value = "https://test.com"
        config_loader.get_config.return_value = {"qdrant_collection": "test-collection"}

        validator = StorageValidator(config_loader=config_loader)

        # Execute
        embedding = [0.1] * 768
        metadata = {
            "source_url": "https://example.com/doc1",
            "page_title": "Example Document"
        }
        result = validator.validate_store_embedding_with_metadata(embedding, metadata, "test-id")

        # Assert
        self.assertFalse(result["success"])
        self.assertIn("Storage failed", result["error"])

    @patch('backend.rag_validation.storage_validator.qdrant_client.QdrantClient')
    def test_validate_metadata_retrieval_success(self, mock_qdrant_client_class):
        """
        Test metadata retrieval validation successfully.
        """
        # Setup
        mock_client = Mock()
        mock_point = Mock()
        mock_point.payload = {
            "source_url": "https://example.com/doc1",
            "page_title": "Example Document"
        }
        mock_client.retrieve.return_value = [mock_point]
        mock_qdrant_client_class.return_value = mock_client

        config_loader = Mock(spec=ConfigLoader)
        config_loader.get_qdrant_api_key.return_value = "test-key"
        config_loader.get_qdrant_url.return_value = "https://test.com"
        config_loader.get_config.return_value = {"qdrant_collection": "test-collection"}

        validator = StorageValidator(config_loader=config_loader)

        # Execute
        result = validator.validate_metadata_retrieval("test-id")

        # Assert
        self.assertTrue(result["success"])
        self.assertEqual(result["point_id"], "test-id")
        self.assertIn("source_url", result["retrieved_metadata"])
        self.assertIn("page_title", result["retrieved_metadata"])

    @patch('backend.rag_validation.storage_validator.qdrant_client.QdrantClient')
    def test_validate_metadata_retrieval_failure(self, mock_qdrant_client_class):
        """
        Test metadata retrieval validation failing.
        """
        # Setup
        mock_client = Mock()
        mock_client.retrieve.return_value = []  # No points found
        mock_qdrant_client_class.return_value = mock_client

        config_loader = Mock(spec=ConfigLoader)
        config_loader.get_qdrant_api_key.return_value = "test-key"
        config_loader.get_qdrant_url.return_value = "https://test.com"
        config_loader.get_config.return_value = {"qdrant_collection": "test-collection"}

        validator = StorageValidator(config_loader=config_loader)

        # Execute
        result = validator.validate_metadata_retrieval("test-id")

        # Assert
        self.assertFalse(result["success"])
        self.assertIn("No point found", result["error"])

    @patch('backend.rag_validation.storage_validator.qdrant_client.QdrantClient')
    def test_run_complete_storage_validation(self, mock_qdrant_client_class):
        """
        Test the complete storage validation workflow.
        """
        # Setup
        mock_client = Mock()
        # Setup collection info
        mock_collection_info = Mock()
        mock_collection_info.config.params.vectors.size = 768
        mock_collection_info.config.params.vectors.distance = "Cosine"
        mock_collection_info.points_count = 0
        mock_client.get_collection.return_value = mock_collection_info
        # Setup upsert
        mock_client.upsert.return_value = None
        # Setup retrieve
        mock_point = Mock()
        mock_point.payload = {"source_url": "https://example.com/doc1"}
        mock_client.retrieve.return_value = [mock_point]
        mock_qdrant_client_class.return_value = mock_client

        config_loader = Mock(spec=ConfigLoader)
        config_loader.get_qdrant_api_key.return_value = "test-key"
        config_loader.get_qdrant_url.return_value = "https://test.com"
        config_loader.get_config.return_value = {"qdrant_collection": "test-collection"}

        validator = StorageValidator(config_loader=config_loader)

        # Execute
        result = validator.run_complete_storage_validation()

        # Assert
        self.assertIn("collection_validation", result)
        self.assertIn("dimension_validation", result)
        self.assertIn("overall_success", result)
        # The overall success depends on the mocked behavior
        overall = result["overall_success"]
        self.assertIsNotNone(overall)


if __name__ == '__main__':
    unittest.main()