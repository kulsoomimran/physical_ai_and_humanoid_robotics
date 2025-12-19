"""
Integration tests for the storage validator with Qdrant.
"""
import unittest
from unittest.mock import patch, Mock
import sys

# Add backend directory to path to import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../'))

from backend.rag_validation.storage_validator import StorageValidator
from backend.rag_validation.config_loader import ConfigLoader
from backend.rag_validation.test_data import TestDataGenerator


class TestStorageIntegration(unittest.TestCase):
    """
    Integration tests for StorageValidator with Qdrant.
    """

    def setUp(self):
        """
        Set up test fixtures before each test method.
        """
        # Create a mock config loader for testing
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

    @patch('backend.rag_validation.storage_validator.qdrant_client.QdrantClient')
    def test_end_to_end_storage_validation_process(self, mock_qdrant_client):
        """
        Test the complete end-to-end storage validation process.
        """
        # Setup mock Qdrant client responses
        mock_collection_info = Mock()
        mock_collection_info.config.params.vectors.size = 768
        mock_collection_info.config.params.vectors.distance = "Cosine"
        mock_collection_info.points_count = 0

        # Setup mock client
        mock_client = Mock()
        mock_client.get_collection.return_value = mock_collection_info
        mock_client.upsert.return_value = None
        mock_point = Mock()
        mock_point.payload = {
            "source_url": "https://example.com/test",
            "page_title": "Test Page",
            "section_hierarchy": "section1",
            "created_at": "2023-01-01T00:00:00Z",
            "chunk_index": 0
        }
        mock_client.retrieve.return_value = [mock_point]
        mock_qdrant_client.return_value = mock_client

        # Create validator
        validator = StorageValidator(config_loader=self.mock_config_loader)

        # Test collection validation
        collection_result = validator.validate_collection_exists()
        self.assertTrue(collection_result["success"])

        # Test dimension validation
        dims_result = validator.validate_vector_dimensions()
        self.assertTrue(dims_result["success"])
        self.assertTrue(dims_result["valid"])

        # Test metadata schema validation
        metadata = TestDataGenerator.generate_metadata(1)[0]
        schema_result = validator.validate_metadata_schema(metadata)
        self.assertTrue(schema_result["valid"])

        # Test storing embedding with metadata
        test_data = TestDataGenerator.generate_embedding_data(1)[0]
        store_result = validator.validate_store_embedding_with_metadata(
            test_data["vector"],
            test_data["metadata"],
            test_data["id"]
        )
        self.assertTrue(store_result["success"])

        # Test retrieving metadata
        retrieval_result = validator.validate_metadata_retrieval(test_data["id"])
        self.assertTrue(retrieval_result["success"])
        self.assertIn("source_url", retrieval_result["retrieved_metadata"])

        # Test complete validation workflow
        complete_result = validator.run_complete_storage_validation()
        self.assertIn("overall_success", complete_result)

    @patch('backend.rag_validation.storage_validator.qdrant_client.QdrantClient')
    def test_storage_retrieval_consistency(self, mock_qdrant_client):
        """
        Test that stored data can be consistently retrieved.
        """
        # Setup
        mock_collection_info = Mock()
        mock_collection_info.config.params.vectors.size = 768
        mock_collection_info.config.params.vectors.distance = "Cosine"
        mock_collection_info.points_count = 0

        mock_client = Mock()
        mock_client.get_collection.return_value = mock_collection_info
        mock_client.upsert.return_value = None

        # We'll track stored data to verify retrieval
        stored_data = {}

        def mock_upsert(collection_name, points):
            for point in points:
                stored_data[point.id] = point.payload

        def mock_retrieve(collection_name, ids, with_payload=True, with_vectors=False):
            result = []
            for point_id in ids:
                if point_id in stored_data:
                    mock_point = Mock()
                    mock_point.payload = stored_data[point_id]
                    result.append(mock_point)
            return result

        mock_client.upsert.side_effect = mock_upsert
        mock_client.retrieve.side_effect = mock_retrieve
        mock_qdrant_client.return_value = mock_client

        # Create validator
        validator = StorageValidator(config_loader=self.mock_config_loader)

        # Generate and store test data
        test_data_list = TestDataGenerator.generate_embedding_data(3)

        stored_ids = []
        for test_data in test_data_list:
            result = validator.validate_store_embedding_with_metadata(
                test_data["vector"],
                test_data["metadata"],
                test_data["id"]
            )
            self.assertTrue(result["success"])
            stored_ids.append(test_data["id"])

        # Retrieve and verify each stored item
        for point_id in stored_ids:
            retrieval_result = validator.validate_metadata_retrieval(point_id)
            self.assertTrue(retrieval_result["success"])
            # Verify that retrieved metadata matches what was stored
            self.assertIsNotNone(retrieval_result["retrieved_metadata"])

    @patch('backend.rag_validation.storage_validator.qdrant_client.QdrantClient')
    def test_metadata_validation_integration(self, mock_qdrant_client):
        """
        Test integration between metadata validation and storage operations.
        """
        # Setup
        mock_collection_info = Mock()
        mock_collection_info.config.params.vectors.size = 768
        mock_collection_info.config.params.vectors.distance = "Cosine"
        mock_collection_info.points_count = 0

        mock_client = Mock()
        mock_client.get_collection.return_value = mock_collection_info
        mock_client.upsert.return_value = None

        retrieved_payloads = []

        def mock_upsert(collection_name, points):
            for point in points:
                retrieved_payloads.append(point.payload)

        mock_client.upsert.side_effect = mock_upsert
        mock_qdrant_client.return_value = mock_client

        # Create validator
        validator = StorageValidator(config_loader=self.mock_config_loader)

        # Generate test metadata
        metadata_list = TestDataGenerator.generate_metadata(2)

        for metadata in metadata_list:
            # Validate metadata schema first
            schema_result = validator.validate_metadata_schema(metadata)
            self.assertTrue(schema_result["valid"], f"Metadata schema validation failed for {metadata}")

            # Then store with this metadata
            embedding = TestDataGenerator.generate_embedding_vector(768)
            store_result = validator.validate_store_embedding_with_metadata(embedding, metadata)
            self.assertTrue(store_result["success"])

        # Verify that stored payloads match expected structure
        self.assertEqual(len(retrieved_payloads), 2)

    @patch('backend.rag_validation.storage_validator.qdrant_client.QdrantClient')
    def test_error_handling_integration(self, mock_qdrant_client):
        """
        Test error handling integration throughout the storage validation process.
        """
        # Setup mock to raise an exception during upsert
        mock_collection_info = Mock()
        mock_collection_info.config.params.vectors.size = 768
        mock_collection_info.config.params.vectors.distance = "Cosine"
        mock_collection_info.points_count = 0

        mock_client = Mock()
        mock_client.get_collection.return_value = mock_collection_info
        mock_client.upsert.side_effect = Exception("Storage Error")
        mock_qdrant_client.return_value = mock_client

        # Create validator
        validator = StorageValidator(config_loader=self.mock_config_loader)

        # Test that errors are handled gracefully during storage
        embedding = TestDataGenerator.generate_embedding_vector(768)
        metadata = TestDataGenerator.generate_metadata(1)[0]

        result = validator.validate_store_embedding_with_metadata(embedding, metadata)
        self.assertFalse(result["success"])
        self.assertIn("Storage Error", result["error"])

        # Test collection validation with connection error
        mock_client_with_error = Mock()
        mock_client_with_error.get_collection.side_effect = Exception("Connection Error")
        mock_qdrant_client.return_value = mock_client_with_error

        validator_with_error = StorageValidator(config_loader=self.mock_config_loader)
        collection_result = validator_with_error.validate_collection_exists()
        self.assertFalse(collection_result["success"])

    @patch('backend.rag_validation.storage_validator.qdrant_client.QdrantClient')
    def test_large_scale_storage_validation(self, mock_qdrant_client):
        """
        Test storage validation with larger amounts of data.
        """
        # Setup
        mock_collection_info = Mock()
        mock_collection_info.config.params.vectors.size = 768
        mock_collection_info.config.params.vectors.distance = "Cosine"
        mock_collection_info.points_count = 0

        mock_client = Mock()
        mock_client.get_collection.return_value = mock_collection_info
        mock_client.upsert.return_value = None
        mock_point = Mock()
        mock_point.payload = {"source_url": "https://example.com/test"}
        mock_client.retrieve.return_value = [mock_point]
        mock_qdrant_client.return_value = mock_client

        # Create validator
        validator = StorageValidator(config_loader=self.mock_config_loader)

        # Generate more test data
        test_data_list = TestDataGenerator.generate_embedding_data(10)  # 10 items instead of 3

        storage_results = []
        retrieval_results = []

        # Store all items
        for test_data in test_data_list:
            store_result = validator.validate_store_embedding_with_metadata(
                test_data["vector"],
                test_data["metadata"],
                test_data["id"]
            )
            storage_results.append(store_result)

        # Retrieve all items
        for test_data in test_data_list:
            retrieval_result = validator.validate_metadata_retrieval(test_data["id"])
            retrieval_results.append(retrieval_result)

        # Verify results
        success_count = sum(1 for r in storage_results if r["success"])
        self.assertEqual(success_count, len(test_data_list))

        retrieval_success_count = sum(1 for r in retrieval_results if r["success"])
        self.assertEqual(retrieval_success_count, len(test_data_list))

    def test_configuration_integration(self):
        """
        Test integration with configuration loading.
        """
        # Create a real ConfigLoader with test configuration
        config_loader = ConfigLoader()

        # Override the config loading to use test values
        original_get_config = config_loader.get_config
        def mock_get_config():
            config = original_get_config()
            config["qdrant_collection"] = "test-rag-embeddings"
            return config
        config_loader.get_config = mock_get_config

        # We can't test with real API key, so we'll just check initialization
        with patch('backend.rag_validation.storage_validator.qdrant_client.QdrantClient'):
            # Mock the API key retrieval to avoid actual API key requirement
            config_loader.get_qdrant_api_key = Mock(return_value="test-key")
            config_loader.get_qdrant_url = Mock(return_value="https://test.com")

            validator = StorageValidator(config_loader=config_loader)

            # Check that the validator was initialized with the config
            self.assertIsNotNone(validator.config_loader)
            self.assertEqual(validator.collection_name, "test-rag-embeddings")


if __name__ == '__main__':
    unittest.main()