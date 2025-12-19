"""
Unit tests for the retrieval validator.
"""
import unittest
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# Add backend directory to path to import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../'))

from backend.rag_validation.retrieval_validator import RetrievalValidator
from backend.rag_validation.config_loader import ConfigLoader


class TestRetrievalValidator(unittest.TestCase):
    """
    Unit tests for RetrievalValidator class.
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
                "retrieval_relevance": 0.95,
                "max_latency_ms": 500
            }
        }
        self.mock_config_loader.get_qdrant_url.return_value = "https://test-qdrant.com"
        self.mock_config_loader.get_qdrant_api_key.return_value = "test-api-key"

        # Create the validator instance with mocked client
        with patch('backend.rag_validation.retrieval_validator.qdrant_client.QdrantClient'):
            self.validator = RetrievalValidator(config_loader=self.mock_config_loader)

    @patch('backend.rag_validation.retrieval_validator.qdrant_client.QdrantClient')
    def test_initialization(self, mock_qdrant_client_class):
        """
        Test that the RetrievalValidator initializes correctly.
        """
        # Setup
        mock_client = Mock()
        mock_qdrant_client_class.return_value = mock_client

        config_loader = Mock(spec=ConfigLoader)
        config_loader.get_qdrant_api_key.return_value = "test-key"
        config_loader.get_qdrant_url.return_value = "https://test.com"
        config_loader.get_config.return_value = {"qdrant_collection": "test-collection"}

        # Execute
        validator = RetrievalValidator(config_loader=config_loader)

        # Assert
        config_loader.get_qdrant_url.assert_called_once()
        config_loader.get_qdrant_api_key.assert_called_once()
        mock_qdrant_client_class.assert_called_once_with(url="https://test.com", api_key="test-key")
        self.assertEqual(validator.collection_name, "test-collection")

    @patch('backend.rag_validation.retrieval_validator.qdrant_client.QdrantClient')
    def test_validate_similarity_search_success(self, mock_qdrant_client_class):
        """
        Test similarity search validation with successful results.
        """
        # Setup
        mock_client = Mock()
        mock_hit = Mock()
        mock_hit.id = "test-id-1"
        mock_hit.score = 0.9
        mock_hit.payload = {"content": "test content", "source_url": "https://example.com"}
        mock_client.search.return_value = [mock_hit]
        mock_qdrant_client_class.return_value = mock_client

        config_loader = Mock(spec=ConfigLoader)
        config_loader.get_qdrant_api_key.return_value = "test-key"
        config_loader.get_qdrant_url.return_value = "https://test.com"
        config_loader.get_config.return_value = {"qdrant_collection": "test-collection"}

        validator = RetrievalValidator(config_loader=config_loader)

        # Execute
        query_vector = [0.1] * 768
        result = validator.validate_similarity_search(query_vector, top_k=5)

        # Assert
        self.assertTrue(result["success"])
        self.assertEqual(result["results_count"], 1)
        self.assertEqual(len(result["results"]), 1)
        mock_client.search.assert_called_once()

    @patch('backend.rag_validation.retrieval_validator.qdrant_client.QdrantClient')
    def test_validate_similarity_search_failure(self, mock_qdrant_client_class):
        """
        Test similarity search validation when search fails.
        """
        # Setup
        mock_client = Mock()
        mock_client.search.side_effect = Exception("Search failed")
        mock_qdrant_client_class.return_value = mock_client

        config_loader = Mock(spec=ConfigLoader)
        config_loader.get_qdrant_api_key.return_value = "test-key"
        config_loader.get_qdrant_url.return_value = "https://test.com"
        config_loader.get_config.return_value = {"qdrant_collection": "test-collection"}

        validator = RetrievalValidator(config_loader=config_loader)

        # Execute
        query_vector = [0.1] * 768
        result = validator.validate_similarity_search(query_vector, top_k=5)

        # Assert
        self.assertFalse(result["success"])
        self.assertIn("Search failed", result["error"])

    @patch('backend.rag_validation.retrieval_validator.qdrant_client.QdrantClient')
    def test_validate_similarity_search_empty_results(self, mock_qdrant_client_class):
        """
        Test similarity search validation with empty results.
        """
        # Setup
        mock_client = Mock()
        mock_client.search.return_value = []  # Empty results
        mock_qdrant_client_class.return_value = mock_client

        config_loader = Mock(spec=ConfigLoader)
        config_loader.get_qdrant_api_key.return_value = "test-key"
        config_loader.get_qdrant_url.return_value = "https://test.com"
        config_loader.get_config.return_value = {"qdrant_collection": "test-collection"}

        validator = RetrievalValidator(config_loader=config_loader)

        # Execute
        query_vector = [0.1] * 768
        result = validator.validate_similarity_search(query_vector, top_k=5)

        # Assert
        self.assertFalse(result["success"])
        self.assertIn("No results", result["error"])

    def test_calculate_relevance_score_perfect_match(self):
        """
        Test relevance scoring with perfect content match.
        """
        # Execute
        query = "artificial intelligence"
        content = "This is about artificial intelligence and its applications"
        score = self.validator.calculate_relevance_score(query, content)

        # Assert
        self.assertGreater(score, 0)  # Should have some relevance
        self.assertLessEqual(score, 1)  # Should not exceed 1

    def test_calculate_relevance_score_no_match(self):
        """
        Test relevance scoring with no content match.
        """
        # Execute
        query = "completely different topic"
        content = "This is about artificial intelligence"
        score = self.validator.calculate_relevance_score(query, content)

        # Should have low relevance but not necessarily 0
        self.assertGreaterEqual(score, 0)
        self.assertLessEqual(score, 1)

    def test_calculate_relevance_score_empty_query(self):
        """
        Test relevance scoring with empty query.
        """
        # Execute
        score = self.validator.calculate_relevance_score("", "some content")

        # Assert
        self.assertEqual(score, 0.0)

    @patch('backend.rag_validation.retrieval_validator.qdrant_client.QdrantClient')
    def test_validate_source_url_mapping_success(self, mock_qdrant_client_class):
        """
        Test source URL mapping validation with successful match.
        """
        # Setup
        mock_client = Mock()
        mock_hit = Mock()
        mock_hit.id = "test-id-1"
        mock_hit.score = 0.9
        mock_hit.payload = {"content": "test content", "source_url": "https://expected.com"}
        mock_client.search.return_value = [mock_hit]
        mock_qdrant_client_class.return_value = mock_client

        config_loader = Mock(spec=ConfigLoader)
        config_loader.get_qdrant_api_key.return_value = "test-key"
        config_loader.get_qdrant_url.return_value = "https://test.com"
        config_loader.get_config.return_value = {"qdrant_collection": "test-collection"}

        validator = RetrievalValidator(config_loader=config_loader)

        # Execute
        result = validator.validate_source_url_mapping(
            query="test query",
            expected_url="https://expected.com",
            top_k=5
        )

        # Assert
        self.assertTrue(result["success"])
        self.assertTrue(result["found_in_results"])
        self.assertEqual(result["found_at_position"], 0)

    @patch('backend.rag_validation.retrieval_validator.qdrant_client.QdrantClient')
    def test_validate_source_url_mapping_not_found(self, mock_qdrant_client_class):
        """
        Test source URL mapping validation when expected URL is not found.
        """
        # Setup
        mock_client = Mock()
        mock_hit = Mock()
        mock_hit.id = "test-id-1"
        mock_hit.score = 0.9
        mock_hit.payload = {"content": "test content", "source_url": "https://different.com"}
        mock_client.search.return_value = [mock_hit]
        mock_qdrant_client_class.return_value = mock_client

        config_loader = Mock(spec=ConfigLoader)
        config_loader.get_qdrant_api_key.return_value = "test-key"
        config_loader.get_qdrant_url.return_value = "https://test.com"
        config_loader.get_config.return_value = {"qdrant_collection": "test-collection"}

        validator = RetrievalValidator(config_loader=config_loader)

        # Execute
        result = validator.validate_source_url_mapping(
            query="test query",
            expected_url="https://expected.com",
            top_k=5
        )

        # Assert
        self.assertTrue(result["success"])  # Operation succeeded, but URL not found
        self.assertFalse(result["found_in_results"])
        self.assertIsNone(result["found_at_position"])

    @patch('backend.rag_validation.retrieval_validator.qdrant_client.QdrantClient')
    def test_validate_content_mapping_accuracy(self, mock_qdrant_client_class):
        """
        Test content mapping accuracy validation.
        """
        # Setup
        mock_client = Mock()
        mock_hit = Mock()
        mock_hit.id = "test-id-1"
        mock_hit.score = 0.9
        mock_hit.payload = {"content": "This is the expected content", "source_url": "https://example.com"}
        mock_client.search.return_value = [mock_hit]
        mock_qdrant_client_class.return_value = mock_client

        config_loader = Mock(spec=ConfigLoader)
        config_loader.get_qdrant_api_key.return_value = "test-key"
        config_loader.get_qdrant_url.return_value = "https://test.com"
        config_loader.get_config.return_value = {"qdrant_collection": "test-collection"}

        validator = RetrievalValidator(config_loader=config_loader)

        # Execute
        result = validator.validate_content_mapping_accuracy(
            query="information about expected content",
            expected_content="This is the expected content",
            top_k=5
        )

        # Assert
        self.assertTrue(result["success"])
        self.assertEqual(result["total_results"], 1)
        self.assertGreaterEqual(result["max_relevance_score"], 0)

    @patch('backend.rag_validation.retrieval_validator.qdrant_client.QdrantClient')
    def test_run_complete_retrieval_validation(self, mock_qdrant_client_class):
        """
        Test the complete retrieval validation workflow.
        """
        # Setup
        mock_client = Mock()
        mock_hit = Mock()
        mock_hit.id = "test-id-1"
        mock_hit.score = 0.9
        mock_hit.payload = {"content": "test content", "source_url": "https://example.com"}
        mock_client.search.return_value = [mock_hit]
        mock_qdrant_client_class.return_value = mock_client

        config_loader = Mock(spec=ConfigLoader)
        config_loader.get_qdrant_api_key.return_value = "test-key"
        config_loader.get_qdrant_url.return_value = "https://test.com"
        config_loader.get_config.return_value = {"qdrant_collection": "test-collection"}

        validator = RetrievalValidator(config_loader=config_loader)

        # Execute
        test_queries = [
            {
                "query": "test query 1",
                "expected_content": "test content",
                "expected_url": "https://example.com"
            }
        ]
        result = validator.run_complete_retrieval_validation(test_queries)

        # Assert
        self.assertIn("overall_success", result)
        self.assertIn("similarity_search_results", result)
        self.assertIn("source_mapping_results", result)
        self.assertIn("content_accuracy_results", result)


if __name__ == '__main__':
    unittest.main()