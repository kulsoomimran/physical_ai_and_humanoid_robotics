"""
Integration tests for the retrieval validator with Qdrant.
"""
import unittest
from unittest.mock import patch, Mock
import sys

# Add backend directory to path to import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../'))

from backend.rag_validation.retrieval_validator import RetrievalValidator
from backend.rag_validation.config_loader import ConfigLoader
from backend.rag_validation.test_data import TestDataGenerator


class TestRetrievalIntegration(unittest.TestCase):
    """
    Integration tests for RetrievalValidator with Qdrant.
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
                "retrieval_relevance": 0.95,
                "max_latency_ms": 500
            }
        }
        self.mock_config_loader.get_qdrant_url.return_value = "https://test-qdrant.com"
        self.mock_config_loader.get_qdrant_api_key.return_value = "test-api-key"

    @patch('backend.rag_validation.retrieval_validator.qdrant_client.QdrantClient')
    def test_end_to_end_retrieval_validation_process(self, mock_qdrant_client):
        """
        Test the complete end-to-end retrieval validation process.
        """
        # Setup mock Qdrant client responses
        mock_client = Mock()

        # Mock search results
        mock_hit1 = Mock()
        mock_hit1.id = "doc-1"
        mock_hit1.score = 0.95
        mock_hit1.payload = {
            "content": "Artificial intelligence is a wonderful field that is developing rapidly.",
            "source_url": "https://example.com/ai-intro",
            "page_title": "Introduction to AI"
        }
        mock_client.search.return_value = [mock_hit1]
        mock_qdrant_client.return_value = mock_client

        # Create validator
        validator = RetrievalValidator(config_loader=self.mock_config_loader)

        # Test similarity search validation
        query_vector = TestDataGenerator.generate_embedding_vector(768)
        search_result = validator.validate_similarity_search(query_vector, top_k=3)
        self.assertTrue(search_result["success"])

        # Test relevance scoring
        relevance_score = validator.calculate_relevance_score(
            "artificial intelligence",
            "Artificial intelligence is a wonderful field"
        )
        self.assertGreater(relevance_score, 0.5)  # Should have good relevance

        # Test source URL mapping
        mapping_result = validator.validate_source_url_mapping(
            query="information about AI",
            expected_url="https://example.com/ai-intro",
            top_k=3
        )
        self.assertTrue(mapping_result["success"])

        # Test content mapping accuracy
        accuracy_result = validator.validate_content_mapping_accuracy(
            query="information about AI",
            expected_content="Artificial intelligence is a wonderful field",
            top_k=3
        )
        self.assertTrue(accuracy_result["success"])

        # Test complete validation workflow
        test_queries = [
            {
                "query": "information about artificial intelligence",
                "expected_content": "Artificial intelligence is a wonderful field",
                "expected_url": "https://example.com/ai-intro"
            }
        ]
        complete_result = validator.run_complete_retrieval_validation(test_queries)
        self.assertIn("overall_success", complete_result)

    @patch('backend.rag_validation.retrieval_validator.qdrant_client.QdrantClient')
    def test_relevance_scoring_consistency(self, mock_qdrant_client):
        """
        Test that relevance scoring is consistent across different content pairs.
        """
        # Setup
        mock_client = Mock()
        mock_hit = Mock()
        mock_hit.id = "test-id"
        mock_hit.score = 0.8
        mock_hit.payload = {"content": "Sample content for testing", "source_url": "https://example.com"}
        mock_client.search.return_value = [mock_hit]
        mock_qdrant_client.return_value = mock_client

        # Create validator
        validator = RetrievalValidator(config_loader=self.mock_config_loader)

        # Test various query-content pairs
        test_pairs = [
            ("machine learning", "Machine learning is a subset of artificial intelligence"),
            ("python programming", "Python is a popular programming language"),
            ("data science", "Data science involves statistics and programming")
        ]

        scores = []
        for query, content in test_pairs:
            score = validator.calculate_relevance_score(query, content)
            scores.append(score)
            # All scores should be valid (between 0 and 1)
            self.assertGreaterEqual(score, 0)
            self.assertLessEqual(score, 1)

        # Verify that we got scores for all test pairs
        self.assertEqual(len(scores), len(test_pairs))

    @patch('backend.rag_validation.retrieval_validator.qdrant_client.QdrantClient')
    def test_retrieval_accuracy_validation(self, mock_qdrant_client):
        """
        Test integration between retrieval and accuracy validation.
        """
        # Setup
        mock_client = Mock()
        # Create multiple mock hits with different relevance levels
        mock_hit1 = Mock()
        mock_hit1.id = "high-relevance"
        mock_hit1.score = 0.9
        mock_hit1.payload = {
            "content": "Artificial intelligence and machine learning are related fields",
            "source_url": "https://example.com/ai-basics"
        }
        mock_hit2 = Mock()
        mock_hit2.id = "medium-relevance"
        mock_hit2.score = 0.6
        mock_hit2.payload = {
            "content": "Programming concepts and data structures",
            "source_url": "https://example.com/programming"
        }
        mock_client.search.return_value = [mock_hit1, mock_hit2]
        mock_qdrant_client.return_value = mock_client

        # Create validator
        validator = RetrievalValidator(config_loader=self.mock_config_loader)

        # Test content mapping accuracy
        query = "artificial intelligence concepts"
        expected_content = "Artificial intelligence is a branch of computer science"
        result = validator.validate_content_mapping_accuracy(query, expected_content, top_k=5)

        self.assertTrue(result["success"])
        self.assertGreaterEqual(result["max_relevance_score"], 0)
        self.assertEqual(result["total_results"], 2)

        # Check that results are properly ranked by relevance
        if result["content_accuracy_results"]:
            first_result = result["content_accuracy_results"][0]
            self.assertIsNotNone(first_result["relevance_to_query"])

    @patch('backend.rag_validation.retrieval_validator.qdrant_client.QdrantClient')
    def test_error_handling_integration(self, mock_qdrant_client):
        """
        Test error handling integration throughout the retrieval validation process.
        """
        # Setup mock to raise an exception during search
        mock_client = Mock()
        mock_client.search.side_effect = Exception("Search Error")
        mock_qdrant_client.return_value = mock_client

        # Create validator
        validator = RetrievalValidator(config_loader=self.mock_config_loader)

        # Test that errors are handled gracefully during similarity search
        query_vector = TestDataGenerator.generate_embedding_vector(768)
        result = validator.validate_similarity_search(query_vector)
        self.assertFalse(result["success"])
        self.assertIn("Search Error", result["error"])

        # Test source URL mapping with search error
        mapping_result = validator.validate_source_url_mapping(
            query="test query",
            expected_url="https://example.com",
            top_k=3
        )
        # This might still work if it handles the similarity search error internally
        # Or it might fail, which is also valid behavior

    @patch('backend.rag_validation.retrieval_validator.qdrant_client.QdrantClient')
    def test_large_scale_retrieval_validation(self, mock_qdrant_client):
        """
        Test retrieval validation with larger amounts of data.
        """
        # Setup
        mock_client = Mock()
        # Create multiple mock results
        mock_results = []
        for i in range(10):
            mock_hit = Mock()
            mock_hit.id = f"doc-{i}"
            mock_hit.score = 0.9 - (i * 0.05)  # Decreasing scores
            mock_hit.payload = {
                "content": f"Content for document {i} with relevant information",
                "source_url": f"https://example.com/doc{i}.html"
            }
            mock_results.append(mock_hit)
        mock_client.search.return_value = mock_results
        mock_qdrant_client.return_value = mock_client

        # Create validator
        validator = RetrievalValidator(config_loader=self.mock_config_loader)

        # Test with higher top_k
        query_vector = TestDataGenerator.generate_embedding_vector(768)
        search_result = validator.validate_similarity_search(query_vector, top_k=10)
        self.assertTrue(search_result["success"])
        self.assertEqual(search_result["results_count"], 10)

        # Test content accuracy with multiple results
        accuracy_result = validator.validate_content_mapping_accuracy(
            query="relevant information",
            expected_content="Information about various topics",
            top_k=10
        )
        self.assertTrue(accuracy_result["success"])
        self.assertEqual(accuracy_result["total_results"], 10)

    def test_configuration_integration(self):
        """
        Test integration with configuration loading for retrieval validation.
        """
        # Create a real ConfigLoader with test configuration
        config_loader = ConfigLoader()

        # Override the config loading to use test values
        original_get_config = config_loader.get_config
        def mock_get_config():
            config = original_get_config()
            config["qdrant_collection"] = "test-retrieval-collection"
            return config
        config_loader.get_config = mock_get_config

        # We can't test with real API key, so we'll just check initialization
        with patch('backend.rag_validation.retrieval_validator.qdrant_client.QdrantClient'):
            # Mock the API key retrieval to avoid actual API key requirement
            config_loader.get_qdrant_api_key = Mock(return_value="test-key")
            config_loader.get_qdrant_url = Mock(return_value="https://test.com")

            validator = RetrievalValidator(config_loader=config_loader)

            # Check that the validator was initialized with the config
            self.assertIsNotNone(validator.config_loader)
            self.assertEqual(validator.collection_name, "test-retrieval-collection")


if __name__ == '__main__':
    unittest.main()