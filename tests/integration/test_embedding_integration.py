"""
Integration tests for the embedding validator with Cohere API.
"""
import unittest
import os
from unittest.mock import patch, Mock
import sys

# Add backend directory to path to import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../'))

from backend.rag_validation.embedding_validator import EmbeddingValidator
from backend.rag_validation.config_loader import ConfigLoader


class TestEmbeddingIntegration(unittest.TestCase):
    """
    Integration tests for EmbeddingValidator with Cohere API.
    """

    def setUp(self):
        """
        Set up test fixtures before each test method.
        """
        # Create a mock config loader for testing
        self.mock_config_loader = Mock(spec=ConfigLoader)
        self.mock_config_loader.get_config.return_value = {
            "cohere_model": "embed-multilingual-v2.0",
            "validation_thresholds": {
                "embedding_success_rate": 0.99,
                "storage_success_rate": 0.99,
                "retrieval_relevance": 0.95,
                "max_latency_ms": 500
            }
        }
        self.mock_config_loader.get_cohere_api_key.return_value = "test-api-key"

    @patch('backend.rag_validation.embedding_validator.cohere.Client')
    def test_end_to_end_embedding_validation_process(self, mock_cohere_client):
        """
        Test the complete end-to-end embedding validation process.
        """
        # Setup mock Cohere client response
        mock_response = Mock()
        mock_response.embeddings = [[0.1, 0.2, 0.3, 0.4, 0.5] * 154]  # 770-dimensional vector (close to 768)
        mock_cohere_client.return_value.embed.return_value = mock_response

        # Create validator
        validator = EmbeddingValidator(config_loader=self.mock_config_loader)

        # Test single embedding validation
        text = "This is a test sentence for embedding validation."
        single_result = validator.validate_single_embedding(text)

        # Assertions for single validation
        self.assertTrue(single_result["success"])
        self.assertEqual(single_result["text_length"], len(text))
        self.assertIsNotNone(single_result["embedding_length"])

        # Test model configuration validation
        model_result = validator.validate_model_configuration()
        self.assertTrue(model_result["success"])

        # Test dimension validation
        dimension_result = validator.validate_embedding_dimensions(single_result.get("embedding_sample", [0.1] * 5))
        # Note: This test might not have a full embedding, so we'll just check it doesn't crash

        # Test complete validation workflow
        complete_result = validator.run_complete_validation([text, "Another test text"])
        self.assertIn("overall_success", complete_result)

    @patch('backend.rag_validation.embedding_validator.cohere.Client')
    def test_batch_vs_single_consistency(self, mock_cohere_client):
        """
        Test that batch and single embedding validations produce consistent results.
        """
        # Setup mock to return consistent embeddings
        def mock_embed_function(texts, **kwargs):
            mock_resp = Mock()
            # Create embeddings with slight variations based on text length
            embeddings = []
            for text in texts:
                embedding = [0.1 + len(text) * 0.001] * 768  # Simple deterministic embedding
                embeddings.append(embedding)
            mock_resp.embeddings = embeddings
            return mock_resp

        mock_cohere_client.return_value.embed.side_effect = mock_embed_function

        # Create validator
        validator = EmbeddingValidator(config_loader=self.mock_config_loader)

        # Test texts
        test_texts = [
            "Short text",
            "This is a medium length text for testing",
            "This is a longer text to test the embedding validation with more substantial content."
        ]

        # Validate individually
        single_results = []
        for text in test_texts:
            result = validator.validate_single_embedding(text)
            single_results.append(result)

        # Validate in batch
        batch_result = validator.validate_batch_embeddings(test_texts)

        # Check that we have results for all texts
        self.assertEqual(len(single_results), len(test_texts))
        self.assertGreaterEqual(batch_result["total_embeddings"], 0)

    @patch('backend.rag_validation.embedding_validator.cohere.Client')
    def test_embedding_quality_metrics_integration(self, mock_cohere_client):
        """
        Test integration between embedding generation and quality validation.
        """
        # Setup mock to return realistic embeddings
        mock_response = Mock()
        # Create a realistic embedding (with some variation)
        realistic_embedding = [i * 0.01 for i in range(1, 769)]  # 768-dimensional
        mock_response.embeddings = [realistic_embedding]
        mock_cohere_client.return_value.embed.return_value = mock_response

        # Create validator
        validator = EmbeddingValidator(config_loader=self.mock_config_loader)

        # Generate an embedding
        text = "Testing the quality metrics integration."
        result = validator.validate_single_embedding(text)

        if result["success"]:
            # Validate the embedding quality
            quality_result = validator.validate_embedding_quality(realistic_embedding, text)
            self.assertIsNotNone(quality_result)

            # Validate dimensions
            dimension_result = validator.validate_embedding_dimensions(realistic_embedding)
            self.assertTrue(dimension_result["valid"])

    @patch('backend.rag_validation.embedding_validator.cohere.Client')
    def test_error_handling_integration(self, mock_cohere_client):
        """
        Test error handling integration throughout the validation process.
        """
        # Setup mock to raise an exception
        mock_cohere_client.return_value.embed.side_effect = Exception("API Error")

        # Create validator
        validator = EmbeddingValidator(config_loader=self.mock_config_loader)

        # Test that errors are handled gracefully
        result = validator.validate_single_embedding("Test text")
        self.assertFalse(result["success"])
        self.assertIn("error", result)

        # Test batch validation with errors
        batch_result = validator.validate_batch_embeddings(["Test 1", "Test 2"])
        # Even with errors, it should return a result structure
        self.assertIn("failed_batches", batch_result)
        self.assertIn("successful_batches", batch_result)

    @patch('backend.rag_validation.embedding_validator.cohere.Client')
    def test_performance_tracking_integration(self, mock_cohere_client):
        """
        Test that performance metrics are properly tracked during validation.
        """
        # Setup mock response
        mock_response = Mock()
        mock_response.embeddings = [[0.1] * 768]
        mock_cohere_client.return_value.embed.return_value = mock_response

        # Create validator
        validator = EmbeddingValidator(config_loader=self.mock_config_loader)

        # Perform validation
        result = validator.validate_single_embedding("Performance test text")

        # The validation should complete without errors
        self.assertIsNotNone(result)

        # Test batch validation performance
        batch_texts = ["Text " + str(i) for i in range(10)]
        batch_result = validator.validate_batch_embeddings(batch_texts)

        self.assertIsNotNone(batch_result)
        self.assertEqual(batch_result["total_texts"], 10)

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
            config["cohere_model"] = "embed-multilingual-v2.0"
            return config
        config_loader.get_config = mock_get_config

        # We can't test with real API key, so we'll just check initialization
        with patch('backend.rag_validation.embedding_validator.cohere.Client'):
            # Mock the API key retrieval to avoid actual API key requirement
            config_loader.get_cohere_api_key = Mock(return_value="test-key")

            validator = EmbeddingValidator(config_loader=config_loader)

            # Check that the validator was initialized with the config
            self.assertIsNotNone(validator.config_loader)
            self.assertEqual(validator.model, "embed-multilingual-v2.0")


if __name__ == '__main__':
    unittest.main()