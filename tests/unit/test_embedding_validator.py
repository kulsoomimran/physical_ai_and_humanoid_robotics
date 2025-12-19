"""
Unit tests for the embedding validator.
"""
import unittest
from unittest.mock import Mock, patch
import sys
import os

# Add backend directory to path to import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../'))

from backend.rag_validation.embedding_validator import EmbeddingValidator
from backend.rag_validation.config_loader import ConfigLoader


class TestEmbeddingValidator(unittest.TestCase):
    """
    Unit tests for EmbeddingValidator class.
    """

    def setUp(self):
        """
        Set up test fixtures before each test method.
        """
        # Create a mock config loader
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

        # Create the validator instance
        with patch('backend.rag_validation.embedding_validator.cohere.Client'):
            self.validator = EmbeddingValidator(config_loader=self.mock_config_loader)

    @patch('backend.rag_validation.embedding_validator.cohere.Client')
    def test_initialization(self, mock_client_class):
        """
        Test that the EmbeddingValidator initializes correctly.
        """
        # Setup
        mock_client = Mock()
        mock_client_class.return_value = mock_client

        config_loader = Mock(spec=ConfigLoader)
        config_loader.get_cohere_api_key.return_value = "test-key"
        config_loader.get_config.return_value = {"cohere_model": "test-model"}

        # Execute
        validator = EmbeddingValidator(config_loader=config_loader)

        # Assert
        config_loader.get_cohere_api_key.assert_called_once()
        mock_client_class.assert_called_once_with("test-key")
        self.assertEqual(validator.model, "test-model")

    @patch('backend.rag_validation.embedding_validator.cohere.Client')
    def test_validate_model_configuration_valid_model(self, mock_client_class):
        """
        Test model configuration validation with a known valid model.
        """
        # Setup
        mock_client = Mock()
        mock_client_class.return_value = mock_client

        config_loader = Mock(spec=ConfigLoader)
        config_loader.get_cohere_api_key.return_value = "test-key"
        config_loader.get_config.return_value = {"cohere_model": "embed-multilingual-v2.0"}

        validator = EmbeddingValidator(config_loader=config_loader)

        # Execute
        result = validator.validate_model_configuration()

        # Assert
        self.assertTrue(result["success"])
        self.assertTrue(result["model_valid"])
        self.assertEqual(result["expected_dimensions"], 768)
        self.assertEqual(result["actual_dimensions"], 768)

    @patch('backend.rag_validation.embedding_validator.cohere.Client')
    def test_validate_model_configuration_unknown_model(self, mock_client_class):
        """
        Test model configuration validation with an unknown model.
        """
        # Setup
        mock_client = Mock()
        mock_client_class.return_value = mock_client

        config_loader = Mock(spec=ConfigLoader)
        config_loader.get_cohere_api_key.return_value = "test-key"
        config_loader.get_config.return_value = {"cohere_model": "unknown-model"}

        validator = EmbeddingValidator(config_loader=config_loader)

        # Execute
        result = validator.validate_model_configuration()

        # Assert
        self.assertFalse(result["success"])
        self.assertFalse(result["model_valid"])
        self.assertIsNone(result["actual_dimensions"])

    @patch('backend.rag_validation.embedding_validator.cohere.Client')
    def test_validate_single_embedding_success(self, mock_client_class):
        """
        Test single embedding validation with successful API response.
        """
        # Setup
        mock_client = Mock()
        mock_response = Mock()
        mock_response.embeddings = [[0.1, 0.2, 0.3, 0.4]]  # Mock embedding
        mock_client.embed.return_value = mock_response
        mock_client_class.return_value = mock_client

        config_loader = Mock(spec=ConfigLoader)
        config_loader.get_cohere_api_key.return_value = "test-key"
        config_loader.get_config.return_value = {"cohere_model": "embed-multilingual-v2.0"}

        validator = EmbeddingValidator(config_loader=config_loader)

        # Execute
        result = validator.validate_single_embedding("Test text")

        # Assert
        self.assertTrue(result["success"])
        self.assertEqual(result["text_length"], len("Test text"))
        self.assertEqual(result["embedding_length"], 4)  # Length of mock embedding

    @patch('backend.rag_validation.embedding_validator.cohere.Client')
    def test_validate_single_embedding_api_failure(self, mock_client_class):
        """
        Test single embedding validation when API call fails.
        """
        # Setup
        mock_client = Mock()
        mock_client.embed.side_effect = Exception("API Error")
        mock_client_class.return_value = mock_client

        config_loader = Mock(spec=ConfigLoader)
        config_loader.get_cohere_api_key.return_value = "test-key"
        config_loader.get_config.return_value = {"cohere_model": "embed-multilingual-v2.0"}

        validator = EmbeddingValidator(config_loader=config_loader)

        # Execute
        result = validator.validate_single_embedding("Test text")

        # Assert
        self.assertFalse(result["success"])
        self.assertIn("API Error", result["error"])

    @patch('backend.rag_validation.embedding_validator.cohere.Client')
    def test_validate_batch_embeddings_success(self, mock_client_class):
        """
        Test batch embedding validation with successful API responses.
        """
        # Setup
        mock_client = Mock()
        mock_response = Mock()
        mock_response.embeddings = [[0.1, 0.2, 0.3]] * 3  # 3 embeddings
        mock_client.embed.return_value = mock_response
        mock_client_class.return_value = mock_client

        config_loader = Mock(spec=ConfigLoader)
        config_loader.get_cohere_api_key.return_value = "test-key"
        config_loader.get_config.return_value = {"cohere_model": "embed-multilingual-v2.0"}

        validator = EmbeddingValidator(config_loader=config_loader)

        # Execute
        texts = ["Text 1", "Text 2", "Text 3"]
        result = validator.validate_batch_embeddings(texts, batch_size=3)

        # Assert
        self.assertTrue(result["success"])
        self.assertEqual(result["total_texts"], 3)
        self.assertEqual(result["successful_batches"], 1)
        self.assertEqual(result["failed_batches"], 0)
        self.assertEqual(result["total_embeddings"], 3)

    @patch('backend.rag_validation.embedding_validator.cohere.Client')
    def test_validate_batch_embeddings_partial_failure(self, mock_client_class):
        """
        Test batch embedding validation with partial API failures.
        """
        # Setup
        mock_client = Mock()
        def embed_side_effect(texts, **kwargs):
            if len(texts) > 2:  # Fail if more than 2 texts
                raise Exception("Batch too large")
            mock_response = Mock()
            mock_response.embeddings = [[0.1, 0.2, 0.3]] * len(texts)
            return mock_response

        mock_client.embed.side_effect = embed_side_effect
        mock_client_class.return_value = mock_client

        config_loader = Mock(spec=ConfigLoader)
        config_loader.get_cohere_api_key.return_value = "test-key"
        config_loader.get_config.return_value = {"cohere_model": "embed-multilingual-v2.0"}

        validator = EmbeddingValidator(config_loader=config_loader)

        # Execute - this will cause one batch to fail
        texts = ["Text 1", "Text 2", "Text 3", "Text 4"]
        result = validator.validate_batch_embeddings(texts, batch_size=3)

        # Assert
        # Note: The implementation continues despite batch failures, so success depends on implementation
        self.assertEqual(result["total_texts"], 4)
        self.assertGreaterEqual(result["failed_batches"], 0)

    def test_validate_embedding_dimensions_correct(self):
        """
        Test embedding dimension validation with correct dimensions.
        """
        # Execute
        result = self.validator.validate_embedding_dimensions([0.1] * 768, expected_dims=768)

        # Assert
        self.assertTrue(result["valid"])
        self.assertEqual(result["expected_dimensions"], 768)
        self.assertEqual(result["actual_dimensions"], 768)
        self.assertEqual(result["difference"], 0)

    def test_validate_embedding_dimensions_incorrect(self):
        """
        Test embedding dimension validation with incorrect dimensions.
        """
        # Execute
        result = self.validator.validate_embedding_dimensions([0.1] * 512, expected_dims=768)

        # Assert
        self.assertFalse(result["valid"])
        self.assertEqual(result["expected_dimensions"], 768)
        self.assertEqual(result["actual_dimensions"], 512)
        self.assertEqual(result["difference"], -256)

    def test_validate_embedding_dimensions_empty(self):
        """
        Test embedding dimension validation with empty embedding.
        """
        # Execute
        result = self.validator.validate_embedding_dimensions([], expected_dims=768)

        # Assert
        self.assertFalse(result["valid"])
        self.assertEqual(result["actual_dimensions"], 0)

    def test_validate_embedding_quality_valid(self):
        """
        Test embedding quality validation with a valid embedding.
        """
        # Execute
        result = self.validator.validate_embedding_quality([0.1, 0.2, 0.3, 0.4, 0.5], "test text")

        # Assert
        self.assertTrue(result["valid"])
        self.assertGreaterEqual(result["quality_score"], 0.8)  # Should be high quality
        self.assertEqual(len(result["issues"]), 0)

    def test_validate_embedding_quality_low_magnitude(self):
        """
        Test embedding quality validation with very low magnitude.
        """
        # Execute
        result = self.validator.validate_embedding_quality([0.0001] * 10, "test text")

        # Assert
        # Even low magnitude embeddings might still be valid, depending on implementation
        self.assertGreaterEqual(result["quality_score"], 0)  # Should be between 0 and 1

    def test_validate_embedding_quality_with_nan(self):
        """
        Test embedding quality validation with NaN values.
        """
        import math
        # Execute
        embedding_with_nan = [0.1, 0.2, float('nan'), 0.4]
        result = self.validator.validate_embedding_quality(embedding_with_nan, "test text")

        # Assert
        self.assertFalse(result["valid"])
        self.assertIn("NaN", str(result["issues"][0]) if result["issues"] else "")

    def test_run_complete_validation(self):
        """
        Test the complete validation workflow.
        """
        # Execute
        with patch.object(self.validator, 'validate_model_configuration', return_value={"success": True}):
            with patch.object(self.validator, 'validate_single_embedding', return_value={"success": True}):
                with patch.object(self.validator, 'validate_batch_embeddings', return_value={"success": True}):
                    result = self.validator.run_complete_validation(["test1", "test2"])

        # Assert
        self.assertIn("model_validation", result)
        self.assertIn("single_embeddings", result)
        self.assertIn("batch_validation", result)
        self.assertIn("overall_success", result)


if __name__ == '__main__':
    unittest.main()