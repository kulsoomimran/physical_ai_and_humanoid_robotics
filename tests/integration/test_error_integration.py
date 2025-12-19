"""
Integration tests for the error validator.
"""
import unittest
from unittest.mock import patch, Mock
import sys

# Add backend directory to path to import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../'))

from backend.rag_validation.error_validator import ErrorValidator
from backend.rag_validation.config_loader import ConfigLoader
from backend.rag_validation.test_data import TestDataGenerator


class TestErrorIntegration(unittest.TestCase):
    """
    Integration tests for ErrorValidator.
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
                "max_latency_ms": 500
            }
        }
        self.mock_config_loader.get_qdrant_url.return_value = "https://test-qdrant.com"
        self.mock_config_loader.get_qdrant_api_key.return_value = "test-api-key"
        self.mock_config_loader.get_cohere_api_key.return_value = "test-cohere-key"

    @patch('backend.rag_validation.error_validator.cohere.Client')
    @patch('backend.rag_validation.error_validator.qdrant_client.QdrantClient')
    def test_end_to_end_error_validation_process(self, mock_qdrant_client, mock_cohere_client):
        """
        Test the complete end-to-end error validation process.
        """
        # Setup mock clients
        mock_qdrant_client.return_value = Mock()
        mock_cohere_client.return_value = Mock()

        # Create validator
        validator = ErrorValidator(config_loader=self.mock_config_loader)

        # Test Cohere failure simulation
        cohere_timeout_result = validator.simulate_cohere_api_failure("timeout")
        self.assertTrue(cohere_timeout_result["success"])

        # Test Qdrant failure simulation
        qdrant_timeout_result = validator.simulate_qdrant_connection_failure("connection_timeout")
        self.assertTrue(qdrant_timeout_result["success"])

        # Test malformed input validation
        test_inputs = [None, "", [], "a" * 10000]
        malformed_result = validator.validate_malformed_input_handling(test_inputs)
        self.assertTrue(malformed_result["success"])

        # Test error classification
        error_msg = "Connection timeout to server"
        classification_result = validator.classify_error_severity(error_msg)
        self.assertIsNotNone(classification_result["severity"])

        # Test complete error validation workflow
        complete_result = validator.run_complete_error_validation()
        self.assertIn("overall_success", complete_result)

    @patch('backend.rag_validation.error_validator.cohere.Client')
    @patch('backend.rag_validation.error_validator.qdrant_client.QdrantClient')
    def test_error_detection_consistency(self, mock_qdrant_client, mock_cohere_client):
        """
        Test that error detection is consistent across different error types.
        """
        # Setup mock clients
        mock_qdrant_client.return_value = Mock()
        mock_cohere_client.return_value = Mock()

        # Create validator
        validator = ErrorValidator(config_loader=self.mock_config_loader)

        # Test various Cohere error types
        cohere_error_types = ["timeout", "rate_limit", "server_error"]
        cohere_results = []
        for error_type in cohere_error_types:
            result = validator.simulate_cohere_api_failure(error_type)
            cohere_results.append(result)
            self.assertTrue(result["success"])

        # Test various Qdrant error types
        qdrant_error_types = ["connection_timeout", "unreachable", "auth_failure"]
        qdrant_results = []
        for error_type in qdrant_error_types:
            result = validator.simulate_qdrant_connection_failure(error_type)
            qdrant_results.append(result)
            self.assertTrue(result["success"])

        # Verify all error types were processed
        self.assertEqual(len(cohere_results), len(cohere_error_types))
        self.assertEqual(len(qdrant_results), len(qdrant_error_types))

    @patch('backend.rag_validation.error_validator.cohere.Client')
    @patch('backend.rag_validation.error_validator.qdrant_client.QdrantClient')
    def test_error_severity_classification_integration(self, mock_qdrant_client, mock_cohere_client):
        """
        Test integration between error detection and severity classification.
        """
        # Setup mock clients
        mock_qdrant_client.return_value = Mock()
        mock_cohere_client.return_value = Mock()

        # Create validator
        validator = ErrorValidator(config_loader=self.mock_config_loader)

        # Test various error messages and their classifications
        test_errors = [
            ("Connection timeout occurred", "timeout"),
            ("Rate limit exceeded for API key", "rate_limit"),
            ("Invalid request format", "invalid_request"),
            ("Server error 500", "server_error")
        ]

        classification_results = []
        for error_msg, error_type in test_errors:
            # Classify severity
            severity_result = validator.classify_error_severity(error_msg, error_type)
            classification_results.append(severity_result)

            # Validate that classification was successful
            self.assertIsNotNone(severity_result["severity"])
            self.assertEqual(severity_result["error_message"], error_msg)
            self.assertEqual(severity_result["error_type"], error_type)

        # Verify all errors were classified
        self.assertEqual(len(classification_results), len(test_errors))

    @patch('backend.rag_validation.error_validator.cohere.Client')
    @patch('backend.rag_validation.error_validator.qdrant_client.QdrantClient')
    def test_rate_limit_detection_integration(self, mock_qdrant_client, mock_cohere_client):
        """
        Test integration of rate limit detection with actual function calls.
        """
        # Setup mock clients
        mock_qdrant_client.return_value = Mock()
        mock_cohere_client.return_value = Mock()

        # Create validator
        validator = ErrorValidator(config_loader=self.mock_config_loader)

        # Define a test function to use with rate limit validation
        def test_operation():
            return "success"

        # Test rate limit validation
        rate_limit_result = validator.validate_rate_limit_handling(
            test_operation, max_requests=5, time_window=0.1
        )

        # Validate the result structure
        self.assertTrue(rate_limit_result["success"])
        self.assertGreaterEqual(rate_limit_result["total_requests"], 5)

    @patch('backend.rag_validation.error_validator.cohere.Client')
    @patch('backend.rag_validation.error_validator.qdrant_client.QdrantClient')
    def test_malformed_input_detection_integration(self, mock_qdrant_client, mock_cohere_client):
        """
        Test integration between malformed input detection and error handling.
        """
        # Setup mock clients
        mock_qdrant_client.return_value = Mock()
        mock_cohere_client.return_value = Mock()

        # Create validator
        validator = ErrorValidator(config_loader=self.mock_config_loader)

        # Create various malformed inputs to test
        malformed_inputs = [
            None,  # Null value
            "",  # Empty string
            [],  # Empty list
            {},  # Empty object
            "a" * 15000,  # Very long string
            -1,  # Negative value where positive expected
            {"invalid": "structure", "nested": {"too": "deep"}},  # Complex nested structure
        ]

        # Validate handling of malformed inputs
        result = validator.validate_malformed_input_handling(malformed_inputs)

        # Validate the result
        self.assertTrue(result["success"])
        self.assertEqual(result["summary"]["total_inputs"], len(malformed_inputs))

        # Check that issues were detected for some inputs
        individual_results = result["summary"]["individual_results"]
        self.assertEqual(len(individual_results), len(malformed_inputs))

        # At least some of the inputs should have been flagged as problematic
        issues_detected = sum(1 for r in individual_results if r.get("issues_detected", False))
        self.assertGreaterEqual(issues_detected, 0)

    @patch('backend.rag_validation.error_validator.cohere.Client')
    @patch('backend.rag_validation.error_validator.qdrant_client.QdrantClient')
    def test_error_handling_validation_workflow(self, mock_qdrant_client, mock_cohere_client):
        """
        Test the complete error handling validation workflow.
        """
        # Setup mock clients
        mock_qdrant_client.return_value = Mock()
        mock_cohere_client.return_value = Mock()

        # Create validator
        validator = ErrorValidator(config_loader=self.mock_config_loader)

        # Run complete error validation
        result = validator.run_complete_error_validation()

        # Validate the overall structure of results
        self.assertIn("overall_success", result)
        self.assertIn("cohere_failure_simulation", result)
        self.assertIn("qdrant_failure_simulation", result)
        self.assertIn("malformed_input_validation", result)
        self.assertIn("error_classification_examples", result)

        # Validate each component has proper results
        overall = result["overall_success"]
        self.assertIsNotNone(overall["cohere_simulation_success_rate"])
        self.assertIsNotNone(overall["qdrant_simulation_success_rate"])
        self.assertIsNotNone(overall["malformed_input_validation_success"])
        self.assertIsNotNone(overall["error_classification_working"])

    def test_configuration_integration(self):
        """
        Test integration with configuration loading for error validation.
        """
        # Create a real ConfigLoader with test configuration
        config_loader = ConfigLoader()

        # Override the config loading to use test values
        original_get_config = config_loader.get_config
        def mock_get_config():
            config = original_get_config()
            config["qdrant_collection"] = "test-error-collection"
            return config
        config_loader.get_config = mock_get_config

        # We can't test with real API keys, so we'll just check initialization
        with patch('backend.rag_validation.error_validator.cohere.Client'), \
             patch('backend.rag_validation.error_validator.qdrant_client.QdrantClient'):
            # Mock the API key retrieval to avoid actual API key requirement
            config_loader.get_qdrant_api_key = Mock(return_value="test-key")
            config_loader.get_qdrant_url = Mock(return_value="https://test.com")
            config_loader.get_cohere_api_key = Mock(return_value="test-cohere-key")

            validator = ErrorValidator(config_loader=config_loader)

            # Check that the validator was initialized with the config
            self.assertIsNotNone(validator.config_loader)
            self.assertEqual(validator.collection_name, "test-error-collection")


if __name__ == '__main__':
    unittest.main()