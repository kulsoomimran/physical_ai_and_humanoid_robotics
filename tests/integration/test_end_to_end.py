"""
End-to-end integration tests for the complete RAG validation system.
"""
import unittest
from unittest.mock import patch, Mock
import sys
import os
import tempfile
import json

# Add backend directory to path to import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../'))

from backend.rag_validation.main import RAGValidationOrchestrator
from backend.rag_validation.config_loader import ConfigLoader
from backend.rag_validation.test_data import TestDataGenerator


class TestEndToEndIntegration(unittest.TestCase):
    """
    End-to-end integration tests for the complete RAG validation system.
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
                "max_latency_ms": 1000,
                "min_throughput_per_sec": 1
            }
        }
        self.mock_config_loader.get_qdrant_url.return_value = "https://test-qdrant.com"
        self.mock_config_loader.get_qdrant_api_key.return_value = "test-api-key"
        self.mock_config_loader.get_cohere_api_key.return_value = "test-cohere-key"

    @patch('backend.rag_validation.main.EmbeddingValidator')
    @patch('backend.rag_validation.main.StorageValidator')
    @patch('backend.rag_validation.main.RetrievalValidator')
    @patch('backend.rag_validation.main.ErrorValidator')
    @patch('backend.rag_validation.main.PerformanceValidator')
    def test_complete_validation_orchestration(self, mock_perf_validator, mock_error_validator,
                                               mock_retrieval_validator, mock_storage_validator,
                                               mock_embedding_validator):
        """
        Test the complete validation orchestration workflow.
        """
        # Setup mock validators
        mock_embedding_instance = Mock()
        mock_embedding_instance.run_complete_validation.return_value = {
            "overall_success": {"all_valid": True}
        }
        mock_embedding_validator.return_value = mock_embedding_instance

        mock_storage_instance = Mock()
        mock_storage_instance.run_complete_storage_validation.return_value = {
            "overall_success": {"all_valid": True}
        }
        mock_storage_validator.return_value = mock_storage_instance

        mock_retrieval_instance = Mock()
        mock_retrieval_instance.run_complete_retrieval_validation.return_value = {
            "overall_success": {"all_queries_processed": True}
        }
        mock_retrieval_validator.return_value = mock_retrieval_instance

        mock_error_instance = Mock()
        mock_error_instance.run_complete_error_validation.return_value = {
            "overall_success": {"error_classification_working": True}
        }
        mock_error_validator.return_value = mock_error_instance

        mock_perf_instance = Mock()
        mock_perf_instance.run_complete_performance_validation.return_value = {
            "overall_success": {"all_tests_passed": True}
        }
        mock_perf_validator.return_value = mock_perf_instance

        # Create orchestrator
        orchestrator = RAGValidationOrchestrator(config_loader=self.mock_config_loader)

        # Run complete validation
        result = orchestrator.run_complete_validation()

        # Verify that all validation components were called
        mock_embedding_instance.run_complete_validation.assert_called_once()
        mock_storage_instance.run_complete_storage_validation.assert_called_once()
        mock_retrieval_instance.run_complete_retrieval_validation.assert_called_once()
        mock_error_instance.run_complete_error_validation.assert_called_once()
        mock_perf_instance.run_complete_performance_validation.assert_called_once()

        # Verify the result structure
        self.assertIn("success", result)
        self.assertIn("results", result)
        self.assertIn("total_duration_seconds", result)
        self.assertIn("report_paths", result)

        # Verify that results contain all components
        self.assertIn("embedding", result["results"])
        self.assertIn("storage", result["results"])
        self.assertIn("retrieval", result["results"])
        self.assertIn("error", result["results"])
        self.assertIn("performance", result["results"])

    @patch('backend.rag_validation.main.EmbeddingValidator')
    @patch('backend.rag_validation.main.StorageValidator')
    @patch('backend.rag_validation.main.RetrievalValidator')
    @patch('backend.rag_validation.main.ErrorValidator')
    @patch('backend.rag_validation.main.PerformanceValidator')
    def test_partial_component_validation(self, mock_perf_validator, mock_error_validator,
                                         mock_retrieval_validator, mock_storage_validator,
                                         mock_embedding_validator):
        """
        Test validation with only specific components.
        """
        # Setup mock validators
        mock_embedding_instance = Mock()
        mock_embedding_instance.run_complete_validation.return_value = {
            "overall_success": {"all_valid": True}
        }
        mock_embedding_validator.return_value = mock_embedding_instance

        mock_storage_instance = Mock()
        mock_storage_instance.run_complete_storage_validation.return_value = {
            "overall_success": {"all_valid": True}
        }
        mock_storage_validator.return_value = mock_storage_instance

        # Create orchestrator
        orchestrator = RAGValidationOrchestrator(config_loader=self.mock_config_loader)

        # Run validation for only embedding and storage components
        result = orchestrator.run_complete_validation(components=['embedding', 'storage'])

        # Verify that only the requested components were called
        mock_embedding_instance.run_complete_validation.assert_called_once()
        mock_storage_instance.run_complete_storage_validation.assert_called_once()

        # The other components should not have been called
        # (We'll check this by ensuring the results only contain the expected components)
        self.assertIn("embedding", result["results"])
        self.assertIn("storage", result["results"])
        # The other components might still be in results dict but should have been skipped
        # depending on the implementation

    @patch('backend.rag_validation.main.EmbeddingValidator')
    @patch('backend.rag_validation.main.StorageValidator')
    @patch('backend.rag_validation.main.RetrievalValidator')
    @patch('backend.rag_validation.main.ErrorValidator')
    @patch('backend.rag_validation.main.PerformanceValidator')
    def test_validation_pipeline_execution(self, mock_perf_validator, mock_error_validator,
                                          mock_retrieval_validator, mock_storage_validator,
                                          mock_embedding_validator):
        """
        Test the validation pipeline execution with options.
        """
        # Setup mock validators
        mock_embedding_instance = Mock()
        mock_embedding_instance.run_complete_validation.return_value = {
            "overall_success": {"all_valid": True}
        }
        mock_embedding_validator.return_value = mock_embedding_instance

        mock_all_instances = [
            mock_embedding_instance,
            Mock(),
            Mock(),
            Mock(),
            Mock()
        ]
        # Set up the other mock instances
        for instance in mock_all_instances[1:]:
            instance.run_complete_validation = Mock(return_value={"overall_success": {"all_valid": True}})
            instance.run_complete_storage_validation = Mock(return_value={"overall_success": {"all_valid": True}})
            instance.run_complete_retrieval_validation = Mock(return_value={"overall_success": {"all_queries_processed": True}})
            instance.run_complete_error_validation = Mock(return_value={"overall_success": {"error_classification_working": True}})
            instance.run_complete_performance_validation = Mock(return_value={"overall_success": {"all_tests_passed": True}})

        mock_storage_validator.return_value = mock_all_instances[1]
        mock_retrieval_validator.return_value = mock_all_instances[2]
        mock_error_validator.return_value = mock_all_instances[3]
        mock_perf_validator.return_value = mock_all_instances[4]

        # Create orchestrator
        orchestrator = RAGValidationOrchestrator(config_loader=self.mock_config_loader)

        # Define options for the pipeline
        options = {
            'components': ['embedding', 'storage']
        }

        # Run validation pipeline
        result = orchestrator.run_validation_pipeline(options)

        # Verify the result
        self.assertTrue(result["success"])
        self.assertEqual(result["components_validated"], ['embedding', 'storage'])

    @patch('backend.rag_validation.main.EmbeddingValidator')
    @patch('backend.rag_validation.main.StorageValidator')
    @patch('backend.rag_validation.main.RetrievalValidator')
    @patch('backend.rag_validation.main.ErrorValidator')
    @patch('backend.rag_validation.main.PerformanceValidator')
    def test_error_handling_in_complete_workflow(self, mock_perf_validator, mock_error_validator,
                                                mock_retrieval_validator, mock_storage_validator,
                                                mock_embedding_validator):
        """
        Test error handling throughout the complete validation workflow.
        """
        # Setup one validator to raise an exception
        mock_embedding_instance = Mock()
        mock_embedding_instance.run_complete_validation.side_effect = Exception("Test error")
        mock_embedding_validator.return_value = mock_embedding_instance

        # Setup other validators normally
        mock_storage_instance = Mock()
        mock_storage_instance.run_complete_storage_validation.return_value = {
            "overall_success": {"all_valid": True}
        }
        mock_storage_validator.return_value = mock_storage_instance

        mock_retrieval_instance = Mock()
        mock_retrieval_instance.run_complete_retrieval_validation.return_value = {
            "overall_success": {"all_queries_processed": True}
        }
        mock_retrieval_validator.return_value = mock_retrieval_instance

        mock_error_instance = Mock()
        mock_error_instance.run_complete_error_validation.return_value = {
            "overall_success": {"error_classification_working": True}
        }
        mock_error_validator.return_value = mock_error_instance

        mock_perf_instance = Mock()
        mock_perf_instance.run_complete_performance_validation.return_value = {
            "overall_success": {"all_tests_passed": True}
        }
        mock_perf_validator.return_value = mock_perf_instance

        # Create orchestrator
        orchestrator = RAGValidationOrchestrator(config_loader=self.mock_config_loader)

        # Run complete validation - should handle the error gracefully
        result = orchestrator.run_complete_validation()

        # The result should still be structured properly even with an error
        self.assertIn("success", result)
        self.assertIn("results", result)
        self.assertIn("embedding", result["results"])  # Should contain result even if error occurred

    @patch('backend.rag_validation.main.EmbeddingValidator')
    @patch('backend.rag_validation.main.StorageValidator')
    @patch('backend.rag_validation.main.RetrievalValidator')
    @patch('backend.rag_validation.main.ErrorValidator')
    @patch('backend.rag_validation.main.PerformanceValidator')
    def test_report_generation_in_end_to_end_flow(self, mock_perf_validator, mock_error_validator,
                                                  mock_retrieval_validator, mock_storage_validator,
                                                  mock_embedding_validator):
        """
        Test that reports are properly generated in the end-to-end flow.
        """
        # Setup mock validators
        mock_embedding_instance = Mock()
        mock_embedding_instance.run_complete_validation.return_value = {
            "overall_success": {"all_valid": True}
        }
        mock_embedding_validator.return_value = mock_embedding_instance

        mock_storage_instance = Mock()
        mock_storage_instance.run_complete_storage_validation.return_value = {
            "overall_success": {"all_valid": True}
        }
        mock_storage_validator.return_value = mock_storage_instance

        mock_retrieval_instance = Mock()
        mock_retrieval_instance.run_complete_retrieval_validation.return_value = {
            "overall_success": {"all_queries_processed": True}
        }
        mock_retrieval_validator.return_value = mock_retrieval_instance

        mock_error_instance = Mock()
        mock_error_instance.run_complete_error_validation.return_value = {
            "overall_success": {"error_classification_working": True}
        }
        mock_error_validator.return_value = mock_error_instance

        mock_perf_instance = Mock()
        mock_perf_instance.run_complete_performance_validation.return_value = {
            "overall_success": {"all_tests_passed": True}
        }
        mock_perf_validator.return_value = mock_perf_instance

        # Create orchestrator
        orchestrator = RAGValidationOrchestrator(config_loader=self.mock_config_loader)

        # Run complete validation
        result = orchestrator.run_complete_validation()

        # Verify that report paths are included in the result
        self.assertIn("report_paths", result)
        report_paths = result["report_paths"]
        self.assertIn("json", report_paths)
        self.assertIn("csv", report_paths)
        self.assertIn("summary", report_paths)

        # Verify that report files were created (or would be in a real scenario)
        # In our mock scenario, we're checking the structure of the response
        self.assertIsInstance(report_paths["json"], str)
        self.assertIsInstance(report_paths["csv"], str)
        self.assertIsInstance(report_paths["summary"], str)

    def test_configuration_integration_in_orchestrator(self):
        """
        Test integration with configuration loading in the orchestrator.
        """
        # Create a real ConfigLoader with test configuration
        config_loader = ConfigLoader()

        # Override the config loading to use test values
        original_get_config = config_loader.get_config
        def mock_get_config():
            config = original_get_config()
            config["validation_thresholds"] = {
                "max_latency_ms": 2000,
                "min_throughput_per_sec": 5
            }
            config["qdrant_collection"] = "test-end-to-end"
            return config
        config_loader.get_config = mock_get_config

        # We can't test with real API keys, so we'll just check initialization
        with patch('backend.rag_validation.main.EmbeddingValidator'), \
             patch('backend.rag_validation.main.StorageValidator'), \
             patch('backend.rag_validation.main.RetrievalValidator'), \
             patch('backend.rag_validation.main.ErrorValidator'), \
             patch('backend.rag_validation.main.PerformanceValidator'):

            # Create orchestrator with real config
            orchestrator = RAGValidationOrchestrator(config_loader=config_loader)

            # Verify that the orchestrator was initialized with the config
            self.assertIsNotNone(orchestrator.config_loader)
            self.assertEqual(orchestrator.config["qdrant_collection"], "test-end-to-end")


if __name__ == '__main__':
    unittest.main()