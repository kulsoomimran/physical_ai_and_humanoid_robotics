"""
Unit tests for the performance validator.
"""
import unittest
from unittest.mock import Mock, patch, MagicMock
import sys
import os
import time

# Add backend directory to path to import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../'))

from backend.rag_validation.performance_validator import PerformanceValidator
from backend.rag_validation.config_loader import ConfigLoader


class TestPerformanceValidator(unittest.TestCase):
    """
    Unit tests for PerformanceValidator class.
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
                "max_latency_ms": 500,
                "min_throughput_per_sec": 10
            }
        }

        # Create the validator instance
        self.validator = PerformanceValidator(config_loader=self.mock_config_loader)

    def test_initialization(self):
        """
        Test that the PerformanceValidator initializes correctly.
        """
        # Verify that the validator was initialized with correct thresholds
        self.assertEqual(self.validator.max_latency_ms, 500)
        self.assertEqual(self.validator.min_throughput_per_sec, 10)

    def test_measure_retrieval_latency_success(self):
        """
        Test measuring retrieval latency for a successful operation.
        """
        # Define a test operation that takes some time
        def test_operation():
            time.sleep(0.01)  # Sleep for 10ms
            return "success result"

        # Execute
        result = self.validator.measure_retrieval_latency(test_operation)

        # Assert
        self.assertTrue(result["success"])
        self.assertIsNotNone(result["latency_ms"])
        self.assertGreaterEqual(result["latency_ms"], 10)  # Should be at least 10ms
        self.assertEqual(result["operation_result"], "success result")

    def test_measure_retrieval_latency_failure(self):
        """
        Test measuring retrieval latency when operation fails.
        """
        # Define a test operation that raises an exception
        def failing_operation():
            raise Exception("Operation failed")

        # Execute
        result = self.validator.measure_retrieval_latency(failing_operation)

        # Assert
        self.assertFalse(result["success"])
        self.assertIn("Operation failed", result["error"])
        self.assertIsNotNone(result["latency_ms"])

    def test_calculate_percentile_latency_p95(self):
        """
        Test calculating p95 latency from a list of latencies.
        """
        # Create a list of latencies
        latencies = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]  # 10 values

        # Execute
        p95_result = self.validator.calculate_percentile_latency(latencies, 95)

        # For 10 values, p95 should be the 10th value (index 9) which is 100
        # But with our calculation: int(0.95 * 10) = 9, so index 9 = 100
        self.assertEqual(p95_result, 100)

    def test_calculate_percentile_latency_p50(self):
        """
        Test calculating p50 (median) latency from a list of latencies.
        """
        # Create a list of latencies
        latencies = [10, 20, 30, 40, 50]  # 5 values

        # Execute
        p50_result = self.validator.calculate_percentile_latency(latencies, 50)

        # For 5 values, p50 should be the 3rd value (index 2) which is 30
        # With our calculation: int(0.5 * 5) = 2, so index 2 = 30
        self.assertEqual(p50_result, 30)

    def test_calculate_percentile_latency_empty_list(self):
        """
        Test calculating percentile latency with an empty list.
        """
        # Execute
        result = self.validator.calculate_percentile_latency([], 95)

        # Should return 0 for empty list
        self.assertEqual(result, 0.0)

    def test_run_concurrent_validation(self):
        """
        Test running concurrent validation with multiple threads.
        """
        # Define a simple test operation
        def test_operation():
            time.sleep(0.001)  # Small delay
            return "result"

        # Execute with minimal parameters for testing
        result = self.validator.run_concurrent_validation(
            test_operation,
            num_threads=2,
            iterations_per_thread=3
        )

        # Assert basic structure
        self.assertTrue(result["success"])
        self.assertEqual(result["total_iterations"], 6)  # 2 threads * 3 iterations
        self.assertEqual(result["thread_config"]["num_threads"], 2)
        self.assertEqual(result["thread_config"]["iterations_per_thread"], 3)
        self.assertEqual(len(result["individual_results"]), 6)

        # Check that latency stats are calculated
        latency_stats = result["latency_stats"]
        self.assertIn("avg_ms", latency_stats)
        self.assertIn("p95_ms", latency_stats)
        self.assertIn("p99_ms", latency_stats)

    def test_measure_throughput(self):
        """
        Test measuring throughput of an operation.
        """
        # Define a test operation
        def test_operation():
            time.sleep(0.001)  # Small delay
            return "result"

        # Execute with a short time limit for testing
        result = self.validator.measure_throughput(
            test_operation,
            time_limit_seconds=0.1  # Very short time limit for testing
        )

        # Assert basic structure
        self.assertTrue(result["success"])
        self.assertGreaterEqual(result["operations_completed"], 0)
        self.assertIsNotNone(result["throughput_ops_per_sec"])
        self.assertIn("meets_target", result)

    def test_validate_performance_thresholds_all_passed(self):
        """
        Test validating performance thresholds when all pass.
        """
        # Create metrics that should pass thresholds
        test_metrics = {
            "latency_stats": {
                "avg_ms": 100,  # Below 500ms threshold
                "p95_ms": 200   # Below 750ms threshold (1.5 * 500)
            },
            "throughput_ops_per_sec": 20  # Above 10 ops/sec threshold
        }

        # Execute
        result = self.validator.validate_performance_thresholds(test_metrics)

        # Assert
        self.assertTrue(result["success"])
        self.assertTrue(result["all_thresholds_passed"])
        self.assertGreater(result["overall_score"], 0)

        # Check individual threshold results
        threshold_results = result["threshold_results"]
        self.assertIn("max_latency", threshold_results)
        self.assertIn("min_throughput", threshold_results)
        self.assertIn("p95_latency", threshold_results)

        # Verify that all thresholds passed
        for threshold_name, threshold_result in threshold_results.items():
            self.assertTrue(threshold_result["passed"], f"Threshold {threshold_name} should have passed")

    def test_validate_performance_thresholds_some_failed(self):
        """
        Test validating performance thresholds when some fail.
        """
        # Create metrics that should fail some thresholds
        test_metrics = {
            "latency_stats": {
                "avg_ms": 600,  # Above 500ms threshold
                "p95_ms": 800   # Above 750ms threshold (1.5 * 500)
            },
            "throughput_ops_per_sec": 5  # Below 10 ops/sec threshold
        }

        # Execute
        result = self.validator.validate_performance_thresholds(test_metrics)

        # Assert
        self.assertTrue(result["success"])
        self.assertFalse(result["all_thresholds_passed"])
        self.assertLess(result["overall_score"], 1.0)

        # Check individual threshold results
        threshold_results = result["threshold_results"]
        self.assertIn("max_latency", threshold_results)
        self.assertIn("min_throughput", threshold_results)
        self.assertIn("p95_latency", threshold_results)

        # Verify that some thresholds failed
        failed_count = sum(1 for tr in threshold_results.values() if not tr["passed"])
        self.assertGreater(failed_count, 0)

    def test_run_complete_performance_validation(self):
        """
        Test the complete performance validation workflow.
        """
        # Define test operations
        test_operations = [
            {
                "name": "Test Op 1",
                "operation": lambda: time.sleep(0.001) or "result1",
                "args": (),
                "kwargs": {}
            },
            {
                "name": "Test Op 2",
                "operation": lambda: time.sleep(0.001) or "result2",
                "args": (),
                "kwargs": {}
            }
        ]

        # Execute
        result = self.validator.run_complete_performance_validation(test_operations)

        # Assert structure
        self.assertIn("overall_success", result)
        self.assertIn("latency_measurements", result)
        self.assertIn("throughput_measurements", result)
        self.assertIn("concurrent_validation_results", result)
        self.assertIn("threshold_validations", result)

        # Check that we have results for each test operation
        self.assertEqual(len(result["latency_measurements"]), 2)
        self.assertEqual(len(result["throughput_measurements"]), 2)
        self.assertEqual(len(result["concurrent_validation_results"]), 2)
        self.assertEqual(len(result["threshold_validations"]), 2)

        # Check overall success structure
        overall = result["overall_success"]
        self.assertIn("latency_tests_passed", overall)
        self.assertIn("throughput_tests_passed", overall)
        self.assertIn("threshold_tests_passed", overall)


if __name__ == '__main__':
    unittest.main()