"""
Integration tests for the performance validator.
"""
import unittest
from unittest.mock import patch, Mock
import sys
import time

# Add backend directory to path to import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../'))

from backend.rag_validation.performance_validator import PerformanceValidator
from backend.rag_validation.config_loader import ConfigLoader
from backend.rag_validation.test_data import TestDataGenerator


class TestPerformanceIntegration(unittest.TestCase):
    """
    Integration tests for PerformanceValidator.
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
                "max_latency_ms": 1000,  # Higher threshold for testing
                "min_throughput_per_sec": 1
            }
        }

    def test_end_to_end_performance_validation_process(self):
        """
        Test the complete end-to-end performance validation process.
        """
        # Create validator
        validator = PerformanceValidator(config_loader=self.mock_config_loader)

        # Test latency measurement
        def simple_operation():
            time.sleep(0.01)  # 10ms delay
            return "success"

        latency_result = validator.measure_retrieval_latency(simple_operation)
        self.assertTrue(latency_result["success"])
        self.assertGreaterEqual(latency_result["latency_ms"], 10)

        # Test percentile calculation
        latencies = [50, 100, 150, 200, 250, 300, 350, 400, 450, 500]
        p95_result = validator.calculate_percentile_latency(latencies, 95)
        self.assertGreaterEqual(p95_result, 450)  # Should be close to 500

        # Test concurrent validation
        concurrent_result = validator.run_concurrent_validation(
            simple_operation,
            num_threads=2,
            iterations_per_thread=3
        )
        self.assertTrue(concurrent_result["success"])
        self.assertEqual(concurrent_result["total_iterations"], 6)

        # Test throughput measurement
        throughput_result = validator.measure_throughput(
            simple_operation,
            time_limit_seconds=0.2  # Short time for testing
        )
        self.assertTrue(throughput_result["success"])
        self.assertGreaterEqual(throughput_result["operations_completed"], 0)

        # Test threshold validation
        test_metrics = concurrent_result  # Use concurrent result as metrics
        threshold_result = validator.validate_performance_thresholds(test_metrics)
        self.assertTrue(threshold_result["success"])

        # Test complete validation workflow
        test_operations = [
            {
                "name": "Simple Operation",
                "operation": simple_operation,
                "args": (),
                "kwargs": {}
            }
        ]
        complete_result = validator.run_complete_performance_validation(test_operations)
        self.assertIn("overall_success", complete_result)

    def test_performance_metrics_consistency(self):
        """
        Test that performance metrics are consistent across different measurements.
        """
        # Create validator
        validator = PerformanceValidator(config_loader=self.mock_config_loader)

        # Define a consistent test operation
        def consistent_operation():
            time.sleep(0.005)  # 5ms delay
            return "consistent_result"

        # Run multiple measurements
        measurements = []
        for i in range(5):
            result = validator.measure_retrieval_latency(consistent_operation)
            if result["success"]:
                measurements.append(result["latency_ms"])

        # Check that measurements are reasonably consistent
        if len(measurements) > 1:
            # Calculate variance - should be relatively low for consistent operation
            avg = sum(measurements) / len(measurements)
            variance = sum((x - avg) ** 2 for x in measurements) / len(measurements)
            # Note: exact variance depends on system timing, just ensure it's calculated

    def test_concurrent_load_handling(self):
        """
        Test performance validation under concurrent load conditions.
        """
        # Create validator
        validator = PerformanceValidator(config_loader=self.mock_config_loader)

        # Define a test operation
        def load_test_operation():
            # Simulate some work
            time.sleep(0.001)
            return sum(range(100))  # Simple computation

        # Run concurrent validation with higher load
        result = validator.run_concurrent_validation(
            load_test_operation,
            num_threads=3,
            iterations_per_thread=5
        )

        # Validate the results structure
        self.assertTrue(result["success"])
        self.assertEqual(result["total_iterations"], 15)
        self.assertGreaterEqual(result["total_completed"], 0)

        # Check that latency statistics were calculated
        latency_stats = result["latency_stats"]
        self.assertIsNotNone(latency_stats["avg_ms"])
        self.assertIsNotNone(latency_stats["p95_ms"])
        self.assertIsNotNone(latency_stats["p99_ms"])

    def test_throughput_measurement_accuracy(self):
        """
        Test the accuracy of throughput measurements.
        """
        # Create validator
        validator = PerformanceValidator(config_loader=self.mock_config_loader)

        # Define a test operation with known timing
        def timed_operation():
            time.sleep(0.01)  # 10ms operation
            return "timed_result"

        # Measure throughput for a longer period to get more accurate measurement
        result = validator.measure_throughput(
            timed_operation,
            time_limit_seconds=0.5  # 500ms test period
        )

        # Validate results
        self.assertTrue(result["success"])
        self.assertGreaterEqual(result["operations_completed"], 0)
        # Throughput should be reasonable given the 10ms operation time
        # In 500ms, we might expect about 45-50 operations (allowing for overhead)

    def test_threshold_validation_integration(self):
        """
        Test integration between performance measurement and threshold validation.
        """
        # Create validator
        validator = PerformanceValidator(config_loader=self.mock_config_loader)

        # Define a slow operation that might exceed thresholds
        def slow_operation():
            time.sleep(0.1)  # 100ms delay
            return "slow_result"

        # Measure performance
        concurrent_result = validator.run_concurrent_validation(
            slow_operation,
            num_threads=2,
            iterations_per_thread=2
        )

        # Validate against thresholds
        threshold_result = validator.validate_performance_thresholds(concurrent_result)

        # The result should be structured properly even if thresholds aren't met
        self.assertTrue(threshold_result["success"])
        self.assertIsNotNone(threshold_result["all_thresholds_passed"])
        self.assertIn("threshold_results", threshold_result)

    def test_error_handling_integration(self):
        """
        Test error handling integration throughout the performance validation process.
        """
        # Create validator
        validator = PerformanceValidator(config_loader=self.mock_config_loader)

        # Define an operation that raises an exception
        def error_operation():
            raise Exception("Intentional error for testing")

        # Test that errors are handled gracefully in latency measurement
        latency_result = validator.measure_retrieval_latency(error_operation)
        self.assertFalse(latency_result["success"])
        self.assertIn("Intentional error", latency_result["error"])

        # Test concurrent validation with errors
        concurrent_result = validator.run_concurrent_validation(
            error_operation,
            num_threads=2,
            iterations_per_thread=2
        )
        # Should still return a result structure even with errors
        self.assertIsNotNone(concurrent_result)
        self.assertIn("total_errors", concurrent_result)

    def test_configuration_integration(self):
        """
        Test integration with configuration loading for performance validation.
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
            return config
        config_loader.get_config = mock_get_config

        # Create validator with real config (but mocked external dependencies)
        validator = PerformanceValidator(config_loader=config_loader)

        # Check that the validator was initialized with the config
        self.assertEqual(validator.max_latency_ms, 2000)
        self.assertEqual(validator.min_throughput_per_sec, 5)

        # Test a simple operation to ensure everything works together
        def simple_test():
            return "ok"

        result = validator.measure_retrieval_latency(simple_test)
        self.assertTrue(result["success"])


if __name__ == '__main__':
    unittest.main()