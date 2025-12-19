"""
Performance validator for RAG validation system.
Validates retrieval latency and performance metrics to ensure system meets response time requirements.
"""
import time
import statistics
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import List, Dict, Any, Optional, Callable
from backend.rag_validation.config_loader import ConfigLoader
from backend.rag_validation.logger import performance_logger
from backend.rag_validation.error_handler import ErrorHandler, handle_validation_error
from backend.rag_validation.performance_utils import PerformanceMetrics, PerformanceTimer, record_performance
from backend.rag_validation.test_data import TestDataGenerator


class PerformanceValidator:
    """
    Validator for performance metrics including retrieval latency, throughput, and threshold validation.
    """

    def __init__(self, config_loader: ConfigLoader = None):
        """
        Initialize the performance validator.

        Args:
            config_loader: Configuration loader instance
        """
        self.config_loader = config_loader or ConfigLoader()
        self.config = self.config_loader.get_config()
        self.error_handler = ErrorHandler(performance_logger)

        # Get performance thresholds from config
        self.thresholds = self.config.get("validation_thresholds", {})
        self.max_latency_ms = self.thresholds.get("max_latency_ms", 500)
        self.min_throughput_per_sec = self.thresholds.get("min_throughput_per_sec", 10)

        performance_logger.info(
            "PerformanceValidator initialized",
            extra={'extra_data': {
                'max_latency_ms': self.max_latency_ms,
                'min_throughput_per_sec': self.min_throughput_per_sec
            }}
        )

    def measure_retrieval_latency(self, operation: Callable, *args, **kwargs) -> Dict[str, Any]:
        """
        Measure the latency of a retrieval operation.

        Args:
            operation: The operation to measure
            *args: Arguments to pass to the operation
            **kwargs: Keyword arguments to pass to the operation

        Returns:
            Dictionary with latency measurement results
        """
        timer = PerformanceTimer()
        timer.start()

        try:
            result = operation(*args, **kwargs)
            duration = timer.stop()

            latency_result = {
                "success": True,
                "operation_result": result,
                "latency_ms": duration,
                "within_threshold": duration <= self.max_latency_ms,
                "threshold_ms": self.max_latency_ms
            }

            record_performance("measure_retrieval_latency", duration)

            return latency_result

        except Exception as e:
            duration = timer.stop()
            error_info = handle_validation_error(e, "performance_validator", performance_logger)
            return {
                "success": False,
                "error": str(e),
                "error_info": error_info,
                "latency_ms": duration,
                "duration_ms": duration
            }

    def calculate_percentile_latency(self, latencies: List[float], percentile: float) -> float:
        """
        Calculate percentile latency from a list of latency measurements.

        Args:
            latencies: List of latency measurements in milliseconds
            percentile: Percentile to calculate (e.g., 95 for p95, 99 for p99)

        Returns:
            Calculated percentile latency
        """
        try:
            if not latencies:
                return 0.0

            sorted_latencies = sorted(latencies)
            index = int((percentile / 100) * len(sorted_latencies))
            index = min(index, len(sorted_latencies) - 1)  # Ensure index is within bounds

            result = sorted_latencies[index] if sorted_latencies else 0.0

            return result

        except Exception as e:
            error_info = handle_validation_error(e, "performance_validator", performance_logger)
            performance_logger.error(f"Error calculating percentile latency: {str(e)}", extra={'extra_data': error_info})
            return 0.0

    def run_concurrent_validation(self, operation: Callable, num_threads: int = 5, iterations_per_thread: int = 10, *args, **kwargs) -> Dict[str, Any]:
        """
        Run concurrent validation testing to measure performance under load.

        Args:
            operation: The operation to test
            num_threads: Number of concurrent threads
            iterations_per_thread: Number of iterations per thread
            *args: Arguments to pass to the operation
            **kwargs: Keyword arguments to pass to the operation

        Returns:
            Dictionary with concurrent validation results
        """
        start_time = time.time()

        latencies = []
        errors = []
        results = []

        def run_operation(iteration_id: int) -> Dict[str, Any]:
            timer = PerformanceTimer()
            timer.start()

            try:
                result = operation(*args, **kwargs)
                duration = timer.stop()

                return {
                    "iteration": iteration_id,
                    "success": True,
                    "latency_ms": duration,
                    "result": result
                }
            except Exception as e:
                duration = timer.stop()
                error_info = handle_validation_error(e, "performance_validator", performance_logger)
                return {
                    "iteration": iteration_id,
                    "success": False,
                    "latency_ms": duration,
                    "error": str(e),
                    "error_info": error_info
                }

        # Execute operations concurrently
        with ThreadPoolExecutor(max_workers=num_threads) as executor:
            futures = []
            total_iterations = num_threads * iterations_per_thread

            for thread_id in range(num_threads):
                for iter_id in range(iterations_per_thread):
                    iteration_id = thread_id * iterations_per_thread + iter_id
                    future = executor.submit(run_operation, iteration_id)
                    futures.append(future)

            for future in as_completed(futures):
                result = future.result()
                results.append(result)

                if result["success"]:
                    latencies.append(result["latency_ms"])
                else:
                    errors.append(result)

        total_time = time.time() - start_time
        total_completed = len([r for r in results if r["success"]])

        # Calculate performance metrics
        if latencies:
            avg_latency = sum(latencies) / len(latencies)
            p95_latency = self.calculate_percentile_latency(latencies, 95)
            p99_latency = self.calculate_percentile_latency(latencies, 99)
            min_latency = min(latencies)
            max_latency = max(latencies)
        else:
            avg_latency = p95_latency = p99_latency = min_latency = max_latency = 0

        throughput = total_completed / total_time if total_time > 0 else 0

        result = {
            "success": True,
            "total_iterations": total_iterations,
            "total_completed": total_completed,
            "total_errors": len(errors),
            "total_time_seconds": total_time,
            "throughput_ops_per_sec": throughput,
            "latency_stats": {
                "avg_ms": avg_latency,
                "p95_ms": p95_latency,
                "p99_ms": p99_latency,
                "min_ms": min_latency,
                "max_ms": max_latency
            },
            "thread_config": {
                "num_threads": num_threads,
                "iterations_per_thread": iterations_per_thread
            },
            "individual_results": results
        }

        record_performance("run_concurrent_validation", total_time * 1000)

        return result

    def measure_throughput(self, operation: Callable, time_limit_seconds: float = 30.0, *args, **kwargs) -> Dict[str, Any]:
        """
        Measure the throughput of an operation (operations per second).

        Args:
            operation: The operation to measure
            time_limit_seconds: Time limit for the measurement in seconds
            *args: Arguments to pass to the operation
            **kwargs: Keyword arguments to pass to the operation

        Returns:
            Dictionary with throughput measurement results
        """
        start_time = time.time()
        operations_completed = 0
        latencies = []
        errors = []

        while time.time() - start_time < time_limit_seconds:
            timer = PerformanceTimer()
            timer.start()

            try:
                result = operation(*args, **kwargs)
                duration = timer.stop()

                operations_completed += 1
                latencies.append(duration)

                # Small delay to prevent overwhelming the system
                time.sleep(0.001)

            except Exception as e:
                duration = timer.stop()
                error_info = handle_validation_error(e, "performance_validator", performance_logger)
                errors.append({
                    "error": str(e),
                    "error_info": error_info,
                    "latency_ms": duration
                })

        total_time = time.time() - start_time
        throughput = operations_completed / total_time if total_time > 0 else 0

        result = {
            "success": True,
            "operations_completed": operations_completed,
            "total_time_seconds": total_time,
            "throughput_ops_per_sec": throughput,
            "target_throughput": self.min_throughput_per_sec,
            "meets_target": throughput >= self.min_throughput_per_sec,
            "errors": len(errors)
        }

        record_performance("measure_throughput", total_time * 1000)

        return result

    def validate_performance_thresholds(self, metrics: Dict[str, Any]) -> Dict[str, Any]:
        """
        Validate performance against defined thresholds.

        Args:
            metrics: Dictionary containing performance metrics to validate

        Returns:
            Dictionary with validation results
        """
        try:
            # Check various performance thresholds
            threshold_results = {}

            # Check max latency
            if "latency_stats" in metrics and "avg_ms" in metrics["latency_stats"]:
                avg_latency = metrics["latency_stats"]["avg_ms"]
                threshold_results["max_latency"] = {
                    "value": avg_latency,
                    "threshold": self.max_latency_ms,
                    "passed": avg_latency <= self.max_latency_ms
                }

            # Check throughput
            if "throughput_ops_per_sec" in metrics:
                throughput = metrics["throughput_ops_per_sec"]
                threshold_results["min_throughput"] = {
                    "value": throughput,
                    "threshold": self.min_throughput_per_sec,
                    "passed": throughput >= self.min_throughput_per_sec
                }

            # Check p95 latency if available
            if "latency_stats" in metrics and "p95_ms" in metrics["latency_stats"]:
                p95_latency = metrics["latency_stats"]["p95_ms"]
                # Using 1.5x the max latency as p95 threshold
                p95_threshold = self.max_latency_ms * 1.5
                threshold_results["p95_latency"] = {
                    "value": p95_latency,
                    "threshold": p95_threshold,
                    "passed": p95_latency <= p95_threshold
                }

            all_passed = all(result["passed"] for result in threshold_results.values())

            result = {
                "success": True,
                "all_thresholds_passed": all_passed,
                "threshold_results": threshold_results,
                "overall_score": sum(1 for r in threshold_results.values() if r["passed"]) / len(threshold_results) if threshold_results else 0
            }

            return result

        except Exception as e:
            error_info = handle_validation_error(e, "performance_validator", performance_logger)
            return {
                "success": False,
                "error": str(e),
                "error_info": error_info
            }

    def run_complete_performance_validation(self, test_operations: List[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Run complete performance validation workflow.

        Args:
            test_operations: List of operations to test (will generate if not provided)

        Returns:
            Dictionary with complete validation results
        """
        if test_operations is None:
            # Generate basic test operations using test data
            test_data = TestDataGenerator.generate_sample_texts(3)
            test_operations = []
            for i, text in enumerate(test_data):
                test_operations.append({
                    "name": f"Test Operation {i}",
                    "operation": lambda x=text: len(x),  # Simple test operation
                    "args": (),
                    "kwargs": {}
                })

        results = {
            "validation_steps": [],
            "latency_measurements": [],
            "throughput_measurements": [],
            "concurrent_validation_results": [],
            "threshold_validations": []
        }

        for op_data in test_operations:
            op_name = op_data["name"]
            operation = op_data["operation"]
            args = op_data.get("args", ())
            kwargs = op_data.get("kwargs", {})

            # Measure basic latency
            latency_result = self.measure_retrieval_latency(operation, *args, **kwargs)
            results["latency_measurements"].append({
                "operation_name": op_name,
                "result": latency_result
            })

            # Measure throughput
            throughput_result = self.measure_throughput(operation, time_limit_seconds=10.0, *args, **kwargs)
            results["throughput_measurements"].append({
                "operation_name": op_name,
                "result": throughput_result
            })

            # Run concurrent validation
            concurrent_result = self.run_concurrent_validation(
                operation, num_threads=3, iterations_per_thread=5, *args, **kwargs
            )
            results["concurrent_validation_results"].append({
                "operation_name": op_name,
                "result": concurrent_result
            })

            # Validate against thresholds
            threshold_result = self.validate_performance_thresholds(concurrent_result)
            results["threshold_validations"].append({
                "operation_name": op_name,
                "result": threshold_result
            })

        # Calculate overall success metrics
        total_latency_tests = len(results["latency_measurements"])
        passed_latency_tests = sum(1 for r in results["latency_measurements"]
                                  if r["result"].get("within_threshold", True))

        total_throughput_tests = len(results["throughput_measurements"])
        passed_throughput_tests = sum(1 for r in results["throughput_measurements"]
                                     if r["result"].get("meets_target", True))

        total_threshold_tests = len(results["threshold_validations"])
        passed_threshold_tests = sum(1 for r in results["threshold_validations"]
                                    if r["result"].get("all_thresholds_passed", False))

        results["overall_success"] = {
            "latency_tests_passed": passed_latency_tests / total_latency_tests if total_latency_tests > 0 else 0,
            "throughput_tests_passed": passed_throughput_tests / total_throughput_tests if total_throughput_tests > 0 else 0,
            "threshold_tests_passed": passed_threshold_tests / total_threshold_tests if total_threshold_tests > 0 else 0,
            "all_tests_passed": (
                passed_latency_tests == total_latency_tests and
                passed_throughput_tests == total_throughput_tests and
                passed_threshold_tests == total_threshold_tests
            )
        }

        return results


def create_performance_validator(config_loader: ConfigLoader = None) -> PerformanceValidator:
    """
    Create and return a performance validator instance.

    Args:
        config_loader: Optional configuration loader instance

    Returns:
        PerformanceValidator instance
    """
    return PerformanceValidator(config_loader)