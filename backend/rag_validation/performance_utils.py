"""
Performance utilities for RAG validation system.
Contains time measurement, latency tracking, and performance metrics functions.
"""
import time
import statistics
from typing import List, Callable, Any, Dict, Optional
from contextlib import contextmanager
import functools


class PerformanceTimer:
    """
    A utility class for measuring execution time of operations.
    """

    def __init__(self):
        self.start_time = None
        self.end_time = None
        self.elapsed_time = 0

    def start(self) -> None:
        """
        Start the timer.
        """
        self.start_time = time.perf_counter()
        self.end_time = None
        self.elapsed_time = 0

    def stop(self) -> float:
        """
        Stop the timer and return elapsed time in milliseconds.

        Returns:
            Elapsed time in milliseconds
        """
        if self.start_time is None:
            raise ValueError("Timer not started. Call start() first.")

        self.end_time = time.perf_counter()
        self.elapsed_time = (self.end_time - self.start_time) * 1000  # Convert to milliseconds
        return self.elapsed_time

    def elapsed_ms(self) -> float:
        """
        Get the elapsed time in milliseconds.
        If timer is still running, returns time since start.

        Returns:
            Elapsed time in milliseconds
        """
        if self.start_time is None:
            return 0

        if self.end_time is None:
            # Timer is still running
            return (time.perf_counter() - self.start_time) * 1000
        else:
            return self.elapsed_time


@contextmanager
def time_it(operation_name: str = "Operation"):
    """
    Context manager to time an operation.

    Args:
        operation_name: Name of the operation being timed

    Yields:
        Timer object to allow for manual timing if needed
    """
    timer = PerformanceTimer()
    timer.start()
    try:
        yield timer
    finally:
        elapsed = timer.stop()
        print(f"{operation_name} took {elapsed:.2f} ms")


def time_function(func: Callable) -> Callable:
    """
    Decorator to time function execution.

    Args:
        func: Function to time

    Returns:
        Wrapped function with timing
    """
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        start_time = time.perf_counter()
        result = func(*args, **kwargs)
        end_time = time.perf_counter()
        elapsed_ms = (end_time - start_time) * 1000
        print(f"{func.__name__} took {elapsed_ms:.2f} ms")
        return result
    return wrapper


class PerformanceMetrics:
    """
    Class for collecting and analyzing performance metrics.
    """

    def __init__(self):
        self.timings: List[float] = []
        self.operation_counts: Dict[str, int] = {}
        self.operation_timings: Dict[str, List[float]] = {}

    def record_timing(self, operation: str, duration_ms: float) -> None:
        """
        Record a timing for an operation.

        Args:
            operation: Name of the operation
            duration_ms: Duration in milliseconds
        """
        self.timings.append(duration_ms)

        # Update operation-specific metrics
        if operation not in self.operation_timings:
            self.operation_timings[operation] = []
        self.operation_timings[operation].append(duration_ms)

        # Update operation count
        self.operation_counts[operation] = self.operation_counts.get(operation, 0) + 1

    def get_total_count(self) -> int:
        """
        Get the total number of recorded timings.

        Returns:
            Total count of recorded timings
        """
        return len(self.timings)

    def get_total_time(self) -> float:
        """
        Get the total time across all operations.

        Returns:
            Total time in milliseconds
        """
        return sum(self.timings) if self.timings else 0

    def get_average_time(self) -> float:
        """
        Get the average time across all operations.

        Returns:
            Average time in milliseconds
        """
        return statistics.mean(self.timings) if self.timings else 0

    def get_median_time(self) -> float:
        """
        Get the median time across all operations.

        Returns:
            Median time in milliseconds
        """
        return statistics.median(self.timings) if self.timings else 0

    def get_p95_time(self) -> float:
        """
        Get the 95th percentile time across all operations.

        Returns:
            95th percentile time in milliseconds
        """
        if not self.timings:
            return 0
        sorted_timings = sorted(self.timings)
        index = int(0.95 * len(sorted_timings))
        return sorted_timings[index] if index < len(sorted_timings) else sorted_timings[-1]

    def get_p99_time(self) -> float:
        """
        Get the 99th percentile time across all operations.

        Returns:
            99th percentile time in milliseconds
        """
        if not self.timings:
            return 0
        sorted_timings = sorted(self.timings)
        index = int(0.99 * len(sorted_timings))
        return sorted_timings[index] if index < len(sorted_timings) else sorted_timings[-1]

    def get_min_time(self) -> float:
        """
        Get the minimum time across all operations.

        Returns:
            Minimum time in milliseconds
        """
        return min(self.timings) if self.timings else 0

    def get_max_time(self) -> float:
        """
        Get the maximum time across all operations.

        Returns:
            Maximum time in milliseconds
        """
        return max(self.timings) if self.timings else 0

    def get_operation_stats(self, operation: str) -> Dict[str, float]:
        """
        Get statistics for a specific operation.

        Args:
            operation: Name of the operation

        Returns:
            Dictionary with operation statistics
        """
        if operation not in self.operation_timings or not self.operation_timings[operation]:
            return {}

        timings = self.operation_timings[operation]
        sorted_timings = sorted(timings)

        stats = {
            'count': len(timings),
            'average': statistics.mean(timings),
            'median': statistics.median(timings),
            'min': min(timings),
            'max': max(timings),
            'p95': sorted_timings[int(0.95 * len(sorted_timings))] if sorted_timings else 0,
            'p99': sorted_timings[int(0.99 * len(sorted_timings))] if sorted_timings else 0,
            'total': sum(timings)
        }

        return stats

    def get_all_operation_stats(self) -> Dict[str, Dict[str, float]]:
        """
        Get statistics for all operations.

        Returns:
            Dictionary mapping operation names to their statistics
        """
        all_stats = {}
        for operation in self.operation_timings:
            all_stats[operation] = self.get_operation_stats(operation)
        return all_stats

    def reset(self) -> None:
        """
        Reset all collected metrics.
        """
        self.timings = []
        self.operation_counts = {}
        self.operation_timings = {}


def measure_concurrent_performance(
    func: Callable,
    args_list: List[tuple],
    kwargs_list: List[dict] = None,
    num_workers: int = 4
) -> List[float]:
    """
    Measure performance of a function under concurrent load.

    Args:
        func: Function to measure
        args_list: List of argument tuples to pass to the function
        kwargs_list: List of keyword argument dicts to pass to the function
        num_workers: Number of concurrent workers (not used in this simple implementation)

    Returns:
        List of execution times in milliseconds
    """
    if kwargs_list is None:
        kwargs_list = [{}] * len(args_list)

    if len(args_list) != len(kwargs_list):
        raise ValueError("args_list and kwargs_list must have the same length")

    timings = []
    for args, kwargs in zip(args_list, kwargs_list):
        start_time = time.perf_counter()
        func(*args, **kwargs)
        end_time = time.perf_counter()
        elapsed_ms = (end_time - start_time) * 1000
        timings.append(elapsed_ms)

    return timings


def throughput_calculator(
    operation: Callable,
    iterations: int,
    *args,
    **kwargs
) -> Dict[str, float]:
    """
    Calculate throughput metrics for an operation.

    Args:
        operation: Operation to measure
        iterations: Number of iterations to run
        *args: Arguments to pass to the operation
        **kwargs: Keyword arguments to pass to the operation

    Returns:
        Dictionary with throughput metrics
    """
    start_time = time.perf_counter()

    for _ in range(iterations):
        operation(*args, **kwargs)

    end_time = time.perf_counter()
    total_time = end_time - start_time

    return {
        'total_time_seconds': total_time,
        'operations_per_second': iterations / total_time if total_time > 0 else 0,
        'average_time_per_operation_ms': (total_time / iterations) * 1000 if iterations > 0 else 0,
        'total_operations': iterations
    }


def validate_performance_threshold(
    operation: Callable,
    threshold_ms: float,
    *args,
    **kwargs
) -> Dict[str, Any]:
    """
    Validate that an operation completes within a performance threshold.

    Args:
        operation: Operation to validate
        threshold_ms: Maximum allowed time in milliseconds
        *args: Arguments to pass to the operation
        **kwargs: Keyword arguments to pass to the operation

    Returns:
        Dictionary with validation results
    """
    start_time = time.perf_counter()

    try:
        result = operation(*args, **kwargs)
        end_time = time.perf_counter()
        elapsed_ms = (end_time - start_time) * 1000

        within_threshold = elapsed_ms <= threshold_ms

        return {
            'success': within_threshold,
            'elapsed_ms': elapsed_ms,
            'threshold_ms': threshold_ms,
            'within_threshold': within_threshold,
            'result': result
        }
    except Exception as e:
        return {
            'success': False,
            'error': str(e),
            'within_threshold': False,
            'result': None
        }


# Global performance metrics instance
global_metrics = PerformanceMetrics()


def record_performance(operation: str, duration_ms: float) -> None:
    """
    Record a performance metric using the global metrics instance.

    Args:
        operation: Name of the operation
        duration_ms: Duration in milliseconds
    """
    global_metrics.record_timing(operation, duration_ms)


def get_global_metrics() -> PerformanceMetrics:
    """
    Get the global performance metrics instance.

    Returns:
        Global PerformanceMetrics instance
    """
    return global_metrics