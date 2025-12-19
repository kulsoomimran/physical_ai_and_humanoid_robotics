from typing import Any, Dict, Optional, Callable, Union
import traceback
import time
import logging
from enum import Enum


class ErrorSeverity(Enum):
    """
    Enum for error severity levels.
    """
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"
    CRITICAL = "critical"


class RAGValidationError(Exception):
    """
    Base exception class for RAG validation errors.
    """
    def __init__(self, message: str, error_type: str = "validation_error", component: str = ""):
        super().__init__(message)
        self.error_type = error_type
        self.component = component
        self.timestamp = time.time()


class ErrorHandler:
    """
    Utility class for handling and logging errors in the RAG validation system.
    """

    def __init__(self, logger: logging.Logger = None):
        """
        Initialize the error handler.

        Args:
            logger: Optional logger instance to use for error logging
        """
        self.logger = logger

    def handle_error(
        self,
        error: Exception,
        component: str,
        severity: ErrorSeverity = ErrorSeverity.MEDIUM,
        context: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """
        Handle an error by logging it and returning structured error information.

        Args:
            error: The exception that occurred
            component: Component where the error occurred
            severity: Severity level of the error
            context: Additional context information about the error

        Returns:
            Dictionary with error information
        """
        error_info = {
            "error_id": f"err_{int(time.time())}_{hash(str(error)) % 10000}",
            "timestamp": time.time(),
            "component": component,
            "error_type": type(error).__name__,
            "severity": severity.value,
            "message": str(error),
            "traceback": traceback.format_exc() if not isinstance(error, RAGValidationError) else None,
            "context": context or {},
            "handled": True
        }

        # Log the error if logger is provided
        if self.logger:
            extra_data = {
                "error_info": error_info
            }
            self.logger.error(
                f"Error in {component}: {str(error)}",
                extra={"extra_data": extra_data}
            )

        return error_info

    def retry_on_failure(
        self,
        func: Callable,
        max_retries: int = 3,
        delay: float = 1.0,
        backoff: float = 2.0,
        component: str = "",
        exceptions: tuple = (Exception,)
    ) -> Any:
        """
        Execute a function with retry logic on failure.

        Args:
            func: Function to execute
            max_retries: Maximum number of retry attempts
            delay: Initial delay between retries (in seconds)
            backoff: Multiplier for delay after each retry
            component: Component name for error logging
            exceptions: Tuple of exceptions to catch and retry on

        Returns:
            Result of the function execution
        """
        current_delay = delay
        last_exception = None

        for attempt in range(max_retries + 1):
            try:
                return func()
            except exceptions as e:
                last_exception = e

                # Log the error if it's not the last attempt
                if attempt < max_retries:
                    error_info = self.handle_error(
                        e,
                        component,
                        ErrorSeverity.LOW if attempt < max_retries - 1 else ErrorSeverity.MEDIUM,
                        {"attempt": attempt + 1, "max_retries": max_retries}
                    )

                    # Wait before retrying
                    time.sleep(current_delay)
                    current_delay *= backoff
                else:
                    # On the final attempt, log as a higher severity error
                    error_info = self.handle_error(
                        e,
                        component,
                        ErrorSeverity.HIGH,
                        {"attempt": attempt + 1, "max_retries": max_retries}
                    )

        # If all retries failed, raise the last exception
        raise last_exception

    def validate_and_handle(
        self,
        value: Any,
        validator: Callable[[Any], bool],
        error_message: str,
        component: str,
        severity: ErrorSeverity = ErrorSeverity.MEDIUM
    ) -> bool:
        """
        Validate a value and handle validation errors.

        Args:
            value: Value to validate
            validator: Function that returns True if value is valid
            error_message: Error message to use if validation fails
            component: Component name for error logging
            severity: Severity level for the error

        Returns:
            True if validation passes, False otherwise
        """
        try:
            if validator(value):
                return True
            else:
                error = RAGValidationError(error_message, "validation_error", component)
                self.handle_error(error, component, severity)
                return False
        except Exception as e:
            self.handle_error(e, component, severity)
            return False

    def safe_execute(
        self,
        func: Callable,
        default_value: Any = None,
        component: str = "",
        suppress_error: bool = True
    ) -> Any:
        """
        Safely execute a function, catching exceptions and returning a default value.

        Args:
            func: Function to execute
            default_value: Value to return if function fails
            component: Component name for error logging
            suppress_error: Whether to suppress the error or re-raise it

        Returns:
            Result of function execution or default value if it fails
        """
        try:
            return func()
        except Exception as e:
            error_info = self.handle_error(e, component, ErrorSeverity.LOW)
            if not suppress_error:
                raise
            return default_value


def create_error_handler(logger: logging.Logger = None) -> ErrorHandler:
    """
    Create and return an error handler instance.

    Args:
        logger: Optional logger instance to use

    Returns:
        ErrorHandler instance
    """
    return ErrorHandler(logger)


def handle_validation_error(
    error: Exception,
    component: str,
    logger: logging.Logger = None
) -> Dict[str, Any]:
    """
    Convenience function to handle a validation error.

    Args:
        error: The exception that occurred
        component: Component where the error occurred
        logger: Optional logger instance to use

    Returns:
        Dictionary with error information
    """
    handler = ErrorHandler(logger)
    return handler.handle_error(error, component)


def retry_validation_call(
    func: Callable,
    max_retries: int = 3,
    delay: float = 1.0,
    component: str = "",
    logger: logging.Logger = None
) -> Any:
    """
    Convenience function to execute a validation call with retry logic.

    Args:
        func: Function to execute
        max_retries: Maximum number of retry attempts
        delay: Initial delay between retries (in seconds)
        component: Component name for error logging
        logger: Optional logger instance to use

    Returns:
        Result of the function execution
    """
    handler = ErrorHandler(logger)
    return handler.retry_on_failure(func, max_retries, delay, component=component)