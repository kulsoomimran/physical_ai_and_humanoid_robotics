import logging
import json
import sys
from datetime import datetime
from typing import Dict, Any
import os

class JsonFormatter(logging.Formatter):
    """
    Custom JSON formatter for structured logging.
    """

    def format(self, record: logging.LogRecord) -> str:
        """
        Format the log record as JSON.

        Args:
            record: Log record to format

        Returns:
            JSON formatted log string
        """
        log_entry = {
            'timestamp': datetime.utcnow().isoformat(),
            'level': record.levelname,
            'logger': record.name,
            'message': record.getMessage(),
            'module': record.module,
            'function': record.funcName,
            'line': record.lineno,
        }

        # Add exception info if present
        if record.exc_info:
            log_entry['exception'] = self.formatException(record.exc_info)

        # Add extra fields if present
        if hasattr(record, 'extra_data'):
            log_entry['extra_data'] = record.extra_data

        return json.dumps(log_entry)


def setup_logger(name: str, level: int = logging.INFO, log_file: str = None) -> logging.Logger:
    """
    Set up a logger with JSON formatting.

    Args:
        name: Name of the logger
        level: Logging level (default: INFO)
        log_file: Optional file path to log to (default: None for console only)

    Returns:
        Configured logger instance
    """
    logger = logging.getLogger(name)
    logger.setLevel(level)

    # Prevent adding multiple handlers if logger already exists
    if logger.handlers:
        return logger

    # Create JSON formatter
    json_formatter = JsonFormatter()

    # Create console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(json_formatter)
    logger.addHandler(console_handler)

    # Create file handler if log_file is specified
    if log_file:
        # Create logs directory if it doesn't exist
        os.makedirs(os.path.dirname(log_file), exist_ok=True)
        file_handler = logging.FileHandler(log_file)
        file_handler.setFormatter(json_formatter)
        logger.addHandler(file_handler)

    # Prevent propagation to root logger to avoid duplicate logs
    logger.propagate = False

    return logger


def log_validation_event(
    logger: logging.Logger,
    validation_type: str,
    status: str,
    details: Dict[str, Any] = None,
    level: int = logging.INFO
) -> None:
    """
    Log a validation event with structured data.

    Args:
        logger: Logger instance to use
        validation_type: Type of validation (e.g., 'embedding', 'storage', 'retrieval')
        status: Status of the validation ('success', 'failure', 'warning')
        details: Additional details about the validation
        level: Logging level to use
    """
    extra_data = {
        'validation_type': validation_type,
        'status': status,
        'details': details or {}
    }

    message = f"Validation {validation_type} {status}"
    logger.log(level, message, extra={'extra_data': extra_data})


# Pre-configured loggers for common use cases
validation_logger = setup_logger('rag_validation', log_file='validation_reports/validation.log')
error_logger = setup_logger('rag_error', level=logging.ERROR, log_file='validation_reports/error.log')
performance_logger = setup_logger('rag_performance', log_file='validation_reports/performance.log')