import logging
from datetime import datetime
import json
from typing import Dict, Any

class CustomFormatter(logging.Formatter):
    """
    Custom formatter to output logs in a structured JSON format
    """
    def format(self, record):
        log_entry = {
            "timestamp": datetime.utcnow().isoformat(),
            "level": record.levelname,
            "logger": record.name,
            "message": record.getMessage(),
            "module": record.module,
            "function": record.funcName,
            "line": record.lineno
        }

        # Add exception info if present
        if record.exc_info:
            log_entry["exception"] = self.formatException(record.exc_info)

        # Add any extra fields
        for key, value in record.__dict__.items():
            if key not in [
                'name', 'msg', 'args', 'levelname', 'levelno', 'pathname',
                'filename', 'module', 'lineno', 'funcName', 'created',
                'msecs', 'relativeCreated', 'thread', 'threadName',
                'processName', 'process', 'getMessage', 'exc_info',
                'exc_text', 'stack_info'
            ]:
                log_entry[key] = value

        return json.dumps(log_entry)

def setup_logging(log_level: str = "INFO"):
    """
    Set up logging configuration for the application
    """
    # Create a custom logger
    logger = logging.getLogger()
    logger.setLevel(getattr(logging, log_level.upper()))

    # Create handler
    handler = logging.StreamHandler()

    # Create formatter and add it to handler
    formatter = CustomFormatter()
    handler.setFormatter(formatter)

    # Add handler to the logger
    logger.addHandler(handler)

    return logger

# Initialize logging
app_logger = setup_logging()