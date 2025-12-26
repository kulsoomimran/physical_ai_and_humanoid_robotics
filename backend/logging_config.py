import logging
import sys
from typing import Dict, Any
from logging import config


def setup_logging(log_level: str = "INFO") -> None:
    """
    Set up logging configuration for the application
    """
    logging_config: Dict[str, Any] = {
        "version": 1,
        "disable_existing_loggers": False,
        "formatters": {
            "default": {
                "format": "%(asctime)s - %(name)s - %(levelname)s - %(message)s",
            },
            "detailed": {
                "format": "%(asctime)s - %(name)s - %(levelname)s - %(module)s - %(funcName)s - %(message)s",
            },
        },
        "handlers": {
            "console": {
                "class": "logging.StreamHandler",
                "level": log_level,
                "formatter": "default",
                "stream": sys.stdout,
            },
        },
        "root": {
            "level": log_level,
            "handlers": ["console"],
        },
        "loggers": {
            "uvicorn": {
                "level": "INFO",
                "handlers": ["console"],
                "propagate": False,
            },
            "uvicorn.error": {
                "level": "INFO",
                "handlers": ["console"],
                "propagate": False,
            },
            "uvicorn.access": {
                "level": "INFO",
                "handlers": ["console"],
                "propagate": False,
            },
            "rag_chatbot": {
                "level": log_level,
                "handlers": ["console"],
                "propagate": False,
            },
        },
    }

    config.dictConfig(logging_config)


def get_logger(name: str) -> logging.Logger:
    """
    Get a logger with the specified name
    """
    return logging.getLogger(name)