import json
import os
from typing import Dict, Any
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

class ConfigLoader:
    """
    Configuration loader for RAG validation system.
    Loads configuration from JSON file and environment variables with validation and defaults.
    """

    def __init__(self, config_path: str = "config/validation_config.json"):
        """
        Initialize the configuration loader.

        Args:
            config_path: Path to the configuration file
        """
        self.config_path = config_path
        self.config = self._load_config()
        self._validate_config()

    def _load_config(self) -> Dict[str, Any]:
        """
        Load configuration from JSON file and environment variables.

        Returns:
            Dictionary containing the loaded configuration
        """
        # Load default configuration from file
        try:
            with open(self.config_path, 'r') as f:
                config = json.load(f)
        except FileNotFoundError:
            print(f"Configuration file {self.config_path} not found. Using default values.")
            config = self._get_default_config()
        except json.JSONDecodeError:
            print(f"Invalid JSON in configuration file {self.config_path}. Using default values.")
            config = self._get_default_config()

        # Apply defaults for missing values
        config = self._apply_defaults(config)

        # Override with environment variables if available
        self._apply_environment_overrides(config)

        return config

    def _apply_defaults(self, config: Dict[str, Any]) -> Dict[str, Any]:
        """
        Apply default values for any missing configuration parameters.

        Args:
            config: Configuration dictionary to update with defaults

        Returns:
            Updated configuration dictionary
        """
        # Ensure the base structure exists
        if "validation_thresholds" not in config:
            config["validation_thresholds"] = {}

        if "test_data" not in config:
            config["test_data"] = {}

        if "validation_settings" not in config:
            config["validation_settings"] = {}

        if "logging" not in config:
            config["logging"] = {}

        if "performance_monitoring" not in config:
            config["performance_monitoring"] = {}

        if "error_handling" not in config:
            config["error_handling"] = {}

        if "output_options" not in config:
            config["output_options"] = {}

        # Apply specific defaults
        defaults = {
            "cohere_model": "embed-multilingual-v2.0",
            "qdrant_collection": "rag-embeddings",
            "validation_thresholds": {
                "embedding_success_rate": 0.99,
                "storage_success_rate": 0.99,
                "retrieval_relevance": 0.95,
                "max_latency_ms": 500,
                "min_throughput_per_sec": 10,
                "p95_latency_ms": 750,
                "p99_latency_ms": 1000
            },
            "test_data": {
                "sample_texts": [
                    "Artificial intelligence is transforming many industries with advanced machine learning algorithms.",
                    "Natural language processing enables computers to understand human language patterns.",
                    "Vector embeddings represent semantic meaning in high-dimensional mathematical spaces."
                ],
                "test_queries": [
                    "What is artificial intelligence?",
                    "How do vector embeddings work?",
                    "Explain semantic similarity in NLP."
                ],
                "expected_urls": [
                    "https://example.com/ai-introduction",
                    "https://example.com/nlp-overview",
                    "https://example.com/vector-embeddings"
                ]
            },
            "validation_settings": {
                "timeout_seconds": 30,
                "retry_attempts": 3,
                "batch_size": 96,
                "top_k_results": 5,
                "concurrent_threads": 5,
                "iterations_per_thread": 10,
                "time_limit_seconds": 30
            },
            "logging": {
                "level": "INFO",
                "format": "%(asctime)s - %(name)s - %(levelname)s - %(message)s",
            },
            "performance_monitoring": {
                "enable_profiling": True,
                "metrics_collection": True,
                "report_frequency_minutes": 5
            },
            "error_handling": {
                "graceful_degradation": True
            },
            "output_options": {
                "report_format": ["json", "csv", "summary"],
                "output_directory": "validation_reports",
                "include_visualizations": True,
                "preserve_raw_data": True
            }
        }

        # Merge defaults with existing config
        for key, value in defaults.items():
            if key not in config:
                config[key] = value
            elif isinstance(value, dict) and isinstance(config[key], dict):
                # Recursively merge nested dictionaries
                for sub_key, sub_value in value.items():
                    if sub_key not in config[key]:
                        config[key][sub_key] = sub_value
                    elif isinstance(sub_value, dict) and isinstance(config[key][sub_key], dict):
                        for sub_sub_key, sub_sub_value in sub_value.items():
                            if sub_sub_key not in config[key][sub_key]:
                                config[key][sub_key][sub_sub_key] = sub_sub_value

        # Apply scalar defaults
        config["validation_timeout"] = config.get("validation_timeout", 30)
        config["retry_attempts"] = config.get("retry_attempts", 3)

        return config

    def _validate_config(self) -> None:
        """
        Validate the loaded configuration against required parameters and constraints.
        """
        errors = []

        # Validate required environment variables
        required_env_vars = ["COHERE_API_KEY", "QDRANT_URL", "QDRANT_API_KEY"]
        for env_var in required_env_vars:
            if not os.getenv(env_var):
                errors.append(f"Missing required environment variable: {env_var}")

        # Validate configuration values
        config = self.config

        # Validate model name
        if not isinstance(config.get("cohere_model"), str):
            errors.append("cohere_model must be a string")

        # Validate collection name
        if not isinstance(config.get("qdrant_collection"), str):
            errors.append("qdrant_collection must be a string")

        # Validate thresholds
        thresholds = config.get("validation_thresholds", {})
        if not isinstance(thresholds, dict):
            errors.append("validation_thresholds must be a dictionary")
        else:
            # Check specific threshold values
            for threshold_name, threshold_value in thresholds.items():
                if threshold_name.endswith("_rate") or threshold_name.startswith("min_") or threshold_name.startswith("max_"):
                    if not isinstance(threshold_value, (int, float)):
                        errors.append(f"validation_thresholds.{threshold_name} must be a number")
                    elif threshold_name.endswith("_rate") and (threshold_value < 0 or threshold_value > 1):
                        errors.append(f"validation_thresholds.{threshold_name} must be between 0 and 1")

        # Validate timeout and retry values
        if not isinstance(config.get("validation_timeout", 30), int) or config.get("validation_timeout", 30) <= 0:
            errors.append("validation_timeout must be a positive integer")

        if not isinstance(config.get("retry_attempts", 3), int) or config.get("retry_attempts", 3) < 0:
            errors.append("retry_attempts must be a non-negative integer")

        # Validate test data
        test_data = config.get("test_data", {})
        if not isinstance(test_data, dict):
            errors.append("test_data must be a dictionary")
        else:
            if not isinstance(test_data.get("sample_texts"), list):
                errors.append("test_data.sample_texts must be a list of strings")
            if not isinstance(test_data.get("test_queries"), list):
                errors.append("test_data.test_queries must be a list of strings")

        # Validate validation settings
        settings = config.get("validation_settings", {})
        if not isinstance(settings, dict):
            errors.append("validation_settings must be a dictionary")
        else:
            if not isinstance(settings.get("timeout_seconds", 30), int) or settings.get("timeout_seconds", 30) <= 0:
                errors.append("validation_settings.timeout_seconds must be a positive integer")
            if not isinstance(settings.get("retry_attempts", 3), int) or settings.get("retry_attempts", 3) < 0:
                errors.append("validation_settings.retry_attempts must be a non-negative integer")
            if not isinstance(settings.get("batch_size", 96), int) or settings.get("batch_size", 96) <= 0:
                errors.append("validation_settings.batch_size must be a positive integer")
            if not isinstance(settings.get("top_k_results", 5), int) or settings.get("top_k_results", 5) <= 0:
                errors.append("validation_settings.top_k_results must be a positive integer")
            if not isinstance(settings.get("concurrent_threads", 5), int) or settings.get("concurrent_threads", 5) <= 0:
                errors.append("validation_settings.concurrent_threads must be a positive integer")
            if not isinstance(settings.get("iterations_per_thread", 10), int) or settings.get("iterations_per_thread", 10) <= 0:
                errors.append("validation_settings.iterations_per_thread must be a positive integer")

        if errors:
            error_msg = "Configuration validation failed:\n" + "\n".join(f"- {error}" for error in errors)
            raise ValueError(error_msg)

    def _apply_environment_overrides(self, config: Dict[str, Any]) -> None:
        """
        Apply environment variable overrides to the configuration.

        Args:
            config: Configuration dictionary to update
        """
        # Override Cohere model if specified in environment
        cohere_model = os.getenv("COHERE_MODEL")
        if cohere_model:
            config["cohere_model"] = cohere_model

        # Override Qdrant collection if specified in environment
        qdrant_collection = os.getenv("QDRANT_COLLECTION")
        if qdrant_collection:
            config["qdrant_collection"] = qdrant_collection

        # Override timeout if specified in environment
        validation_timeout = os.getenv("VALIDATION_TIMEOUT")
        if validation_timeout and validation_timeout.isdigit():
            config["validation_timeout"] = int(validation_timeout)

        # Override retry attempts if specified in environment
        retry_attempts = os.getenv("RETRY_ATTEMPTS")
        if retry_attempts and retry_attempts.isdigit():
            config["retry_attempts"] = int(retry_attempts)

        # Override other settings from environment variables
        cohere_api_key = os.getenv("COHERE_API_KEY")
        if cohere_api_key:
            # Don't store API keys in config for security reasons, but validate they exist
            pass

        qdrant_url = os.getenv("QDRANT_URL")
        if qdrant_url:
            # Don't store URLs in config for security reasons, but validate they exist
            pass

        qdrant_api_key = os.getenv("QDRANT_API_KEY")
        if qdrant_api_key:
            # Don't store API keys in config for security reasons, but validate they exist
            pass

    def get_config(self) -> Dict[str, Any]:
        """
        Get the loaded configuration.

        Returns:
            Dictionary containing the configuration
        """
        return self.config

    def get_cohere_api_key(self) -> str:
        """
        Get Cohere API key from environment variables.

        Returns:
            Cohere API key
        """
        api_key = os.getenv("COHERE_API_KEY")
        if not api_key:
            raise ValueError("COHERE_API_KEY environment variable is required")
        return api_key

    def get_qdrant_url(self) -> str:
        """
        Get Qdrant URL from environment variables.

        Returns:
            Qdrant URL
        """
        url = os.getenv("QDRANT_URL")
        if not url:
            raise ValueError("QDRANT_URL environment variable is required")
        return url

    def get_qdrant_api_key(self) -> str:
        """
        Get Qdrant API key from environment variables.

        Returns:
            Qdrant API key
        """
        api_key = os.getenv("QDRANT_API_KEY")
        if not api_key:
            raise ValueError("QDRANT_API_KEY environment variable is required")
        return api_key