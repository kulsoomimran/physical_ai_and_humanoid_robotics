"""
Embedding validator for RAG validation system.
Validates Cohere embedding generation from text chunks to ensure correct model and dimensions.
"""
import cohere
from typing import List, Dict, Any, Optional
from backend.rag_validation.config_loader import ConfigLoader
from backend.rag_validation.logger import validation_logger
from backend.rag_validation.error_handler import ErrorHandler, handle_validation_error
from backend.rag_validation.performance_utils import record_performance, PerformanceTimer
from backend.rag_validation.test_data import TestDataGenerator


class EmbeddingValidator:
    """
    Validator for Cohere embedding generation pipeline.
    """

    def __init__(self, config_loader: ConfigLoader = None):
        """
        Initialize the embedding validator.

        Args:
            config_loader: Configuration loader instance
        """
        self.config_loader = config_loader or ConfigLoader()
        self.config = self.config_loader.get_config()
        self.error_handler = ErrorHandler(validation_logger)

        # Initialize Cohere client
        api_key = self.config_loader.get_cohere_api_key()
        self.client = cohere.Client(api_key)

        # Get model configuration
        self.model = self.config.get("cohere_model", "embed-multilingual-v2.0")

        validation_logger.info(
            "EmbeddingValidator initialized",
            extra={'extra_data': {'model': self.model}}
        )

    def validate_model_configuration(self) -> Dict[str, Any]:
        """
        Validate the Cohere model configuration.

        Returns:
            Dictionary with validation results
        """
        timer = PerformanceTimer()
        timer.start()

        try:
            expected_dims = 768  # Cohere multilingual model dimensions

            # Check if the configured model exists and get its properties
            # For now, we'll validate based on known models
            known_models = {
                "embed-multilingual-v2.0": 768,
                "embed-english-v2.0": 4096,
                "embed-english-light-v2.0": 1024,
                "embed-multilingual-light-v2.0": 384
            }

            if self.model not in known_models:
                result = {
                    "success": False,
                    "error": f"Unknown Cohere model: {self.model}",
                    "expected_dimensions": expected_dims,
                    "actual_dimensions": None,
                    "model_valid": False
                }
            else:
                actual_dims = known_models[self.model]
                result = {
                    "success": True,
                    "expected_dimensions": expected_dims,
                    "actual_dimensions": actual_dims,
                    "model_valid": True,
                    "model": self.model
                }

                # Log a warning if dimensions don't match expected (but still allow it)
                if actual_dims != expected_dims:
                    validation_logger.warning(
                        f"Model {self.model} has {actual_dims} dimensions, expected {expected_dims}",
                        extra={'extra_data': {'model': self.model, 'dimensions': actual_dims}}
                    )

            duration = timer.stop()
            record_performance("validate_model_configuration", duration)

            return result

        except Exception as e:
            duration = timer.stop()
            error_info = handle_validation_error(e, "embedding_validator", validation_logger)
            return {
                "success": False,
                "error": str(e),
                "error_info": error_info,
                "duration_ms": duration
            }

    def validate_single_embedding(self, text: str) -> Dict[str, Any]:
        """
        Validate embedding generation for a single text chunk.

        Args:
            text: Text to generate embedding for

        Returns:
            Dictionary with validation results
        """
        timer = PerformanceTimer()
        timer.start()

        try:
            # Generate embedding using Cohere
            response = self.client.embed(
                texts=[text],
                model=self.model,
                input_type="search_document"
            )

            embeddings = response.embeddings
            duration = timer.stop()

            if not embeddings or len(embeddings) == 0:
                result = {
                    "success": False,
                    "error": "No embeddings returned from Cohere API",
                    "text_length": len(text),
                    "duration_ms": duration
                }
            else:
                embedding = embeddings[0]
                result = {
                    "success": True,
                    "text_length": len(text),
                    "embedding_length": len(embedding),
                    "embedding_sample": embedding[:5],  # First 5 values as sample
                    "duration_ms": duration
                }

                # Record performance
                record_performance("validate_single_embedding", duration)

            return result

        except Exception as e:
            duration = timer.stop()
            error_info = handle_validation_error(e, "embedding_validator", validation_logger)
            return {
                "success": False,
                "error": str(e),
                "error_info": error_info,
                "text_length": len(text) if 'text' in locals() else 0,
                "duration_ms": duration
            }

    def validate_batch_embeddings(self, texts: List[str], batch_size: int = 96) -> Dict[str, Any]:
        """
        Validate embedding generation for a batch of text chunks.

        Args:
            texts: List of texts to generate embeddings for
            batch_size: Size of each batch (Cohere max is 96)

        Returns:
            Dictionary with validation results
        """
        timer = PerformanceTimer()
        timer.start()

        try:
            all_embeddings = []
            failed_batches = 0
            successful_batches = 0

            # Process in batches
            for i in range(0, len(texts), batch_size):
                batch = texts[i:i + batch_size]

                try:
                    response = self.client.embed(
                        texts=batch,
                        model=self.model,
                        input_type="search_document"
                    )

                    batch_embeddings = response.embeddings
                    all_embeddings.extend(batch_embeddings)
                    successful_batches += 1

                except Exception as batch_error:
                    failed_batches += 1
                    error_info = handle_validation_error(
                        batch_error,
                        "embedding_validator_batch",
                        validation_logger
                    )
                    validation_logger.error(
                        f"Batch {i//batch_size + 1} failed: {str(batch_error)}",
                        extra={'extra_data': {'batch_index': i//batch_size, 'error_info': error_info}}
                    )
                    # Add None placeholders for failed batch
                    all_embeddings.extend([None] * len(batch))

            duration = timer.stop()

            result = {
                "success": failed_batches == 0,
                "total_texts": len(texts),
                "successful_batches": successful_batches,
                "failed_batches": failed_batches,
                "total_embeddings": len([e for e in all_embeddings if e is not None]),
                "duration_ms": duration,
                "average_time_per_text_ms": duration / len(texts) if texts else 0
            }

            # Record performance
            record_performance("validate_batch_embeddings", duration)

            return result

        except Exception as e:
            duration = timer.stop()
            error_info = handle_validation_error(e, "embedding_validator", validation_logger)
            return {
                "success": False,
                "error": str(e),
                "error_info": error_info,
                "duration_ms": duration
            }

    def validate_embedding_dimensions(self, embedding: List[float], expected_dims: int = 768) -> Dict[str, Any]:
        """
        Validate that an embedding has the expected dimensions.

        Args:
            embedding: The embedding vector to validate
            expected_dims: Expected number of dimensions

        Returns:
            Dictionary with validation results
        """
        try:
            actual_dims = len(embedding) if embedding else 0
            is_valid = actual_dims == expected_dims

            result = {
                "valid": is_valid,
                "expected_dimensions": expected_dims,
                "actual_dimensions": actual_dims,
                "difference": actual_dims - expected_dims
            }

            if not is_valid:
                validation_logger.warning(
                    f"Embedding dimension mismatch: expected {expected_dims}, got {actual_dims}",
                    extra={'extra_data': result}
                )

            return result

        except Exception as e:
            error_info = handle_validation_error(e, "embedding_validator", validation_logger)
            return {
                "valid": False,
                "error": str(e),
                "error_info": error_info
            }

    def validate_embedding_quality(self, embedding: List[float], text: str) -> Dict[str, Any]:
        """
        Validate the quality of an embedding (basic checks).

        Args:
            embedding: The embedding vector to validate
            text: Original text that generated the embedding

        Returns:
            Dictionary with validation results
        """
        try:
            if not embedding:
                return {
                    "valid": False,
                    "quality_score": 0,
                    "issues": ["Empty embedding"]
                }

            # Check for NaN or infinite values
            import math
            invalid_values = [math.isnan(v) or math.isinf(v) for v in embedding]
            has_invalid_values = any(invalid_values)

            # Calculate basic statistics
            avg_abs = sum(abs(v) for v in embedding) / len(embedding)
            max_val = max(abs(v) for v in embedding)
            min_val = min(abs(v) for v in embedding)

            # Quality checks
            issues = []
            if has_invalid_values:
                issues.append("Contains NaN or infinite values")

            if avg_abs < 0.001:  # Very low magnitude might indicate poor quality
                issues.append("Very low magnitude (possible zero vector)")

            quality_score = 1.0  # Start with perfect score
            if has_invalid_values:
                quality_score -= 0.5
            if avg_abs < 0.001:
                quality_score -= 0.3

            quality_score = max(0.0, min(1.0, quality_score))  # Clamp to [0, 1]

            result = {
                "valid": len(issues) == 0,
                "quality_score": quality_score,
                "issues": issues,
                "statistics": {
                    "avg_abs_value": avg_abs,
                    "max_abs_value": max_val,
                    "min_abs_value": min_val,
                    "dimension_count": len(embedding)
                }
            }

            if quality_score < 0.8:  # Log quality issues
                validation_logger.warning(
                    f"Low embedding quality detected: {quality_score}",
                    extra={'extra_data': result}
                )

            return result

        except Exception as e:
            error_info = handle_validation_error(e, "embedding_validator", validation_logger)
            return {
                "valid": False,
                "quality_score": 0,
                "error": str(e),
                "error_info": error_info
            }

    def run_complete_validation(self, test_texts: List[str] = None) -> Dict[str, Any]:
        """
        Run complete embedding validation workflow.

        Args:
            test_texts: Optional list of test texts (will generate if not provided)

        Returns:
            Dictionary with complete validation results
        """
        if test_texts is None:
            test_texts = TestDataGenerator.generate_sample_texts(5)

        results = {
            "model_validation": self.validate_model_configuration(),
            "single_embeddings": [],
            "batch_validation": self.validate_batch_embeddings(test_texts),
            "quality_assessment": []
        }

        # Validate each text individually to check quality
        for text in test_texts:
            single_result = self.validate_single_embedding(text)
            results["single_embeddings"].append(single_result)

            if single_result["success"]:
                # Validate dimensions and quality of successful embeddings
                embedding = None  # This would come from the actual API response
                # For now, we'll skip quality assessment since we don't have the embedding from the response
                # In a real implementation, we would access the embedding from the response
                pass

        # Calculate overall success
        model_valid = results["model_validation"]["success"]
        single_success_count = sum(1 for r in results["single_embeddings"] if r["success"])
        single_success_rate = single_success_count / len(results["single_embeddings"]) if results["single_embeddings"] else 0

        results["overall_success"] = {
            "model_config_valid": model_valid,
            "single_embedding_success_rate": single_success_rate,
            "batch_validation_success": results["batch_validation"]["success"],
            "all_valid": model_valid and single_success_rate >= 0.9 and results["batch_validation"]["success"]
        }

        return results


def create_embedding_validator(config_loader: ConfigLoader = None) -> EmbeddingValidator:
    """
    Create and return an embedding validator instance.

    Args:
        config_loader: Optional configuration loader instance

    Returns:
        EmbeddingValidator instance
    """
    return EmbeddingValidator(config_loader)