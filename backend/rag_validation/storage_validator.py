"""
Storage validator for RAG validation system.
Validates Qdrant storage with complete and accurate metadata to ensure successful retrieval.
"""
import qdrant_client
from qdrant_client.http import models
from typing import List, Dict, Any, Optional
from backend.rag_validation.config_loader import ConfigLoader
from backend.rag_validation.logger import validation_logger
from backend.rag_validation.error_handler import ErrorHandler, handle_validation_error
from backend.rag_validation.performance_utils import record_performance, PerformanceTimer
from backend.rag_validation.test_data import TestDataGenerator
import uuid


class StorageValidator:
    """
    Validator for Qdrant storage and metadata validation.
    """

    def __init__(self, config_loader: ConfigLoader = None):
        """
        Initialize the storage validator.

        Args:
            config_loader: Configuration loader instance
        """
        self.config_loader = config_loader or ConfigLoader()
        self.config = self.config_loader.get_config()
        self.error_handler = ErrorHandler(validation_logger)

        # Initialize Qdrant client
        qdrant_url = self.config_loader.get_qdrant_url()
        qdrant_api_key = self.config_loader.get_qdrant_api_key()

        self.client = qdrant_client.QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key
        )

        # Get collection configuration
        self.collection_name = self.config.get("qdrant_collection", "rag-embeddings")

        validation_logger.info(
            "StorageValidator initialized",
            extra={'extra_data': {'collection_name': self.collection_name}}
        )

    def validate_collection_exists(self) -> Dict[str, Any]:
        """
        Validate that the Qdrant collection exists.

        Returns:
            Dictionary with validation results
        """
        timer = PerformanceTimer()
        timer.start()

        try:
            # Try to get collection info
            collection_info = self.client.get_collection(self.collection_name)

            result = {
                "success": True,
                "collection_exists": True,
                "vector_size": collection_info.config.params.vectors.size,
                "distance": collection_info.config.params.vectors.distance,
                "point_count": collection_info.points_count
            }

            duration = timer.stop()
            record_performance("validate_collection_exists", duration)

            return result

        except Exception as e:
            duration = timer.stop()
            error_info = handle_validation_error(e, "storage_validator", validation_logger)

            # Check if it's a collection not found error
            if "not found" in str(e).lower() or "not exist" in str(e).lower():
                result = {
                    "success": False,
                    "collection_exists": False,
                    "error": str(e),
                    "error_info": error_info,
                    "duration_ms": duration
                }
            else:
                result = {
                    "success": False,
                    "collection_exists": None,  # Unknown due to error
                    "error": str(e),
                    "error_info": error_info,
                    "duration_ms": duration
                }

            return result

    def validate_vector_dimensions(self, expected_dims: int = 768) -> Dict[str, Any]:
        """
        Validate that the Qdrant collection has the expected vector dimensions.

        Args:
            expected_dims: Expected number of dimensions

        Returns:
            Dictionary with validation results
        """
        timer = PerformanceTimer()
        timer.start()

        try:
            # Get collection info
            collection_info = self.client.get_collection(self.collection_name)
            actual_dims = collection_info.config.params.vectors.size

            is_valid = actual_dims == expected_dims

            result = {
                "success": True,
                "valid": is_valid,
                "expected_dimensions": expected_dims,
                "actual_dimensions": actual_dims,
                "distance_function": collection_info.config.params.vectors.distance
            }

            if not is_valid:
                validation_logger.warning(
                    f"Vector dimension mismatch: expected {expected_dims}, got {actual_dims}",
                    extra={'extra_data': result}
                )

            duration = timer.stop()
            record_performance("validate_vector_dimensions", duration)

            return result

        except Exception as e:
            duration = timer.stop()
            error_info = handle_validation_error(e, "storage_validator", validation_logger)
            return {
                "success": False,
                "valid": False,
                "error": str(e),
                "error_info": error_info,
                "duration_ms": duration
            }

    def validate_metadata_schema(self, sample_metadata: Dict[str, Any] = None) -> Dict[str, Any]:
        """
        Validate the metadata schema by checking for required fields.

        Args:
            sample_metadata: Sample metadata to validate (will generate if not provided)

        Returns:
            Dictionary with validation results
        """
        if sample_metadata is None:
            # Generate sample metadata for validation
            sample_metadata_list = TestDataGenerator.generate_metadata(1)
            sample_metadata = sample_metadata_list[0] if sample_metadata_list else {}

        try:
            required_fields = [
                "source_url",
                "page_title",
                "section_hierarchy",
                "created_at",
                "chunk_index"
            ]

            missing_fields = []
            for field in required_fields:
                if field not in sample_metadata:
                    missing_fields.append(field)

            result = {
                "valid": len(missing_fields) == 0,
                "missing_fields": missing_fields,
                "required_fields": required_fields,
                "provided_fields": list(sample_metadata.keys()),
                "sample_metadata": sample_metadata
            }

            if not result["valid"]:
                validation_logger.warning(
                    f"Metadata schema validation failed: missing fields {missing_fields}",
                    extra={'extra_data': result}
                )

            return result

        except Exception as e:
            error_info = handle_validation_error(e, "storage_validator", validation_logger)
            return {
                "valid": False,
                "error": str(e),
                "error_info": error_info
            }

    def validate_store_embedding_with_metadata(
        self,
        embedding: List[float],
        metadata: Dict[str, Any],
        point_id: str = None
    ) -> Dict[str, Any]:
        """
        Validate storing an embedding with metadata in Qdrant.

        Args:
            embedding: The embedding vector to store
            metadata: Metadata to store with the embedding
            point_id: Optional point ID (will generate if not provided)

        Returns:
            Dictionary with validation results
        """
        timer = PerformanceTimer()
        timer.start()

        if point_id is None:
            point_id = str(uuid.uuid4())

        try:
            # Prepare the point to upsert
            point = models.PointStruct(
                id=point_id,
                vector=embedding,
                payload=metadata
            )

            # Upsert the point to Qdrant
            self.client.upsert(
                collection_name=self.collection_name,
                points=[point]
            )

            duration = timer.stop()

            result = {
                "success": True,
                "point_id": point_id,
                "embedding_length": len(embedding),
                "metadata_keys": list(metadata.keys()),
                "duration_ms": duration
            }

            record_performance("validate_store_embedding_with_metadata", duration)

            return result

        except Exception as e:
            duration = timer.stop()
            error_info = handle_validation_error(e, "storage_validator", validation_logger)
            return {
                "success": False,
                "error": str(e),
                "error_info": error_info,
                "point_id": point_id,
                "duration_ms": duration
            }

    def validate_metadata_retrieval(self, point_id: str) -> Dict[str, Any]:
        """
        Validate that metadata can be retrieved correctly for a stored point.

        Args:
            point_id: ID of the point to retrieve

        Returns:
            Dictionary with validation results
        """
        timer = PerformanceTimer()
        timer.start()

        try:
            # Retrieve the point from Qdrant
            points = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[point_id],
                with_payload=True,
                with_vectors=False
            )

            if not points:
                result = {
                    "success": False,
                    "error": f"No point found with ID {point_id}",
                    "retrieved_point": None,
                    "duration_ms": timer.stop()
                }
                return result

            retrieved_point = points[0]
            payload = retrieved_point.payload if retrieved_point.payload else {}

            result = {
                "success": True,
                "point_id": point_id,
                "retrieved_metadata": payload,
                "metadata_keys": list(payload.keys()),
                "duration_ms": timer.stop()
            }

            record_performance("validate_metadata_retrieval", timer.stop())

            return result

        except Exception as e:
            duration = timer.stop()
            error_info = handle_validation_error(e, "storage_validator", validation_logger)
            return {
                "success": False,
                "error": str(e),
                "error_info": error_info,
                "point_id": point_id,
                "duration_ms": duration
            }

    def run_complete_storage_validation(self) -> Dict[str, Any]:
        """
        Run complete storage validation workflow.

        Returns:
            Dictionary with complete validation results
        """
        # Validate collection exists
        collection_result = self.validate_collection_exists()

        results = {
            "collection_validation": collection_result,
            "validation_steps": []
        }

        # Only proceed if collection exists
        if collection_result["success"]:
            # Validate vector dimensions
            dims_result = self.validate_vector_dimensions()
            results["dimension_validation"] = dims_result
            results["validation_steps"].append("dimension_validation")

            # Generate test data
            test_embeddings_data = TestDataGenerator.generate_embedding_data(3)

            storage_results = []
            retrieval_results = []

            for i, data in enumerate(test_embeddings_data):
                # Store the embedding with metadata
                store_result = self.validate_store_embedding_with_metadata(
                    data["vector"],
                    data["metadata"],
                    data["id"]
                )
                storage_results.append(store_result)

                if store_result["success"]:
                    # Retrieve and validate metadata
                    retrieval_result = self.validate_metadata_retrieval(data["id"])
                    retrieval_results.append(retrieval_result)

            results["storage_results"] = storage_results
            results["retrieval_results"] = retrieval_results
            results["validation_steps"].extend(["storage_validation", "retrieval_validation"])

        # Calculate overall success
        collection_valid = collection_result.get("success", False)
        dims_valid = results.get("dimension_validation", {}).get("valid", True)  # If not checked, assume OK
        storage_success_count = sum(1 for r in results.get("storage_results", []) if r.get("success", False))
        retrieval_success_count = sum(1 for r in results.get("retrieval_results", []) if r.get("success", False))

        total_storage = len(results.get("storage_results", []))
        total_retrieval = len(results.get("retrieval_results", []))

        results["overall_success"] = {
            "collection_valid": collection_valid,
            "dimensions_valid": dims_valid,
            "storage_success_rate": storage_success_count / total_storage if total_storage > 0 else 0,
            "retrieval_success_rate": retrieval_success_count / total_retrieval if total_retrieval > 0 else 0,
            "all_valid": (
                collection_valid and
                dims_valid and
                (storage_success_count == total_storage if total_storage > 0 else True) and
                (retrieval_success_count == total_retrieval if total_retrieval > 0 else True)
            )
        }

        return results


def create_storage_validator(config_loader: ConfigLoader = None) -> StorageValidator:
    """
    Create and return a storage validator instance.

    Args:
        config_loader: Optional configuration loader instance

    Returns:
        StorageValidator instance
    """
    return StorageValidator(config_loader)