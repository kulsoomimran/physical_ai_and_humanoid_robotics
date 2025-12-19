"""
Retrieval validator for RAG validation system.
Validates similarity-based retrieval from Qdrant to ensure relevant chunks map back to source URLs and content.
"""
import qdrant_client
from qdrant_client.http import models
from typing import List, Dict, Any, Optional, Tuple
from backend.rag_validation.config_loader import ConfigLoader
from backend.rag_validation.logger import validation_logger
from backend.rag_validation.error_handler import ErrorHandler, handle_validation_error
from backend.rag_validation.performance_utils import record_performance, PerformanceTimer
from backend.rag_validation.test_data import TestDataGenerator
import numpy as np
from scipy.spatial.distance import cosine


class RetrievalValidator:
    """
    Validator for similarity-based retrieval from Qdrant.
    """

    def __init__(self, config_loader: ConfigLoader = None):
        """
        Initialize the retrieval validator.

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
            "RetrievalValidator initialized",
            extra={'extra_data': {'collection_name': self.collection_name}}
        )

    def validate_similarity_search(self, query_vector: List[float], top_k: int = 5) -> Dict[str, Any]:
        """
        Validate similarity search functionality in Qdrant.

        Args:
            query_vector: Vector to search for similar items
            top_k: Number of top results to retrieve

        Returns:
            Dictionary with validation results
        """
        timer = PerformanceTimer()
        timer.start()

        try:
            # Perform the search in Qdrant
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=top_k,
                with_payload=True,
                with_vectors=False
            )

            duration = timer.stop()

            if not search_results:
                result = {
                    "success": False,
                    "error": "No results returned from similarity search",
                    "query_vector_length": len(query_vector),
                    "top_k": top_k,
                    "results_count": 0,
                    "duration_ms": duration
                }
            else:
                # Extract relevant information from results
                result_data = []
                for hit in search_results:
                    result_data.append({
                        "id": hit.id,
                        "score": hit.score,
                        "payload": hit.payload
                    })

                result = {
                    "success": True,
                    "query_vector_length": len(query_vector),
                    "top_k": top_k,
                    "results_count": len(search_results),
                    "results": result_data,
                    "duration_ms": duration
                }

            record_performance("validate_similarity_search", duration)

            return result

        except Exception as e:
            duration = timer.stop()
            error_info = handle_validation_error(e, "retrieval_validator", validation_logger)
            return {
                "success": False,
                "error": str(e),
                "error_info": error_info,
                "query_vector_length": len(query_vector) if 'query_vector' in locals() else 0,
                "duration_ms": duration
            }

    def calculate_relevance_score(self, query: str, retrieved_content: str) -> float:
        """
        Calculate relevance score between query and retrieved content.

        Args:
            query: Original query text
            retrieved_content: Content retrieved from vector database

        Returns:
            Relevance score between 0 and 1 (1 being most relevant)
        """
        try:
            # Simple keyword overlap-based relevance scoring
            # In a real implementation, this would use more sophisticated methods
            query_words = set(query.lower().split())
            content_words = set(retrieved_content.lower().split())

            if not query_words:
                return 0.0

            # Calculate Jaccard similarity
            intersection = len(query_words.intersection(content_words))
            union = len(query_words.union(content_words))

            if union == 0:
                return 0.0

            jaccard_score = intersection / union

            # Also consider the ratio of matching words to query words
            word_match_ratio = intersection / len(query_words) if query_words else 0

            # Combine both metrics
            relevance_score = (jaccard_score + word_match_ratio) / 2

            return relevance_score

        except Exception as e:
            error_info = handle_validation_error(e, "retrieval_validator", validation_logger)
            validation_logger.error(f"Error calculating relevance score: {str(e)}", extra={'extra_data': error_info})
            return 0.0

    def validate_source_url_mapping(self, query: str, expected_url: str, top_k: int = 5) -> Dict[str, Any]:
        """
        Validate that the retrieved results map back to the expected source URL.

        Args:
            query: Query text to search for
            expected_url: Expected source URL that should be retrieved
            top_k: Number of results to check

        Returns:
            Dictionary with validation results
        """
        timer = PerformanceTimer()
        timer.start()

        try:
            # First, we need to generate an embedding for the query
            # For now, we'll simulate this or use test data
            # In a real implementation, we would call Cohere to embed the query
            from backend.rag_validation.embedding_validator import EmbeddingValidator
            config_loader = self.config_loader
            embedding_validator = EmbeddingValidator(config_loader)

            # For this validation, we'll use test data or a mock embedding
            # Since we don't want to make actual Cohere calls during validation
            query_embedding = [0.1] * 768  # Mock embedding

            # Perform similarity search
            search_result = self.validate_similarity_search(query_embedding, top_k)

            if not search_result["success"]:
                return {
                    "success": False,
                    "error": search_result.get("error", "Similarity search failed"),
                    "expected_url": expected_url,
                    "top_k": top_k
                }

            # Check if the expected URL is in the top results
            found_in_results = False
            found_at_position = -1
            relevant_results = []

            for i, result in enumerate(search_result["results"]):
                payload = result.get("payload", {})
                retrieved_url = payload.get("source_url", "")

                # Calculate relevance score
                retrieved_content = payload.get("content", "")
                relevance_score = self.calculate_relevance_score(query, retrieved_content)

                relevant_results.append({
                    "position": i,
                    "id": result["id"],
                    "retrieved_url": retrieved_url,
                    "relevance_score": relevance_score,
                    "is_expected_url": retrieved_url == expected_url
                })

                if retrieved_url == expected_url:
                    found_in_results = True
                    found_at_position = i
                    break

            result = {
                "success": True,
                "expected_url": expected_url,
                "top_k": top_k,
                "found_in_results": found_in_results,
                "found_at_position": found_at_position if found_in_results else None,
                "relevant_results": relevant_results,
                "total_results_checked": len(search_result["results"])
            }

            duration = timer.stop()
            record_performance("validate_source_url_mapping", duration)

            return result

        except Exception as e:
            duration = timer.stop()
            error_info = handle_validation_error(e, "retrieval_validator", validation_logger)
            return {
                "success": False,
                "error": str(e),
                "error_info": error_info,
                "expected_url": expected_url,
                "duration_ms": duration
            }

    def validate_content_mapping_accuracy(self, query: str, expected_content: str, top_k: int = 5) -> Dict[str, Any]:
        """
        Validate the accuracy of content mapping in retrieval results.

        Args:
            query: Query text that should match relevant content
            expected_content: Expected content that should be retrieved
            top_k: Number of results to check

        Returns:
            Dictionary with validation results
        """
        timer = PerformanceTimer()
        timer.start()

        try:
            # Mock embedding for query (in real implementation, would call Cohere)
            query_embedding = [0.1] * 768

            # Perform similarity search
            search_result = self.validate_similarity_search(query_embedding, top_k)

            if not search_result["success"]:
                return {
                    "success": False,
                    "error": search_result.get("error", "Similarity search failed"),
                    "top_k": top_k
                }

            # Calculate relevance scores for all results
            content_accuracy_results = []
            max_relevance_score = 0
            best_match_position = -1

            for i, result in enumerate(search_result["results"]):
                payload = result.get("payload", {})
                retrieved_content = payload.get("content", "")

                # Calculate relevance score between query and retrieved content
                relevance_score = self.calculate_relevance_score(query, retrieved_content)

                # Calculate similarity to expected content
                content_similarity = self.calculate_relevance_score(expected_content, retrieved_content)

                content_accuracy_results.append({
                    "position": i,
                    "id": result["id"],
                    "relevance_to_query": relevance_score,
                    "similarity_to_expected": content_similarity,
                    "retrieved_content_preview": retrieved_content[:100]  # First 100 chars
                })

                if relevance_score > max_relevance_score:
                    max_relevance_score = relevance_score
                    best_match_position = i

            result = {
                "success": True,
                "top_k": top_k,
                "content_accuracy_results": content_accuracy_results,
                "max_relevance_score": max_relevance_score,
                "best_match_position": best_match_position,
                "total_results": len(search_result["results"])
            }

            duration = timer.stop()
            record_performance("validate_content_mapping_accuracy", duration)

            return result

        except Exception as e:
            duration = timer.stop()
            error_info = handle_validation_error(e, "retrieval_validator", validation_logger)
            return {
                "success": False,
                "error": str(e),
                "error_info": error_info,
                "duration_ms": duration
            }

    def run_complete_retrieval_validation(self, test_queries: List[Dict[str, str]] = None) -> Dict[str, Any]:
        """
        Run complete retrieval validation workflow.

        Args:
            test_queries: List of test queries with expected results (will generate if not provided)

        Returns:
            Dictionary with complete validation results
        """
        if test_queries is None:
            # Generate test queries using test data generator
            sample_texts = TestDataGenerator.generate_sample_texts(3)
            test_queries = []
            for i, text in enumerate(sample_texts):
                test_queries.append({
                    "query": f"Information about {text.split()[0] if text.split() else 'topic'}",
                    "expected_content": text,
                    "expected_url": f"https://example.com/doc{i}.html"
                })

        results = {
            "validation_queries": len(test_queries),
            "similarity_search_results": [],
            "source_mapping_results": [],
            "content_accuracy_results": [],
            "relevance_scores": []
        }

        for query_data in test_queries:
            query = query_data["query"]
            expected_content = query_data.get("expected_content", "")
            expected_url = query_data.get("expected_url", "")

            # Perform similarity search
            # Using mock embedding since we don't want to call Cohere during validation
            mock_embedding = TestDataGenerator.generate_embedding_vector(768)
            search_result = self.validate_similarity_search(mock_embedding, top_k=3)
            results["similarity_search_results"].append(search_result)

            # Validate source URL mapping if expected URL is provided
            if expected_url:
                mapping_result = self.validate_source_url_mapping(query, expected_url, top_k=3)
                results["source_mapping_results"].append(mapping_result)

            # Validate content mapping accuracy if expected content is provided
            if expected_content:
                content_result = self.validate_content_mapping_accuracy(query, expected_content, top_k=3)
                results["content_accuracy_results"].append(content_result)

        # Calculate overall success metrics
        successful_searches = sum(1 for r in results["similarity_search_results"] if r["success"])
        successful_mappings = sum(1 for r in results["source_mapping_results"] if r.get("success", False))

        results["overall_success"] = {
            "similarity_search_success_rate": successful_searches / len(results["similarity_search_results"]) if results["similarity_search_results"] else 0,
            "source_mapping_success_rate": successful_mappings / len(results["source_mapping_results"]) if results["source_mapping_results"] else 0,
            "all_queries_processed": len(results["similarity_search_results"]) == len(test_queries)
        }

        return results


def create_retrieval_validator(config_loader: ConfigLoader = None) -> RetrievalValidator:
    """
    Create and return a retrieval validator instance.

    Args:
        config_loader: Optional configuration loader instance

    Returns:
        RetrievalValidator instance
    """
    return RetrievalValidator(config_loader)