import random
import string
from typing import List, Dict, Any
import uuid


class TestDataGenerator:
    """
    Generator for test data used in RAG validation.
    """

    @staticmethod
    def generate_sample_texts(count: int = 5, min_length: int = 50, max_length: int = 500) -> List[str]:
        """
        Generate sample text chunks for embedding validation.

        Args:
            count: Number of text chunks to generate
            min_length: Minimum length of each text chunk
            max_length: Maximum length of each text chunk

        Returns:
            List of sample text chunks
        """
        sample_texts = [
            "The field of artificial intelligence has seen remarkable progress in recent years.",
            "Machine learning algorithms are increasingly being applied to solve complex problems.",
            "Natural language processing enables computers to understand human language.",
            "Deep learning models require significant computational resources for training.",
            "Vector embeddings represent semantic meaning in high-dimensional space."
        ]

        # If we need more texts than the sample, generate additional random content
        if count > len(sample_texts):
            additional_texts = TestDataGenerator._generate_random_texts(count - len(sample_texts), min_length, max_length)
            sample_texts.extend(additional_texts)

        return sample_texts[:count]

    @staticmethod
    def _generate_random_texts(count: int, min_length: int, max_length: int) -> List[str]:
        """
        Generate random text content.

        Args:
            count: Number of text chunks to generate
            min_length: Minimum length of each text chunk
            max_length: Maximum length of each text chunk

        Returns:
            List of randomly generated text chunks
        """
        texts = []
        for _ in range(count):
            length = random.randint(min_length, max_length)
            text = ''.join(random.choices(string.ascii_letters + string.digits + ' .,!?-', k=length))
            # Ensure it's more readable by adding some real words
            text = text.replace(' ', ' ').replace('.', '. ').replace('!', '! ').replace('?', '? ')
            texts.append(text)
        return texts

    @staticmethod
    def generate_test_queries(count: int = 3) -> List[str]:
        """
        Generate test queries for retrieval validation.

        Args:
            count: Number of test queries to generate

        Returns:
            List of test queries
        """
        queries = [
            "What is artificial intelligence?",
            "How do vector embeddings work?",
            "Explain machine learning concepts"
        ]

        # If we need more queries, create variations
        if count > len(queries):
            additional_queries = TestDataGenerator._generate_query_variations(count - len(queries))
            queries.extend(additional_queries)

        return queries[:count]

    @staticmethod
    def _generate_query_variations(count: int) -> List[str]:
        """
        Generate variations of test queries.

        Args:
            count: Number of query variations to generate

        Returns:
            List of query variations
        """
        base_queries = [
            "What are neural networks?",
            "Explain deep learning",
            "How does NLP work?",
            "What is computer vision?",
            "Describe reinforcement learning"
        ]
        return base_queries[:count]

    @staticmethod
    def generate_metadata(count: int = 5) -> List[Dict[str, Any]]:
        """
        Generate sample metadata for validation.

        Args:
            count: Number of metadata entries to generate

        Returns:
            List of metadata dictionaries
        """
        metadata_list = []
        for i in range(count):
            metadata = {
                "source_url": f"https://example.com/document_{i}.html",
                "page_title": f"Document Title {i}",
                "section_hierarchy": f"section_{i % 3}/subsection_{i % 2}",
                "created_at": f"2025-12-{19:02d}T10:30:0{i:02}Z",
                "chunk_index": i,
                "start_pos": i * 100,
                "end_pos": i * 100 + 200
            }
            metadata_list.append(metadata)
        return metadata_list

    @staticmethod
    def generate_embedding_vector(size: int = 768) -> List[float]:
        """
        Generate a sample embedding vector.

        Args:
            size: Size of the embedding vector (default 768 for Cohere multilingual model)

        Returns:
            List of floats representing an embedding vector
        """
        return [random.uniform(-1, 1) for _ in range(size)]

    @staticmethod
    def generate_embedding_data(count: int = 5, vector_size: int = 768) -> List[Dict[str, Any]]:
        """
        Generate sample embedding data with content, vector, and metadata.

        Args:
            count: Number of embedding data entries to generate
            vector_size: Size of each embedding vector

        Returns:
            List of embedding data dictionaries
        """
        texts = TestDataGenerator.generate_sample_texts(count)
        metadata_list = TestDataGenerator.generate_metadata(count)
        embedding_data = []

        for i in range(count):
            embedding_data.append({
                "id": str(uuid.uuid4()),
                "content": texts[i],
                "vector": TestDataGenerator.generate_embedding_vector(vector_size),
                "metadata": metadata_list[i]
            })

        return embedding_data

    @staticmethod
    def generate_validation_config() -> Dict[str, Any]:
        """
        Generate a sample validation configuration.

        Returns:
            Dictionary with validation configuration
        """
        return {
            "cohere_model": "embed-multilingual-v2.0",
            "qdrant_collection": "rag-embeddings",
            "validation_thresholds": {
                "embedding_success_rate": 0.99,
                "storage_success_rate": 0.99,
                "retrieval_relevance": 0.95,
                "max_latency_ms": 500
            },
            "test_data": {
                "sample_texts": TestDataGenerator.generate_sample_texts(3),
                "test_queries": TestDataGenerator.generate_test_queries(2)
            },
            "validation_timeout": 30,
            "retry_attempts": 3
        }

    @staticmethod
    def generate_error_scenarios() -> List[Dict[str, str]]:
        """
        Generate different error scenarios for testing error handling.

        Returns:
            List of error scenario descriptions
        """
        return [
            {
                "type": "cohere_api_error",
                "description": "Simulate Cohere API unavailability or rate limiting"
            },
            {
                "type": "qdrant_connection_error",
                "description": "Simulate Qdrant connection failure"
            },
            {
                "type": "invalid_input_error",
                "description": "Simulate malformed input text"
            },
            {
                "type": "storage_error",
                "description": "Simulate storage failure in Qdrant"
            },
            {
                "type": "retrieval_error",
                "description": "Simulate retrieval failure or timeout"
            }
        ]

    @staticmethod
    def generate_embedding_validation_test_data() -> List[Dict[str, Any]]:
        """
        Generate specific test data for embedding validation.

        Returns:
            List of dictionaries containing test data for embedding validation
        """
        return [
            {
                "id": "valid_text_1",
                "text": "Artificial intelligence is a wonderful field that is developing rapidly.",
                "expected_to_embed": True,
                "category": "valid_content"
            },
            {
                "id": "valid_text_2",
                "text": "Machine learning algorithms can process large amounts of data efficiently.",
                "expected_to_embed": True,
                "category": "valid_content"
            },
            {
                "id": "empty_text",
                "text": "",
                "expected_to_embed": False,
                "category": "edge_case"
            },
            {
                "id": "very_short_text",
                "text": "Hi.",
                "expected_to_embed": True,
                "category": "edge_case"
            },
            {
                "id": "very_long_text",
                "text": " ".join(["This is a very long text. " for _ in range(100)]),
                "expected_to_embed": True,
                "category": "edge_case"
            },
            {
                "id": "special_chars_text",
                "text": "Text with special characters: @#$%^&*()_+=[]{}|;:,.<>?",
                "expected_to_embed": True,
                "category": "edge_case"
            },
            {
                "id": "unicode_text",
                "text": "Text with unicode: café, naïve, résumé, and 日本語",
                "expected_to_embed": True,
                "category": "edge_case"
            }
        ]

    @staticmethod
    def sanitize_input(text: str) -> str:
        """
        Sanitize input text to prevent potential issues.

        Args:
            text: Input text to sanitize

        Returns:
            Sanitized text
        """
        if text is None:
            return ""

        # Remove null bytes and other problematic characters
        sanitized = text.replace('\x00', '').strip()

        # Limit length to prevent extremely large inputs
        max_length = 10000  # 10k characters max
        if len(sanitized) > max_length:
            sanitized = sanitized[:max_length]

        return sanitized

    @staticmethod
    def validate_input_text(text: str, min_length: int = 1, max_length: int = 10000) -> Dict[str, Any]:
        """
        Validate input text for common issues.

        Args:
            text: Text to validate
            min_length: Minimum allowed length
            max_length: Maximum allowed length

        Returns:
            Dictionary with validation results
        """
        result = {
            "valid": True,
            "errors": [],
            "sanitized_text": TestDataGenerator.sanitize_input(text)
        }

        if text is None:
            result["valid"] = False
            result["errors"].append("Text is None")
            return result

        if not isinstance(text, str):
            result["valid"] = False
            result["errors"].append("Text is not a string")
            return result

        if len(text) < min_length:
            result["valid"] = False
            result["errors"].append(f"Text length {len(text)} is less than minimum {min_length}")

        if len(text) > max_length:
            result["valid"] = False
            result["errors"].append(f"Text length {len(text)} exceeds maximum {max_length}")

        # Check for potential injection issues
        dangerous_patterns = [
            "<script", "javascript:", "vbscript:", "onerror=", "onload=",
            "eval(", "exec(", "__import__", "os.system"
        ]

        lower_text = text.lower()
        for pattern in dangerous_patterns:
            if pattern in lower_text:
                result["valid"] = False
                result["errors"].append(f"Dangerous pattern detected: {pattern}")

        return result