#!/usr/bin/env python3
"""
RAG Retrieval Validation Script

This script connects to Qdrant to validate the RAG retrieval pipeline by:
- Loading existing vector collections
- Accepting test queries
- Performing top-k similarity search
- Validating results using returned text, metadata, and source URLs
"""

import os
import sys
import argparse
import logging
from typing import List, Dict, Any, Optional
import time
import json

# Load environment variables from .env file
from dotenv import load_dotenv
load_dotenv()

import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct, Distance, VectorParams

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


# Configuration for Qdrant and Cohere
class Config:
    """Configuration class for Qdrant and Cohere settings"""
    QDRANT_HOST = os.getenv("QDRANT_HOST", "localhost")
    QDRANT_PORT = int(os.getenv("QDRANT_PORT", 6333))
    QDRANT_GRPC_PORT = int(os.getenv("QDRANT_GRPC_PORT", 6334))
    QDRANT_URL = os.getenv("QDRANT_URL", "https://1da2ca80-1e47-4bae-b6f7-0c6ead2d353d.europe-west3-0.gcp.cloud.qdrant.io:6333")  # Cloud cluster URL
    QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")  # Required for cloud instances
    COHERE_API_KEY = os.getenv("COHERE_API_KEY")
    COHERE_MODEL = os.getenv("COHERE_MODEL", "embed-multilingual-v2.0")
    COLLECTION_NAME = os.getenv("QDRANT_COLLECTION", "book_content")
    DEFAULT_TOP_K = int(os.getenv("DEFAULT_TOP_K", 5))


# Data models based on data-model.md
class QueryVector:
    """The embedded representation of user queries used for similarity matching"""
    def __init__(self, query_id: str, vector: List[float], text: str):
        self.id = query_id
        self.vector = vector
        self.text = text
        self.timestamp = time.time()

        # Validation
        if not text.strip():
            raise ValueError("Text must not be empty")
        if not vector:
            raise ValueError("Vector must not be empty")


class RetrievedChunk:
    """Text chunks returned from the vector database that match the query semantics"""
    def __init__(self, chunk_id: str, text: str, score: float, vector_id: str, metadata: Dict[str, Any]):
        self.id = chunk_id
        self.text = text
        self.score = score
        self.vector_id = vector_id
        self.metadata = metadata

        # Validation
        if not text.strip():
            raise ValueError("Text must not be empty")
        if not (0 <= score <= 1):
            raise ValueError("Score must be between 0 and 1")
        if not metadata:
            raise ValueError("Metadata must contain required source information")


class SourceMetadata:
    """Information linking retrieved content back to original URLs and document identifiers"""
    def __init__(self, source_url: str, document_id: str, chunk_index: int,
                 page_number: Optional[int] = None, section_title: Optional[str] = None):
        self.source_url = source_url
        self.document_id = document_id
        self.chunk_index = chunk_index
        self.page_number = page_number
        self.section_title = section_title

        # Validation
        if not source_url or not source_url.startswith(('http://', 'https://')):
            raise ValueError("Source URL must be a valid URL")
        if not document_id.strip():
            raise ValueError("Document ID must exist and be non-empty")


class ValidationResult:
    """Output representing the result of a validation check"""
    def __init__(self, query_id: str, retrieved_chunks: List[RetrievedChunk],
                 is_valid: bool, error_message: Optional[str] = None):
        self.query_id = query_id
        self.retrieved_chunks = retrieved_chunks
        self.is_valid = is_valid
        self.error_message = error_message
        self.validation_timestamp = time.time()

        # Validation
        if not retrieved_chunks and is_valid:
            raise ValueError("Retrieved chunks list must not be empty when validation passes")
        if not self.validation_timestamp:
            raise ValueError("Validation timestamp must be current")


class ValidationReport:
    """Output document summarizing the results of the retrieval validation tests"""
    def __init__(self, report_id: str, total_queries: int, successful_queries: int,
                 failed_queries: int, details: List[ValidationResult]):
        self.report_id = report_id
        self.total_queries = total_queries
        self.successful_queries = successful_queries
        self.failed_queries = failed_queries
        self.details = details
        self.success_rate = (successful_queries / total_queries * 100) if total_queries > 0 else 0
        self.execution_time = None

        # Validation
        if self.success_rate < 0 or self.success_rate > 100:
            raise ValueError("Success rate must be between 0 and 100")
        if total_queries != (successful_queries + failed_queries):
            raise ValueError("Total queries must equal successful + failed queries")
        if len(details) != total_queries:
            raise ValueError("Details list must match the number of queries tested")


def get_qdrant_client():
    """Create and return Qdrant client instance - supports both local and cloud instances"""
    try:
        # If QDRANT_URL is set (for cloud instances), use URL with API key
        if Config.QDRANT_URL and Config.QDRANT_URL.startswith('http'):
            if not Config.QDRANT_API_KEY:
                logger.warning("QDRANT_API_KEY is required for cloud instances")
            client = QdrantClient(
                url=Config.QDRANT_URL,
                api_key=Config.QDRANT_API_KEY,
                prefer_grpc=False  # Using REST API for cloud instances
            )
        else:
            # Use host/port for local instances
            client = QdrantClient(
                host=Config.QDRANT_HOST,
                port=Config.QDRANT_PORT
            )
        return client
    except Exception as e:
        logger.error(f"Failed to create Qdrant client: {str(e)}")
        raise


def load_collection(client: QdrantClient, collection_name: str):
    """Load and verify the specified collection exists"""
    try:
        collection_info = client.get_collection(collection_name)
        logger.info(f"Successfully loaded collection '{collection_name}' with {collection_info.points_count} points")
        return True
    except Exception as e:
        logger.error(f"Failed to load collection '{collection_name}': {str(e)}")
        return False


def generate_query_embedding(query_text: str) -> List[float]:
    """Generate embedding for a query using Cohere"""
    try:
        if not Config.COHERE_API_KEY:
            raise ValueError("COHERE_API_KEY environment variable not set")

        co = cohere.Client(Config.COHERE_API_KEY)
        response = co.embed(
            texts=[query_text],
            model=Config.COHERE_MODEL,
            input_type="search_query"  # Use search_query for queries
        )
        embedding = response.embeddings[0]
        return embedding
    except Exception as e:
        logger.error(f"Failed to generate query embedding: {str(e)}")
        raise


def perform_similarity_search(client: QdrantClient, query_embedding: List[float],
                            collection_name: str, top_k: int):
    """Perform top-k similarity search in Qdrant"""
    try:
        search_results = client.query_points(
            collection_name=collection_name,
            query=query_embedding,
            limit=top_k,
            with_payload=True,
            with_vectors=False
        )
        return search_results.points  # Return the points from the result
    except Exception as e:
        logger.error(f"Similarity search failed: {str(e)}")
        raise


def extract_and_validate_metadata(qdrant_results) -> List[Dict]:
    """Extract and validate metadata from Qdrant results"""
    validated_results = []

    for result in qdrant_results:
        # Extract metadata from payload
        payload = result.payload or {}
        metadata = payload.copy()  # Copy all payload data

        # Extract specific metadata fields
        source_url = payload.get("source_url", "")
        document_id = payload.get("document_id", "")
        chunk_index = payload.get("chunk_index", 0)
        page_number = payload.get("page_number", None)
        section_title = payload.get("section_title", "")

        # Create SourceMetadata object to validate the metadata
        try:
            source_metadata = SourceMetadata(
                source_url=source_url,
                document_id=document_id,
                chunk_index=chunk_index,
                page_number=page_number,
                section_title=section_title
            )

            # Add validated metadata to results
            validated_result = {
                "id": result.id,
                "text": payload.get("content", ""),  # Changed from "text" to "content" to match the actual field name
                "score": result.score,
                "vector_id": result.id,
                "metadata": {
                    "source_url": source_metadata.source_url,
                    "document_id": source_metadata.document_id,
                    "chunk_index": source_metadata.chunk_index,
                    "page_number": source_metadata.page_number,
                    "section_title": source_metadata.section_title,
                    # Include any additional metadata from payload
                    **{k: v for k, v in payload.items() if k not in ["content", "source_url", "document_id", "chunk_index", "page_number", "section_title"]}
                }
            }
            validated_results.append(validated_result)
        except ValueError as e:
            logger.warning(f"Metadata validation failed for result {result.id}: {str(e)}")
            # Include the result with available metadata but mark as having validation issues
            validated_result = {
                "id": result.id,
                "text": payload.get("text", ""),
                "score": result.score,
                "vector_id": result.id,
                "metadata": payload,
                "validation_warning": str(e)
            }
            validated_results.append(validated_result)

    return validated_results


def validate_source_url_and_document_id(metadata: Dict) -> bool:
    """Validate source URL and document ID in metadata"""
    source_url = metadata.get("source_url", "")
    document_id = metadata.get("document_id", "")

    # Check if source URL is valid
    is_valid_url = source_url and (source_url.startswith("http://") or source_url.startswith("https://"))

    # Check if document ID exists and is not empty
    is_valid_doc_id = bool(document_id and document_id.strip())

    return is_valid_url and is_valid_doc_id


def validate_retrieved_chunks(retrieved_chunks: List[Dict]) -> bool:
    """Basic validation for retrieved text chunks"""
    if not retrieved_chunks:
        return False

    # Check that we have valid results with text content
    valid_chunks = [chunk for chunk in retrieved_chunks if chunk.get('text', '').strip()]
    return len(valid_chunks) > 0


def main():
    """Main function to run the RAG retrieval validation"""
    parser = argparse.ArgumentParser(description='RAG Retrieval Validation Script')
    parser.add_argument('--query', type=str, help='Test query to validate retrieval')
    parser.add_argument('--top-k', type=int, default=Config.DEFAULT_TOP_K, help='Number of top results to retrieve (default: 5)')
    parser.add_argument('--collection', type=str, default=Config.COLLECTION_NAME, help='Qdrant collection name (default: book_chunks)')
    parser.add_argument('--validate-all', action='store_true', help='Run comprehensive validation suite')

    args = parser.parse_args()

    if args.validate_all:
        logger.info("Running comprehensive validation suite...")
        run_comprehensive_validation(args.collection, args.top_k)
    elif args.query:
        logger.info(f"Validating retrieval for query: '{args.query}'")
        result = validate_single_query(args.query, args.collection, args.top_k)
        print(json.dumps(result, indent=2))
    else:
        logger.info("No query provided. Use --query 'your query' or --validate-all")
        parser.print_help()


def validate_single_query(query: str, collection_name: str, top_k: int) -> Dict[str, Any]:
    """
    Validate a single query against the RAG retrieval system using updated implementation
    """
    try:
        # Initialize Qdrant client using configuration
        client = get_qdrant_client()

        # Load collection
        if not load_collection(client, collection_name):
            return {
                "query": query,
                "retrieved_chunks": [],
                "is_valid": False,
                "error_message": f"Collection '{collection_name}' does not exist in Qdrant",
                "validation_timestamp": time.time()
            }

        # Generate embedding for the query using Cohere
        query_embedding = generate_query_embedding(query)

        # Perform similarity search in Qdrant
        search_results = perform_similarity_search(client, query_embedding, collection_name, top_k)

        # Extract and validate metadata from Qdrant results
        validated_results = extract_and_validate_metadata(search_results)

        # Validate results
        is_valid = validate_retrieved_chunks(validated_results)

        return {
            "query": query,
            "retrieved_chunks": validated_results,
            "is_valid": is_valid,
            "error_message": None if is_valid else "No valid chunks retrieved for the query",
            "validation_timestamp": time.time()
        }

    except Exception as e:
        logger.error(f"Error during validation: {str(e)}")
        return {
            "query": query,
            "retrieved_chunks": [],
            "is_valid": False,
            "error_message": str(e),
            "validation_timestamp": time.time()
        }


def collection_exists(client: QdrantClient, collection_name: str) -> bool:
    """Check if a collection exists in Qdrant"""
    try:
        client.get_collection(collection_name)
        return True
    except:
        return False


def run_comprehensive_validation(collection_name: str, top_k: int) -> Dict[str, Any]:
    """
    Run comprehensive validation on the entire RAG retrieval pipeline
    """
    # Sample test queries for validation
    test_queries = [
        "What is humanoid robotics?",
        "Explain ROS 2 architecture",
        "How does computer vision work in robotics?",
        "What are the principles of robot kinematics?",
        "Explain path planning algorithms"
    ]

    validation_results = []
    successful_queries = 0
    start_time = time.time()

    for query in test_queries:
        logger.info(f"Validating query: '{query}'")
        result = validate_single_query(query, collection_name, top_k)
        validation_results.append(result)

        if result["is_valid"]:
            successful_queries += 1

    execution_time = time.time() - start_time
    success_rate = (successful_queries / len(test_queries)) * 100 if test_queries else 0

    # Create comprehensive validation report using the ValidationReport data model
    report = ValidationReport(
        report_id=f"validation_report_{int(time.time())}",
        total_queries=len(test_queries),
        successful_queries=successful_queries,
        failed_queries=len(test_queries) - successful_queries,
        details=[]
    )
    report.execution_time = execution_time

    # Format the report as a dictionary for JSON serialization
    final_report = {
        "report_id": report.report_id,
        "total_queries": report.total_queries,
        "successful_queries": report.successful_queries,
        "failed_queries": report.failed_queries,
        "success_rate": report.success_rate,
        "details": validation_results,
        "execution_time": report.execution_time
    }

    logger.info(f"Comprehensive validation completed. Success rate: {success_rate:.2f}%")
    print(json.dumps(final_report, indent=2))

    return final_report


if __name__ == "__main__":
    main()