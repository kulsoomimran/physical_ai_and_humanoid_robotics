from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import Optional
import os
from dotenv import load_dotenv

load_dotenv()

class VectorDBConfig:
    """
    Configuration class for Qdrant vector database
    """
    QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")
    QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
    COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "document_chunks")

    # Vector dimensions for sentence-transformers all-MiniLM-L6-v2 model
    VECTOR_DIMENSION = 384

def get_qdrant_client() -> QdrantClient:
    """
    Get Qdrant client instance with proper configuration
    """
    if VectorDBConfig.QDRANT_API_KEY:
        client = QdrantClient(
            url=VectorDBConfig.QDRANT_URL,
            api_key=VectorDBConfig.QDRANT_API_KEY
        )
    else:
        client = QdrantClient(url=VectorDBConfig.QDRANT_URL)

    return client

def initialize_collections():
    """
    Initialize required collections in Qdrant
    """
    client = get_qdrant_client()

    # Check if collection already exists
    collection_names = [collection.name for collection in client.get_collections().collections]

    if VectorDBConfig.COLLECTION_NAME not in collection_names:
        client.create_collection(
            collection_name=VectorDBConfig.COLLECTION_NAME,
            vectors_config=models.VectorParams(
                size=VectorDBConfig.VECTOR_DIMENSION,
                distance=models.Distance.COSINE
            )
        )

        # Create payload index for efficient filtering
        client.create_payload_index(
            collection_name=VectorDBConfig.COLLECTION_NAME,
            field_name="source_id",
            field_schema=models.PayloadSchemaType.KEYWORD
        )

        client.create_payload_index(
            collection_name=VectorDBConfig.COLLECTION_NAME,
            field_name="source_type",
            field_schema=models.PayloadSchemaType.KEYWORD
        )

        print(f"Created collection '{VectorDBConfig.COLLECTION_NAME}' with vector dimension {VectorDBConfig.VECTOR_DIMENSION}")
    else:
        print(f"Collection '{VectorDBConfig.COLLECTION_NAME}' already exists")

if __name__ == "__main__":
    initialize_collections()