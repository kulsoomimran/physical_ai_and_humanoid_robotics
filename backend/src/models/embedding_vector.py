from sqlalchemy import Column, String, DateTime, Text, ForeignKey
from sqlalchemy.dialects.postgresql import UUID
from .base import Base
import uuid
from datetime import datetime


class EmbeddingVector(Base):
    """
    Embedding Vector model representing the numerical representation of text content used for similarity matching in the vector database
    """
    __tablename__ = "embedding_vectors"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    chunk_id = Column(UUID(as_uuid=True), ForeignKey("document_chunks.id"), nullable=False)  # Reference to the source document chunk
    vector_db_id = Column(String, unique=True, nullable=False)  # ID in the vector database (Qdrant)
    model_used = Column(String, nullable=False)  # Name of the model used to generate the embedding
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)