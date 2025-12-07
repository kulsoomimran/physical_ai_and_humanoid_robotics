from sqlalchemy import Column, String, DateTime, Text, Integer, ForeignKey
from sqlalchemy.dialects.postgresql import UUID
from .base import Base
import uuid
from datetime import datetime

class DocumentChunk(Base):
    """
    Document Chunk model representing segments of book content or user-provided text
    """
    __tablename__ = "document_chunks"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    source_id = Column(UUID(as_uuid=True), nullable=False)  # Reference to the original document/source
    source_type = Column(String, nullable=False)  # Enum: book_content, user_text
    content = Column(Text, nullable=False)  # The actual text content of the chunk
    chunk_order = Column(Integer)  # Order of this chunk within the original document
    metadata_json = Column(Text)  # Additional metadata about the chunk
    created_at = Column(DateTime, default=datetime.utcnow)