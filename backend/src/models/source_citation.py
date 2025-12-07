from sqlalchemy import Column, String, DateTime, Text, Float, ForeignKey
from sqlalchemy.dialects.postgresql import UUID
from .base import Base
import uuid
from datetime import datetime

class SourceCitation(Base):
    """
    Source Citation model representing reference to specific part of book or user text
    """
    __tablename__ = "source_citations"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    response_id = Column(UUID(as_uuid=True), ForeignKey("responses.id"), nullable=False)
    chunk_id = Column(UUID(as_uuid=True), ForeignKey("document_chunks.id"), nullable=False)
    relevance_score = Column(Float)  # Score indicating how relevant this chunk was to the response
    citation_text = Column(Text)  # The specific text that was cited
    source_location = Column(String)  # Location in the original source (page, section, etc.)