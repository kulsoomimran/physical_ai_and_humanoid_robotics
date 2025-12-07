from sqlalchemy import Column, String, DateTime, Text, ForeignKey
from sqlalchemy.dialects.postgresql import UUID
from .base import Base
import uuid
from datetime import datetime

class Response(Base):
    """
    Response model representing the system's response to a user query
    """
    __tablename__ = "responses"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    query_id = Column(UUID(as_uuid=True), ForeignKey("queries.id"), nullable=False)
    content = Column(Text, nullable=False)  # The actual text of the response
    response_type = Column(String)  # Enum: factual, explanatory, comparative
    created_at = Column(DateTime, default=datetime.utcnow)
    processing_time = Column(String)  # Time taken to generate the response in seconds