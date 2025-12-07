from sqlalchemy import Column, String, DateTime, Text, ForeignKey
from sqlalchemy.dialects.postgresql import UUID
from .base import Base
import uuid
from datetime import datetime

class Query(Base):
    """
    Query model representing a user's question or request for information
    """
    __tablename__ = "queries"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(UUID(as_uuid=True), ForeignKey("conversations.id"), nullable=False)
    content = Column(Text, nullable=False)  # The actual text of the user's query
    query_type = Column(String, nullable=False)  # Enum: book_content, user_text, mixed
    created_at = Column(DateTime, default=datetime.utcnow)
    processed_at = Column(DateTime, nullable=True)
    user_preferences_json = Column(Text)  # JSON for user preferences for response style/level