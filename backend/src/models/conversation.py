from sqlalchemy import Column, String, DateTime, Boolean, Text
from sqlalchemy.dialects.postgresql import UUID
from .base import Base
import uuid
from datetime import datetime

class Conversation(Base):
    """
    Conversation model representing a sequence of related queries and responses
    """
    __tablename__ = "conversations"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_token = Column(String, unique=True, nullable=False)
    user_id = Column(UUID(as_uuid=True), nullable=True)  # Optional reference to user
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    metadata_json = Column(Text)  # JSON metadata about the conversation
    active = Column(Boolean, default=True)