from pydantic import BaseModel, Field, field_validator
from typing import Optional, Dict, Any
from uuid import UUID
from datetime import datetime
import re


class ChatRequest(BaseModel):
    """
    Represents a user query with optional selected text context
    """
    question: str = Field(
        ...,
        description="The user's question or query",
        min_length=1,
        max_length=2000
    )
    selected_text: Optional[str] = Field(
        None,
        description="Additional context from selected text",
        max_length=10000
    )
    thread_id: Optional[str] = Field(
        None,
        description="ID to maintain conversation context",
        pattern=r"^[a-zA-Z0-9_-]+$"
    )
    metadata: Optional[Dict[str, Any]] = Field(
        None,
        description="Additional metadata for the request"
    )

    @field_validator('question')
    @classmethod
    def validate_question(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('Question must not be empty')
        if len(v) > 2000:
            raise ValueError('Question must be between 1 and 2000 characters')
        return v.strip()

    @field_validator('selected_text')
    @classmethod
    def validate_selected_text(cls, v):
        if v is not None and len(v) > 10000:
            raise ValueError('Selected text must be between 0 and 10000 characters')
        return v

    @field_validator('thread_id')
    @classmethod
    def validate_thread_id(cls, v):
        if v is not None:
            # Check if thread_id matches the required pattern
            if not re.match(r'^[a-zA-Z0-9_-]+$', v):
                raise ValueError('thread_id must contain only alphanumeric characters, underscores, and hyphens')
        return v


class ChatResponse(BaseModel):
    """
    Represents the response from the RAG agent
    """
    response: str = Field(
        ...,
        description="The agent's response to the question",
        min_length=1,
        max_length=10000
    )
    thread_id: str = Field(
        ...,
        description="ID of the conversation thread",
        pattern=r"^[a-zA-Z0-9_-]+$"
    )
    metadata: Optional[Dict[str, Any]] = Field(
        None,
        description="Additional metadata about the response"
    )
    timestamp: str = Field(
        ...,
        description="ISO 8601 formatted timestamp"
    )

    @field_validator('response')
    @classmethod
    def validate_response(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('Response must not be empty')
        if len(v) > 10000:
            raise ValueError('Response must be between 1 and 10000 characters')
        return v.strip()

    @field_validator('timestamp')
    @classmethod
    def validate_timestamp(cls, v):
        try:
            # Try to parse the timestamp to ensure it's in ISO 8601 format
            datetime.fromisoformat(v.replace('Z', '+00:00'))
        except ValueError:
            raise ValueError('Timestamp must be in ISO 8601 format')
        return v

    @field_validator('thread_id')
    @classmethod
    def validate_response_thread_id(cls, v):
        # Check if thread_id matches the required pattern
        if not re.match(r'^[a-zA-Z0-9_-]+$', v):
            raise ValueError('thread_id must contain only alphanumeric characters, underscores, and hyphens')
        return v