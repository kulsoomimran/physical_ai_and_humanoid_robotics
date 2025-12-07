"""
Security utilities for the RAG Chatbot application
"""
import re
from typing import Optional
from pydantic import BaseModel, field_validator
from fastapi import HTTPException
import html
from .rate_limit_util import check_rate_limit as actual_check_rate_limit


class SecurityConfig:
    """
    Security configuration constants
    """
    # Input validation
    MAX_QUERY_LENGTH = 2000  # Maximum length for user queries
    MAX_SESSION_TOKEN_LENGTH = 100  # Maximum length for session tokens
    MAX_CONTEXT_MODE_LENGTH = 20  # Maximum length for context mode

    # Rate limiting (these would typically be configured via middleware)
    MAX_QUERIES_PER_MINUTE = 30
    MAX_QUERIES_PER_HOUR = 500


def sanitize_input(text: str) -> str:
    """
    Sanitize user input to prevent injection attacks
    """
    if not text:
        return text

    # Remove potentially dangerous characters/sequences
    # This is a basic implementation - in production, use a more robust sanitization library
    sanitized = html.escape(text)

    # Additional sanitization could include:
    # - Removing SQL injection patterns
    # - Removing script tags (if accepting HTML)
    # - Normalizing Unicode characters

    return sanitized


def validate_session_token(session_token: str) -> bool:
    """
    Validate session token format
    """
    if not session_token:
        return False

    if len(session_token) > SecurityConfig.MAX_SESSION_TOKEN_LENGTH:
        return False

    # Check if it looks like a UUID or a properly formatted token
    # This is a basic check - adjust based on your token format
    uuid_pattern = re.compile(
        r'^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$',
        re.IGNORECASE
    )

    return bool(uuid_pattern.match(session_token))


def validate_query_length(query: str) -> bool:
    """
    Validate query length is within acceptable limits
    """
    return len(query) <= SecurityConfig.MAX_QUERY_LENGTH if query else True


def validate_context_mode(context_mode: str) -> bool:
    """
    Validate context mode is one of the allowed values
    """
    allowed_modes = ["book_content", "user_text", "mixed"]
    return context_mode in allowed_modes


def validate_response_style(response_style: Optional[str]) -> bool:
    """
    Validate response style is one of the allowed values
    """
    if response_style is None:
        return True

    allowed_styles = ["standard", "simplified", "detailed", "examples"]
    return response_style in allowed_styles


class SecureQueryRequest(BaseModel):
    """
    Secure version of QueryRequest with validation
    """
    session_token: str
    query: str
    context_mode: str = "book_content"
    response_style: Optional[str] = None

    @field_validator('session_token')
    @classmethod
    def validate_session_token_format(cls, v):
        if not validate_session_token(v):
            raise ValueError('Invalid session token format')
        return v

    @field_validator('query')
    @classmethod
    def validate_query_content(cls, v):
        if not validate_query_length(v):
            raise ValueError(f'Query exceeds maximum length of {SecurityConfig.MAX_QUERY_LENGTH} characters')

        # Sanitize the query
        return sanitize_input(v)

    @field_validator('context_mode')
    @classmethod
    def validate_context_mode_value(cls, v):
        if not validate_context_mode(v):
            raise ValueError(f'Invalid context mode. Allowed values: book_content, user_text, mixed')
        return v

    @field_validator('response_style')
    @classmethod
    def validate_response_style_value(cls, v):
        if not validate_response_style(v):
            raise ValueError(f'Invalid response style. Allowed values: standard, simplified, detailed, examples')
        return v


def check_rate_limit(session_token: str) -> bool:
    """
    Check if the session has exceeded rate limits
    """
    return actual_check_rate_limit(session_token)


def log_security_event(event_type: str, details: dict):
    """
    Log security-related events
    """
    # In a real implementation, this would log to a security-specific log
    # For now, we'll just print (in production, use proper logging)
    print(f"SECURITY EVENT: {event_type} - {details}")