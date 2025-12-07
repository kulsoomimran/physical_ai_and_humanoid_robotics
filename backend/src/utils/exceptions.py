from fastapi import HTTPException, status
from typing import Optional
import logging

logger = logging.getLogger(__name__)

class RAGChatbotException(HTTPException):
    """
    Base exception class for RAG Chatbot application
    """
    def __init__(self, detail: str, status_code: int = status.HTTP_500_INTERNAL_SERVER_ERROR):
        super().__init__(status_code=status_code, detail=detail)
        logger.error(f"RAGChatbotException: {detail}, Status: {status_code}")

class QueryProcessingError(RAGChatbotException):
    """
    Exception raised when there's an error processing a user query
    """
    def __init__(self, detail: str = "Error processing query"):
        super().__init__(detail=detail, status_code=status.HTTP_422_UNPROCESSABLE_ENTITY)

class DocumentProcessingError(RAGChatbotException):
    """
    Exception raised when there's an error processing document chunks
    """
    def __init__(self, detail: str = "Error processing document"):
        super().__init__(detail=detail, status_code=status.HTTP_422_UNPROCESSABLE_ENTITY)

class VectorDBError(RAGChatbotException):
    """
    Exception raised when there's an error with vector database operations
    """
    def __init__(self, detail: str = "Vector database error"):
        super().__init__(detail=detail, status_code=status.HTTP_500_INTERNAL_SERVER_ERROR)

class ConfigurationError(RAGChatbotException):
    """
    Exception raised when there's a configuration error
    """
    def __init__(self, detail: str = "Configuration error"):
        super().__init__(detail=detail, status_code=status.HTTP_500_INTERNAL_SERVER_ERROR)

# Error handling middleware
async def handle_exception(request, call_next):
    """
    Middleware to handle exceptions globally
    """
    try:
        response = await call_next(request)
    except RAGChatbotException:
        # These are already formatted HTTP exceptions
        raise
    except Exception as e:
        # Log the unexpected error
        logger.exception(f"Unexpected error occurred: {str(e)}")
        # Raise a generic internal server error
        raise RAGChatbotException(
            detail="An unexpected error occurred",
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR
        )

    return response