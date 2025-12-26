from fastapi import HTTPException, status
from typing import Optional, Dict, Any
from .models.error import Error


class RAGChatbotException(HTTPException):
    """
    Custom exception class for RAG Chatbot API
    """
    def __init__(
        self,
        status_code: int,
        error_message: str,
        error_code: Optional[str] = None,
        details: Optional[Dict[str, Any]] = None
    ):
        super().__init__(status_code=status_code, detail=error_message)
        self.error_message = error_message
        self.error_code = error_code
        self.details = details or {}

    def to_error_model(self) -> Error:
        """
        Convert this exception to the Error model for API responses
        """
        return Error(
            error=self.error_message,
            code=str(self.status_code),
            details=self.details
        )


class InvalidInputException(RAGChatbotException):
    """
    Exception raised when input validation fails
    """
    def __init__(self, error_message: str, details: Optional[Dict[str, Any]] = None):
        super().__init__(
            status_code=status.HTTP_400_BAD_REQUEST,
            error_message=error_message,
            error_code="INVALID_INPUT",
            details=details or {}
        )


class ResourceNotFoundException(RAGChatbotException):
    """
    Exception raised when a requested resource is not found
    """
    def __init__(self, error_message: str, details: Optional[Dict[str, Any]] = None):
        super().__init__(
            status_code=status.HTTP_404_NOT_FOUND,
            error_message=error_message,
            error_code="RESOURCE_NOT_FOUND",
            details=details or {}
        )


class InternalServerErrorException(RAGChatbotException):
    """
    Exception raised when an internal server error occurs
    """
    def __init__(self, error_message: str, details: Optional[Dict[str, Any]] = None):
        super().__init__(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            error_message=error_message,
            error_code="INTERNAL_SERVER_ERROR",
            details=details or {}
        )


class ServiceUnavailableException(RAGChatbotException):
    """
    Exception raised when a service is unavailable
    """
    def __init__(self, error_message: str, details: Optional[Dict[str, Any]] = None):
        super().__init__(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            error_message=error_message,
            error_code="SERVICE_UNAVAILABLE",
            details=details or {}
        )