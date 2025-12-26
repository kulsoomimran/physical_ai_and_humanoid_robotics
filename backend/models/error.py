from pydantic import BaseModel, Field
from typing import Optional, Dict, Any


class Error(BaseModel):
    """
    Represents error responses from the API
    """
    error: str = Field(
        ...,
        description="Error message",
        min_length=1,
        max_length=500
    )
    code: str = Field(
        ...,
        description="Error code (HTTP status code)"
    )
    details: Optional[Dict[str, Any]] = Field(
        None,
        description="Additional error details"
    )

    def __init__(self, **data):
        super().__init__(**data)
        # Validate that code is a valid HTTP status code
        try:
            code_int = int(self.code)
            if code_int < 100 or code_int > 599:
                raise ValueError("Code must be a valid HTTP status code (100-599)")
        except ValueError:
            raise ValueError("Code must be a valid HTTP status code")

    def __str__(self):
        return f"Error(error={self.error}, code={self.code}, details={self.details})"