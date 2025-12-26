from pydantic import BaseModel, Field
from typing import Optional
from datetime import datetime


class HealthCheck(BaseModel):
    """
    Represents the health status of the backend service
    """
    status: str = Field(
        ...,
        description="Health status of the service",
        pattern=r"^(healthy|unhealthy)$"
    )
    timestamp: str = Field(
        ...,
        description="ISO 8601 formatted timestamp"
    )
    version: Optional[str] = Field(
        None,
        description="API version information"
    )

    def __init__(self, **data):
        super().__init__(**data)
        if self.status not in ["healthy", "unhealthy"]:
            raise ValueError("Status must be either 'healthy' or 'unhealthy'")

    def __str__(self):
        return f"HealthCheck(status={self.status}, timestamp={self.timestamp}, version={self.version})"