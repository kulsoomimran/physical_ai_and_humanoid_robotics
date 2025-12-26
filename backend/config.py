import os
from typing import Optional
from pydantic import BaseModel, Field
from dotenv import load_dotenv


# Load environment variables from .env file
load_dotenv()


class Settings(BaseModel):
    """
    Application settings loaded from environment variables
    """
    # API Configuration
    api_title: str = Field(default="RAG Chatbot Backend API", env="API_TITLE")
    api_description: str = Field(default="API for interacting with a RAG agent through a FastAPI backend", env="API_DESCRIPTION")
    api_version: str = Field(default="1.0.0", env="API_VERSION")
    debug: bool = Field(default=False, env="DEBUG")
    log_level: str = Field(default="INFO", env="LOG_LEVEL")
    port: int = Field(default=8000, env="PORT")

    # Agent Configuration
    openrouter_api_key: Optional[str] = Field(default=None, env="OPENROUTER_API_KEY")
    cohere_api_key: Optional[str] = Field(default=None, env="COHERE_API_KEY")
    qdrant_url: Optional[str] = Field(default=None, env="QDRANT_URL")
    qdrant_api_key: Optional[str] = Field(default=None, env="QDRANT_API_KEY")
    qdrant_host: str = Field(default="localhost", env="QDRANT_HOST")
    qdrant_port: int = Field(default=6333, env="QDRANT_PORT")
    qdrant_collection: str = Field(default="book_content", env="QDRANT_COLLECTION")

    class Config:
        env_file = ".env"
        case_sensitive = True


# Create global settings instance
settings = Settings()


def get_settings() -> Settings:
    """
    Get the application settings instance
    """
    return settings