from pydantic_settings import BaseSettings
from typing import List, Optional
import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

class Settings(BaseSettings):
    """
    Application settings loaded from environment variables
    """
    # API settings
    API_V1_STR: str = "/v1"
    PROJECT_NAME: str = "RAG Chatbot API"

    # CORS settings - more secure defaults
    ALLOWED_ORIGINS: List[str] = os.getenv("ALLOWED_ORIGINS", "http://localhost:3000,http://127.0.0.1:3000,http://localhost:8000,https://ai-and-humanoid-robotics-book.vercel.app").split(",")

    # Database settings
    DATABASE_URL: Optional[str] = os.getenv("DATABASE_URL")

    # Qdrant settings
    QDRANT_URL: Optional[str] = os.getenv("QDRANT_URL")
    QDRANT_API_KEY: Optional[str] = os.getenv("QDRANT_API_KEY")
    QDRANT_COLLECTION_NAME: str = os.getenv("QDRANT_COLLECTION_NAME", "document_chunks")

    # OpenAI API settings
    OPENAI_API_KEY: Optional[str] = os.getenv("OPENAI_API_KEY")
    OPENAI_ASSISTANT_ID: Optional[str] = os.getenv("OPENAI_ASSISTANT_ID")

    # Gemini API settings (kept for compatibility)
    GEMINI_API_KEY: str = os.getenv("GEMINI_API_KEY", "")
    GEMINI_MODEL: str = os.getenv("GEMINI_MODEL", "gemini-1.5-flash")

    # Application settings
    DEBUG: bool = os.getenv("DEBUG", "False").lower() == "true"
    LOG_LEVEL: str = os.getenv("LOG_LEVEL", "INFO")

    # Security settings
    SECRET_KEY: str = os.getenv("SECRET_KEY", "dev-secret-key-change-in-production")
    ALGORITHM: str = os.getenv("ALGORITHM", "HS256")
    ACCESS_TOKEN_EXPIRE_MINUTES: int = int(os.getenv("ACCESS_TOKEN_EXPIRE_MINUTES", "30"))

    # Environment settings
    ENVIRONMENT: str = os.getenv("ENVIRONMENT", "development")

    class Config:
        env_file = ".env"
        case_sensitive = True

settings = Settings()