from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.middleware.trustedhost import TrustedHostMiddleware
from contextlib import asynccontextmanager
from src.config.database import engine
from src.config.vector_db import initialize_collections
from src.api.chat import router as chat_router
from src.api.health import router as health_router
from src.api.documents import router as documents_router
from src.core.config import settings
import logging

# Set up basic logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    FastAPI lifespan event handler
    """
    # Startup
    logger.info("Initializing application...")

    try:
        # Initialize vector database collections
        initialize_collections()
        logger.info("Vector database initialized")
    except Exception as e:
        # Log the error but don't prevent the app from starting
        logger.error(f"Failed to initialize vector database: {e}")
        logger.warning("Application will start without vector database functionality")

    yield

    # Shutdown
    logger.info("Shutting down application...")

# Create FastAPI app with lifespan
app = FastAPI(
    title="RAG Chatbot API",
    description="API for the embedded RAG chatbot in the Physical AI & Humanoid Robotics book",
    version="1.0.0",
    lifespan=lifespan
)

# Add TrustedHost middleware to prevent HTTP Host Header attacks
app.add_middleware(
    TrustedHostMiddleware,
    allowed_hosts=settings.ALLOWED_ORIGINS if settings.ALLOWED_ORIGINS != ["*"] else ["*"]
)

# Add CORS middleware with more secure defaults
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.ALLOWED_ORIGINS,
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],  # Be specific about allowed methods
    allow_headers=["*"],  # Allow all headers by default
    # Additional security: expose only necessary headers
    expose_headers=["Access-Control-Allow-Origin"]
)


# Add security headers
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import Response
from typing import Callable


class SecurityHeadersMiddleware(BaseHTTPMiddleware):
    async def dispatch(self, request, call_next):
        response = await call_next(request)

        # Add security headers to all responses
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["X-Frame-Options"] = "DENY"  # Or "SAMEORIGIN" if needed
        response.headers["X-XSS-Protection"] = "1; mode=block"
        response.headers["Strict-Transport-Security"] = "max-age=31536000; includeSubDomains"
        response.headers["Referrer-Policy"] = "strict-origin-when-cross-origin"
        response.headers["Content-Security-Policy"] = "default-src 'self'; frame-ancestors 'none';"

        return response


# Add security headers middleware
app.add_middleware(SecurityHeadersMiddleware)

# Include API routers
app.include_router(chat_router, prefix="/chat", tags=["chat"])
app.include_router(health_router, tags=["health"])
app.include_router(documents_router, prefix="/documents", tags=["documents"])

@app.get("/")
def read_root():
    return {"message": "RAG Chatbot API is running!"}

if __name__ == "__main__":
    import uvicorn
    import os
    port = int(os.environ.get("PORT", 8000))
    uvicorn.run("main:app", host="0.0.0.0", port=port, reload=True)