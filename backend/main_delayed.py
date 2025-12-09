from fastapi import FastAPI
from contextlib import asynccontextmanager
import logging
import os

# Set up basic logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Import and include routers at module level so they appear in /docs
from fastapi.middleware.cors import CORSMiddleware
from fastapi.middleware.trustedhost import TrustedHostMiddleware
from src.core.config import settings

@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    FastAPI lifespan event handler - only do blocking operations here, after server starts
    """
    logger.info("Application starting up...")

    # Do all potentially blocking operations inside the lifespan
    try:
        # Initialize vector database collections
        try:
            from src.config.vector_db import initialize_collections
            initialize_collections()
            logger.info("Vector database initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize vector database: {e}")
            logger.warning("Application will run without vector database functionality")

        logger.info("All blocking startup operations completed successfully")

    except Exception as e:
        logger.error(f"Error during startup: {e}")
        # Still continue to allow server to start

    yield  # Run the application

    logger.info("Shutting down application...")

# Create FastAPI app with lifespan - this will start immediately
app = FastAPI(
    title="RAG Chatbot API",
    description="API for the embedded RAG chatbot in the Physical AI & Humanoid Robotics book",
    version="1.0.0",
    lifespan=lifespan
)

# Add middleware
app.add_middleware(
    TrustedHostMiddleware,
    allowed_hosts=settings.ALLOWED_ORIGINS if settings.ALLOWED_ORIGINS != ["*"] else ["*"]
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.ALLOWED_ORIGINS,
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allow_headers=["*"],
    expose_headers=["Access-Control-Allow-Origin"]
)

# Add security headers middleware
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import Response

class SecurityHeadersMiddleware(BaseHTTPMiddleware):
    async def dispatch(self, request, call_next):
        response = await call_next(request)
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["X-Frame-Options"] = "DENY"
        response.headers["X-XSS-Protection"] = "1; mode=block"
        response.headers["Strict-Transport-Security"] = "max-age=31536000; includeSubDomains"
        response.headers["Referrer-Policy"] = "strict-origin-when-cross-origin"
        response.headers["Content-Security-Policy"] = "default-src 'self'; frame-ancestors 'none';"
        return response

app.add_middleware(SecurityHeadersMiddleware)

# Import and include API routers at module level so they appear in docs
from src.api.health import router as health_router
from src.api.chat import router as chat_router
from src.api.documents import router as documents_router

app.include_router(health_router, tags=["health"])
app.include_router(chat_router, prefix="/chat", tags=["chat"])
app.include_router(documents_router, prefix="/documents", tags=["documents"])

@app.get("/")
def read_root():
    return {"message": "RAG Chatbot API is running!"}

if __name__ == "__main__":
    import uvicorn
    port = int(os.environ.get("PORT", 8000))
    uvicorn.run("main_delayed:app", host="0.0.0.0", port=port, reload=True)