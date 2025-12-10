from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.middleware.trustedhost import TrustedHostMiddleware
from contextlib import asynccontextmanager
import logging
import sys
import os
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import Response
from typing import Callable

# Set up basic logging first
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[logging.StreamHandler(sys.stdout)]
)
logger = logging.getLogger(__name__)

# Log startup
logger.info("=" * 50)
logger.info("Starting RAG Chatbot API")
logger.info(f"Python version: {sys.version}")
logger.info(f"Working directory: {os.getcwd()}")
logger.info("=" * 50)

# Import configuration with error handling
try:
    from src.core.config import settings
    logger.info("Configuration loaded successfully")
except Exception as e:
    logger.error(f"Failed to load configuration: {e}")
    # Create minimal settings
    class MinimalSettings:
        ALLOWED_ORIGINS = ["*"]
    settings = MinimalSettings()
    logger.warning("Using minimal settings")

# Import API routers with error handling
routers_loaded = []
try:
    from src.api.chat import router as chat_router
    routers_loaded.append(("chat", chat_router))
    logger.info("Chat router loaded")
except Exception as e:
    logger.error(f"Failed to load chat router: {e}")
    chat_router = None

try:
    from src.api.health import router as health_router
    routers_loaded.append(("health", health_router))
    logger.info("Health router loaded")
except Exception as e:
    logger.error(f"Failed to load health router: {e}")
    health_router = None

try:
    from src.api.documents import router as documents_router
    routers_loaded.append(("documents", documents_router))
    logger.info("Documents router loaded")
except Exception as e:
    logger.error(f"Failed to load documents router: {e}")
    documents_router = None

@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    FastAPI lifespan event handler
    """
    # Startup
    logger.info("Initializing application...")

    # Initialize vector database collections in a non-blocking way
    try:
        from src.config.vector_db import initialize_collections
        # Run initialization but don't block startup if it fails
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

# Include API routers only if they loaded successfully
if chat_router:
    app.include_router(chat_router, prefix="/chat", tags=["chat"])
    logger.info("Chat router registered")
if health_router:
    app.include_router(health_router, tags=["health"])
    logger.info("Health router registered")
if documents_router:
    app.include_router(documents_router, prefix="/documents", tags=["documents"])
    logger.info("Documents router registered")

logger.info(f"Total routers loaded: {len(routers_loaded)}")

@app.get("/")
def read_root():
    return {
        "message": "RAG Chatbot API is running!",
        "status": "healthy",
        "routers_loaded": len(routers_loaded),
        "port": os.environ.get("PORT", "not set")
    }

@app.get("/ping")
def ping():
    """Simple ping endpoint for health checks"""
    return {"status": "ok"}

if __name__ == "__main__":
    import uvicorn
    
    port = int(os.environ.get("PORT", 10000))
    logger.info(f"Starting server on 0.0.0.0:{port}")
    
    try:
        uvicorn.run(
            "main:app",
            host="0.0.0.0",
            port=port,
            reload=False,
            log_level="info",
            access_log=True
        )
    except Exception as e:
        logger.error(f"Failed to start server: {e}")
        sys.exit(1)
