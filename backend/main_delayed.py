from fastapi import FastAPI
from contextlib import asynccontextmanager
import logging
import os

# Set up basic logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    FastAPI lifespan event handler - only do imports here, after server starts
    """
    logger.info("Application starting up...")

    # Do all potentially blocking imports inside the lifespan
    try:
        # Import CORS middleware
        from fastapi.middleware.cors import CORSMiddleware
        from fastapi.middleware.trustedhost import TrustedHostMiddleware

        # Add middleware after imports
        app.add_middleware(
            TrustedHostMiddleware,
            allowed_hosts=os.getenv("ALLOWED_ORIGINS", "http://localhost:3000,http://127.0.0.1:3000,http://localhost:8000").split(",")
        )

        app.add_middleware(
            CORSMiddleware,
            allow_origins=os.getenv("ALLOWED_ORIGINS", "http://localhost:3000,http://127.0.0.1:3000,http://localhost:8000").split(","),
            allow_credentials=True,
            allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
            allow_headers=["*"],
            expose_headers=["Access-Control-Allow-Origin"]
        )

        # Import and initialize security headers
        from starlette.middleware.base import BaseHTTPMiddleware
        from starlette.responses import Response
        from src.core.config import settings

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

        # Import and include API routers
        from src.api.health import router as health_router
        from src.api.chat import router as chat_router
        from src.api.documents import router as documents_router

        app.include_router(health_router, tags=["health"])
        app.include_router(chat_router, prefix="/chat", tags=["chat"])
        app.include_router(documents_router, prefix="/documents", tags=["documents"])

        logger.info("All imports and setup completed successfully")

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

@app.get("/")
def read_root():
    return {"message": "RAG Chatbot API is running!"}

if __name__ == "__main__":
    import uvicorn
    port = int(os.environ.get("PORT", 8000))
    uvicorn.run("main_delayed:app", host="0.0.0.0", port=port, reload=True)