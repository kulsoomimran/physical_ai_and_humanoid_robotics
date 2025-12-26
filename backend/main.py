from fastapi import FastAPI, HTTPException, BackgroundTasks, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
import logging
from datetime import datetime
from typing import Optional
import sys
import os
from .models.chat import ChatRequest, ChatResponse
from .models.health import HealthCheck
from .models.error import Error
from .services.rag_agent import RAGAgentService
from .config import settings
from .logging_config import setup_logging
from .exceptions import RAGChatbotException

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Setup logging
setup_logging(settings.log_level)
logger = logging.getLogger(__name__)

# Create FastAPI app
app = FastAPI(
    title="RAG Chatbot Backend API",
    description="API for interacting with a RAG agent through a FastAPI backend",
    version="1.0.0"
)

# Add CORS middleware for cross-origin requests
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize RAG agent service
rag_agent_service = RAGAgentService()

# Custom exception handler
@app.exception_handler(RAGChatbotException)
async def custom_exception_handler(request: Request, exc: RAGChatbotException):
    """
    Custom exception handler for RAGChatbotException
    """
    logger.error(f"RAGChatbotException: {exc.error_message}, details: {exc.details}")

    error_response = exc.to_error_model()

    return JSONResponse(
        status_code=exc.status_code,
        content=error_response.dict()
    )

@app.exception_handler(Exception)
async def general_exception_handler(request: Request, exc: Exception):
    """
    General exception handler for unexpected errors
    """
    logger.error(f"Unexpected error: {str(exc)}", exc_info=True)

    error_response = Error(
        error="An unexpected error occurred",
        code="500",
        details={"type": type(exc).__name__}
    )

    return JSONResponse(
        status_code=500,
        content=error_response.dict()
    )

@app.get("/")
async def root():
    """
    Root endpoint to check if the API is running
    """
    return {"message": "RAG Chatbot Backend API is running"}

@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(chat_request: ChatRequest):
    """
    Process a user question through the RAG agent and return the response
    """
    try:
        logger.info(f"Received chat request for thread: {chat_request.thread_id}")

        # Process the request using the RAG agent service (await the async method)
        result = await rag_agent_service.chat(
            question=chat_request.question,
            thread_id=chat_request.thread_id,
            selected_text=chat_request.selected_text
        )

        # Create response object
        response = ChatResponse(
            response=result["response"],
            thread_id=result["thread_id"],
            timestamp=result["timestamp"],
            metadata=result.get("metadata", {})
        )

        logger.info(f"Successfully processed chat request for thread: {response.thread_id}")
        return response

    except Exception as e:
        logger.error(f"Error processing chat request: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/health", response_model=HealthCheck)
async def health_check():
    """
    Check the health status of the service
    """
    try:
        health_status = HealthCheck(
            status="healthy",
            timestamp=datetime.now().isoformat(),
            version="1.0.0"
        )
        logger.info("Health check performed successfully")
        return health_status
    except Exception as e:
        logger.error(f"Health check failed: {str(e)}")
        health_status = HealthCheck(
            status="unhealthy",
            timestamp=datetime.now().isoformat(),
            version="1.0.0"
        )
        return health_status

# Additional endpoints for User Story 2 and 3 will be implemented as needed

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=True
    )