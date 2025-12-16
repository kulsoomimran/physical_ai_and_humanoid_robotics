from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from typing import Optional
from uuid import UUID
from pydantic import BaseModel
from ..database.session import get_db
from ..services.document_ingestion_service import DocumentIngestionService
from ..services.rag_service import RAGService
from ..utils.logging_util import rag_logger
import os


# Initialize services
rag_service = RAGService(
    qdrant_url=os.getenv("QDRANT_URL", "https://your-qdrant-cluster.qdrant.tech:6333"),
    qdrant_api_key=os.getenv("QDRANT_API_KEY", "your-api-key-here"),
    user_collection_name="user_content"  # Use a specific collection for user content
)
document_ingestion_service = DocumentIngestionService(rag_service=rag_service)

router = APIRouter()


class IngestUserTextRequest(BaseModel):
    session_token: str
    text_content: str
    user_id: Optional[str] = None


class IngestUserTextResponse(BaseModel):
    message: str
    chunks_ingested: int
    session_token: str


@router.post("/ingest-user-text")
async def ingest_user_text(request: IngestUserTextRequest, db: Session = Depends(get_db)):
    """
    Ingest user-provided text for use in the RAG system
    """
    try:
        # Validate input
        if not request.text_content or len(request.text_content.strip()) == 0:
            raise HTTPException(status_code=400, detail="Text content cannot be empty")

        if len(request.text_content) > 10000:  # 10KB limit
            raise HTTPException(status_code=400, detail="Text content exceeds maximum length of 10000 characters")

        # Convert session token to UUID (in a real implementation, you'd validate the session token)
        # For now, we'll use the session token as the source ID
        try:
            # In a real implementation, you would validate the session token against the database
            # For now, we'll just use the session_token string as a source ID
            source_id = UUID(int=int(request.session_token.replace('-', ''), 16) % 2**128 if len(request.session_token) > 0 else 123456789)
        except:
            # If session token is not a valid UUID, generate a new one or use a placeholder
            # For this implementation, we'll just use a placeholder approach
            source_id = UUID(int=hash(request.session_token) % 2**128)

        user_id = UUID(request.user_id) if request.user_id else None

        # Ingest the user text
        chunks = await document_ingestion_service.ingest_user_text(
            db=db,
            user_text=request.text_content,
            session_id=source_id,
            user_id=user_id
        )

        # Log the ingestion
        rag_logger.log_rag_retrieval(
            query=f"User text ingestion for session {request.session_token}",
            retrieved_chunks_count=len(chunks),
            retrieval_time=0,  # Time tracking not applicable for ingestion
            context_mode="user_text"
        )

        # Unload the model to save memory after processing
        try:
            rag_service.unload_model()
        except:
            # If unload fails, continue anyway
            pass

        return IngestUserTextResponse(
            message=f"Successfully ingested user text into {len(chunks)} chunks",
            chunks_ingested=len(chunks),
            session_token=request.session_token
        )
    except HTTPException:
        raise
    except Exception as e:
        # Log the error
        rag_logger.log_error(
            error_type="user_text_ingestion_error",
            error_message=str(e),
            session_token=request.session_token,
            additional_info={"text_length": len(request.text_content)}
        )
        raise HTTPException(status_code=500, detail=f"Error ingesting user text: {str(e)}")


@router.get("/")
async def document_root():
    return {"message": "Document API endpoints for user text ingestion are available"}