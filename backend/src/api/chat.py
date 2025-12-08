from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from typing import Optional
from uuid import UUID
import uuid
import time
from ..database.session import get_db
from ..models.conversation import Conversation
from ..services.conversation_service import ConversationService
from ..services.rag_service import RAGService
from ..services.openai_agent_service import OpenAIAgentService
from ..utils.logging_util import rag_logger
from pydantic import BaseModel
from ..utils.security_util import (
    SecureQueryRequest,
    validate_session_token,
    validate_query_length,
    validate_context_mode,
    validate_response_style,
    check_rate_limit,
    log_security_event,
    sanitize_input
)
import os


# Initialize services - these would typically be configured via dependency injection
from ..core.config import settings

conversation_service = ConversationService()
rag_service = RAGService(
    qdrant_url=settings.QDRANT_URL,
    qdrant_api_key=settings.QDRANT_API_KEY
)

# Determine if we should use Gemini API (cost-effective) or OpenAI
use_gemini = settings.GEMINI_API_KEY and settings.GEMINI_API_KEY.strip() != ""
openai_agent_service = OpenAIAgentService(
    assistant_id=settings.OPENAI_ASSISTANT_ID,
    use_gemini=use_gemini
)


router = APIRouter()


# Using SecureQueryRequest instead of QueryRequest for enhanced security
# The QueryRequest class is kept for backward compatibility if needed
# class QueryRequest(BaseModel):
#     session_token: str
#     query: str
#     context_mode: str = "book_content"  # book_content, user_text, mixed
#     response_style: Optional[str] = None  # simplified, detailed, examples


class QueryResponse(BaseModel):
    response: str
    session_token: str
    query_id: str
    response_id: str
    source_citations: list = []


class StartChatRequest(BaseModel):
    user_id: Optional[str] = None


class StartChatResponse(BaseModel):
    session_token: str
    conversation_id: str


@router.post("/start")
async def start_chat(request: StartChatRequest, db: Session = Depends(get_db)):
    """
    Start a new chat session
    """
    session_token = str(uuid.uuid4())
    conversation = conversation_service.create_conversation(
        db, session_token, UUID(request.user_id) if request.user_id else None
    )

    # Log conversation start
    rag_logger.log_conversation_start(session_token, request.user_id)

    return StartChatResponse(
        session_token=conversation.session_token,
        conversation_id=str(conversation.id)
    )


@router.post("/{session_token}/query")
async def query_chat(session_token: str, request: SecureQueryRequest, db: Session = Depends(get_db)):
    """
    Submit a query to the chatbot and get a response
    """
    # Additional security checks
    if not validate_session_token(session_token):
        log_security_event("invalid_session_token", {
            "session_token": session_token,
            "client_ip": "unknown"  # Would get from request in real implementation
        })
        raise HTTPException(status_code=400, detail="Invalid session token format")

    # Check rate limits
    if not check_rate_limit(session_token):
        log_security_event("rate_limit_exceeded", {
            "session_token": session_token
        })
        raise HTTPException(status_code=429, detail="Rate limit exceeded")

    # Get or create conversation
    conversation = conversation_service.get_conversation_by_session_token(db, session_token)
    if not conversation:
        rag_logger.log_error(
            error_type="conversation_not_found",
            error_message="Conversation not found for session token",
            session_token=session_token
        )
        raise HTTPException(status_code=404, detail="Conversation not found")

    # Add query to conversation
    query = conversation_service.add_query_to_conversation(
        db,
        conversation_id=conversation.id,
        content=request.query,
        query_type=request.context_mode,
        user_preferences={"response_style": request.response_style} if request.response_style else None
    )

    # Log the query
    rag_logger.log_query(
        session_token=session_token,
        query_id=str(query.id),
        query_text=request.query,
        query_type=request.context_mode,
        user_preferences={"response_style": request.response_style} if request.response_style else None
    )

    # Start timing for processing
    start_time = time.time()

    try:
        # Retrieve relevant context using RAG service
        retrieval_start_time = time.time()
        context_chunks = await rag_service.retrieve_context_for_query(request.query, request.context_mode)
        retrieval_time = time.time() - retrieval_start_time

        # Log RAG retrieval
        rag_logger.log_rag_retrieval(
            query=request.query,
            retrieved_chunks_count=len(context_chunks),
            retrieval_time=retrieval_time,
            context_mode=request.context_mode
        )

        # Get recent conversation history for context
        recent_queries = conversation_service.get_recent_queries(db, conversation.id, limit=3)
        conversation_history = []
        for q in recent_queries:
            # Get the corresponding response
            from ..models.response import Response
            response = db.query(Response).filter(
                Response.query_id == q.id
            ).first()
            if response:
                conversation_history.append({
                    "query": q.content,
                    "response": response.content
                })

        # Generate response using OpenAI Agent with Assistant API
        openai_start_time = time.time()
        response_content = await openai_agent_service.create_thread_and_run_with_history(
            query=request.query,
            conversation_history=conversation_history,
            context=context_chunks,
            response_style=request.response_style or "standard"
        )
        openai_time = time.time() - openai_start_time

        # Log API call (either OpenAI or Gemini based on the service configuration)
        if openai_agent_service.is_using_gemini():
            rag_logger.log_gemini_api_call(
                query_length=len(request.query),
                response_length=len(response_content),
                api_call_time=openai_time,
                response_style=request.response_style or "standard"
            )
        else:
            rag_logger.log_openai_api_call(
                query_length=len(request.query),
                response_length=len(response_content),
                api_call_time=openai_time,
                response_style=request.response_style or "standard"
            )

        # Calculate total processing time
        processing_time = time.time() - start_time

        # Add response to query
        response = conversation_service.add_response_to_query(
            db,
            query_id=query.id,
            content=response_content,
            response_type="factual",  # This could be determined by the request
            processing_time=processing_time
        )

        # Add source citations
        citations = rag_service.add_source_citations(db, response.id, context_chunks)

        # Log the response
        rag_logger.log_response(
            session_token=session_token,
            query_id=str(query.id),
            response_id=str(response.id),
            response_text=response_content,
            processing_time=processing_time,
            source_citations_count=len(citations)
        )

        # Prepare citation data for response
        citation_data = [
            {
                "chunk_id": str(citation.chunk_id),
                "relevance_score": citation.relevance_score,
                "citation_text": citation.citation_text[:200] + "..." if len(citation.citation_text) > 200 else citation.citation_text,
                "source_location": citation.source_location
            }
            for citation in citations
        ]

        # Unload the model to save memory after processing
        try:
            rag_service.unload_model()
        except:
            # If unload fails, continue anyway
            pass

        return QueryResponse(
            response=response_content,
            session_token=session_token,
            query_id=str(query.id),
            response_id=str(response.id),
            source_citations=citation_data
        )
    except Exception as e:
        # Log any errors that occur during processing
        rag_logger.log_error(
            error_type="query_processing_error",
            error_message=str(e),
            session_token=session_token,
            query_id=str(query.id) if 'query' in locals() else None,
            additional_info={"request_context_mode": request.context_mode}
        )
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")


@router.get("/{session_token}/history")
async def get_conversation_history(session_token: str, db: Session = Depends(get_db)):
    """
    Get the history of a conversation
    """
    # Validate session token format
    if not validate_session_token(session_token):
        log_security_event("invalid_session_token", {
            "session_token": session_token,
            "client_ip": "unknown"  # Would get from request in real implementation
        })
        raise HTTPException(status_code=400, detail="Invalid session token format")

    conversation = conversation_service.get_conversation_by_session_token(db, session_token)
    if not conversation:
        raise HTTPException(status_code=404, detail="Conversation not found")

    # Get recent queries from the conversation
    recent_queries = conversation_service.get_recent_queries(db, conversation.id, limit=10)

    history = []
    for query in recent_queries:
        # Get the corresponding response
        from ..models.response import Response
        response = db.query(Response).filter(
            Response.query_id == query.id
        ).first()

        history.append({
            "query_id": str(query.id),
            "query_content": query.content,
            "response_content": response.content if response else None,
            "created_at": query.created_at.isoformat() if query.created_at else None
        })

    # Reverse to show oldest first
    history.reverse()

    return {"session_token": session_token, "history": history}