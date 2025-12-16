import logging
import json
from datetime import datetime
from typing import Dict, Any
from .monitoring_util import monitoring_service


class RAGChatbotLogger:
    """
    Logger class for RAG Chatbot operations with monitoring integration
    """

    def __init__(self, name: str = "rag_chatbot"):
        self.logger = logging.getLogger(name)
        self.logger.setLevel(logging.INFO)

        # Create console handler if not already exists
        if not self.logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            handler.setFormatter(formatter)
            self.logger.addHandler(handler)

        # Reference to monitoring service for metrics
        self.monitoring_service = monitoring_service

    def log_query(self, session_token: str, query_id: str, query_text: str,
                  query_type: str, user_preferences: Dict[str, Any] = None):
        """
        Log a query operation
        """
        log_data = {
            "event": "query_received",
            "session_token": session_token,
            "query_id": query_id,
            "query_text": query_text,
            "query_type": query_type,
            "user_preferences": user_preferences,
            "timestamp": datetime.utcnow().isoformat()
        }
        self.logger.info(f"QUERY: {json.dumps(log_data)}")

    def log_response(self, session_token: str, query_id: str, response_id: str,
                     response_text: str, processing_time: float, source_citations_count: int):
        """
        Log a response operation
        """
        log_data = {
            "event": "response_generated",
            "session_token": session_token,
            "query_id": query_id,
            "response_id": response_id,
            "response_length": len(response_text),
            "processing_time": processing_time,
            "source_citations_count": source_citations_count,
            "timestamp": datetime.utcnow().isoformat()
        }
        self.logger.info(f"RESPONSE: {json.dumps(log_data)}")

    def log_rag_retrieval(self, query: str, retrieved_chunks_count: int,
                         retrieval_time: float, context_mode: str):
        """
        Log RAG retrieval operation
        """
        log_data = {
            "event": "rag_retrieval",
            "query_length": len(query),
            "retrieved_chunks_count": retrieved_chunks_count,
            "retrieval_time": retrieval_time,
            "context_mode": context_mode,
            "timestamp": datetime.utcnow().isoformat()
        }
        self.logger.info(f"RAG_RETRIEVAL: {json.dumps(log_data)}")

    def log_error(self, error_type: str, error_message: str,
                  session_token: str = None, query_id: str = None,
                  additional_info: Dict[str, Any] = None):
        """
        Log an error with monitoring
        """
        log_data = {
            "event": "error",
            "error_type": error_type,
            "error_message": error_message,
            "session_token": session_token,
            "query_id": query_id,
            "additional_info": additional_info,
            "timestamp": datetime.utcnow().isoformat()
        }
        self.logger.error(f"ERROR: {json.dumps(log_data)}")

        # Also log to monitoring service
        self.monitoring_service.log_error(
            error_type=error_type,
            error_message=error_message,
            request_id=query_id,
            additional_info=additional_info
        )

    def log_conversation_start(self, session_token: str, user_id: str = None):
        """
        Log a conversation start
        """
        log_data = {
            "event": "conversation_start",
            "session_token": session_token,
            "user_id": user_id,
            "timestamp": datetime.utcnow().isoformat()
        }
        self.logger.info(f"CONVERSATION_START: {json.dumps(log_data)}")

    def log_llm_api_call(self, query_length: int, response_length: int,
                        api_call_time: float, response_style: str = None, provider: str = "unknown"):
        """
        Log LLM API call (works for both OpenAI and Gemini) with monitoring
        """
        log_data = {
            "event": "llm_api_call",
            "provider": provider,
            "query_length": query_length,
            "response_length": response_length,
            "api_call_time": api_call_time,
            "response_style": response_style,
            "timestamp": datetime.utcnow().isoformat()
        }
        self.logger.info(f"LLM_API: {json.dumps(log_data)}")

        # Also log to monitoring service for metrics
        self.monitoring_service.log_api_call(
            api_type=provider,
            duration=api_call_time,
            query_length=query_length,
            response_length=response_length,
            success=True
        )

    def log_gemini_api_call(self, query_length: int, response_length: int,
                           api_call_time: float, response_style: str = None):
        """
        Log Gemini API call
        """
        self.log_llm_api_call(query_length, response_length, api_call_time, response_style, "gemini")

    def log_openai_api_call(self, query_length: int, response_length: int,
                           api_call_time: float, response_style: str = None):
        """
        Log OpenAI API call
        """
        self.log_llm_api_call(query_length, response_length, api_call_time, response_style, "openai")


# Global logger instance
rag_logger = RAGChatbotLogger()