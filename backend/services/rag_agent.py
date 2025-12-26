from typing import Dict, Any, Optional
from datetime import datetime
import logging
from ..agent import RAGAgent as OriginalRAGAgent, ChatResponse as AgentChatResponse


logger = logging.getLogger(__name__)


class RAGAgentService:
    """
    Service wrapper for the RAG agent to handle chat interactions
    """
    def __init__(self):
        """
        Initialize the RAG agent service
        """
        try:
            self.agent = OriginalRAGAgent()
            logger.info("RAG Agent service initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize RAG Agent service: {e}")
            raise

    async def chat(self, question: str, thread_id: Optional[str] = None, selected_text: Optional[str] = None) -> Dict[str, Any]:
        """
        Process a chat request through the RAG agent

        Args:
            question: The user's question
            thread_id: Optional thread ID for conversation context
            selected_text: Optional selected text to provide additional context

        Returns:
            Dictionary containing the response, thread_id, and metadata
        """
        try:
            # If selected_text is provided, we might want to include it in the question or context
            # For now, we'll just pass the question to the agent
            agent_response: AgentChatResponse = await self.agent.chat(question, thread_id)

            # Create response with timestamp
            response_data = {
                "response": agent_response.response,
                "thread_id": agent_response.thread_id,
                "timestamp": datetime.now().isoformat(),
                "metadata": {
                    "source": "rag_agent",
                    "selected_text_provided": selected_text is not None
                }
            }

            if selected_text:
                # Add selected text to metadata for potential use
                response_data["metadata"]["selected_text_length"] = len(selected_text)

            logger.info(f"Successfully processed chat request for thread: {agent_response.thread_id}")
            return response_data

        except Exception as e:
            logger.error(f"Error processing chat request: {e}")
            return {
                "response": f"An error occurred while processing your request: {str(e)}",
                "thread_id": thread_id or "",
                "timestamp": datetime.now().isoformat(),
                "metadata": {
                    "error": str(e),
                    "source": "rag_agent"
                }
            }