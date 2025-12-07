import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch, MagicMock
from uuid import UUID
import sys
import os

# Add the backend directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from main import app  # The FastAPI app is in main.py at backend root

client = TestClient(app)


class TestChatContract:
    """
    Contract tests for the chat API endpoints
    """

    def test_chat_start_endpoint_contract(self):
        """
        Test the /chat/start endpoint contract
        """
        # Test valid request
        response = client.post("/chat/start", json={"user_id": "test-user-123"})

        # Assert status code
        assert response.status_code == 200

        # Assert response structure
        data = response.json()
        assert "session_token" in data
        assert "conversation_id" in data

        # Validate UUID format for conversation_id
        assert UUID(data["conversation_id"])  # This will raise an exception if not a valid UUID

        # Validate session_token is a string
        assert isinstance(data["session_token"], str)
        assert len(data["session_token"]) > 0

    def test_chat_start_endpoint_without_user_id(self):
        """
        Test the /chat/start endpoint without user_id
        """
        # Test request without user_id
        response = client.post("/chat/start", json={})

        # Assert status code
        assert response.status_code == 200

        # Assert response structure
        data = response.json()
        assert "session_token" in data
        assert "conversation_id" in data

        # Validate UUID format for conversation_id
        assert UUID(data["conversation_id"])

        # Validate session_token is a string
        assert isinstance(data["session_token"], str)
        assert len(data["session_token"]) > 0

    def test_chat_query_endpoint_contract(self):
        """
        Test the /chat/{session_token}/query endpoint contract
        """
        # First, create a session
        start_response = client.post("/chat/start", json={})
        assert start_response.status_code == 200
        session_data = start_response.json()
        session_token = session_data["session_token"]

        # Test valid query request
        query_request = {
            "session_token": session_token,
            "query": "What is Physical AI?",
            "context_mode": "book_content",
            "response_style": "detailed"
        }

        # Mock the OpenAI Agent response to avoid actual API calls during testing
        with patch('backend.src.services.openai_agent_service.OpenAIAgentService.create_thread_and_run_with_history') as mock_openai:
            mock_openai.return_value = "Physical AI is an approach to robotics that emphasizes the physical interaction between robots and their environment."

            response = client.post(f"/chat/{session_token}/query", json=query_request)

        # Assert status code
        assert response.status_code == 200

        # Assert response structure
        data = response.json()
        assert "response" in data
        assert "session_token" in data
        assert "query_id" in data
        assert "response_id" in data
        assert "source_citations" in data

        # Validate specific fields
        assert isinstance(data["response"], str)
        assert data["session_token"] == session_token
        assert UUID(data["query_id"])
        assert UUID(data["response_id"])
        assert isinstance(data["source_citations"], list)

    def test_chat_query_endpoint_with_different_context_modes(self):
        """
        Test the /chat/{session_token}/query endpoint with different context modes
        """
        # First, create a session
        start_response = client.post("/chat/start", json={})
        assert start_response.status_code == 200
        session_data = start_response.json()
        session_token = session_data["session_token"]

        context_modes = ["book_content", "user_text", "mixed"]

        for context_mode in context_modes:
            query_request = {
                "session_token": session_token,
                "query": "What is Physical AI?",
                "context_mode": context_mode
            }

            # Mock the OpenAI Agent response
            with patch('backend.src.services.openai_agent_service.OpenAIAgentService.create_thread_and_run_with_history') as mock_openai:
                mock_openai.return_value = f"Response for {context_mode} context."

                response = client.post(f"/chat/{session_token}/query", json=query_request)

            # Assert status code
            assert response.status_code == 200

            # Assert response structure
            data = response.json()
            assert "response" in data
            assert data["session_token"] == session_token

    def test_chat_query_endpoint_missing_session(self):
        """
        Test the /chat/{session_token}/query endpoint with missing session
        """
        fake_session_token = "non-existent-session-token"
        query_request = {
            "session_token": fake_session_token,
            "query": "What is Physical AI?"
        }

        response = client.post(f"/chat/{fake_session_token}/query", json=query_request)

        # Should return 404 for non-existent session
        assert response.status_code == 404
        assert "Conversation not found" in response.json()["detail"]

    def test_chat_history_endpoint_contract(self):
        """
        Test the /chat/{session_token}/history endpoint contract
        """
        # First, create a session
        start_response = client.post("/chat/start", json={})
        assert start_response.status_code == 200
        session_data = start_response.json()
        session_token = session_data["session_token"]

        # Request history for the session
        response = client.get(f"/chat/{session_token}/history")

        # Assert status code
        assert response.status_code == 200

        # Assert response structure
        data = response.json()
        assert "session_token" in data
        assert "history" in data
        assert data["session_token"] == session_token
        assert isinstance(data["history"], list)