import pytest
from unittest.mock import Mock, MagicMock, patch, AsyncMock
from src.services.openai_agent_service import OpenAIAgentService
from typing import List, Dict, Any


class TestOpenAIAgentService:
    """
    Unit tests for OpenAIAgentService
    """

    def setup_method(self):
        """Set up test fixtures before each test method."""
        # Mock the OpenAI client
        with patch('src.services.openai_agent_service.OpenAI') as mock_openai:
            self.mock_client = Mock()
            mock_openai.return_value = self.mock_client

            # Mock the beta.assistants attribute
            self.mock_assistants = Mock()
            self.mock_client.beta.assistants = self.mock_assistants

            self.agent_service = OpenAIAgentService(
                api_key="test-key",
                use_gemini=False
            )

    @patch('src.services.openai_agent_service.asyncio.get_event_loop')
    def test_generate_response_standard(self, mock_loop):
        """Test generating a standard response."""
        # Mock the event loop and executor
        mock_executor = AsyncMock()
        mock_loop.return_value.run_in_executor = mock_executor

        # Mock the API response
        mock_response = Mock()
        mock_response.choices = [Mock()]
        mock_response.choices[0].message = Mock()
        mock_response.choices[0].message.content = "This is a test response."
        mock_executor.return_value = mock_response

        result = self.agent_service.generate_response("Test query", [])

        assert result == "This is a test response."
        mock_executor.assert_called_once()

    @patch('src.services.openai_agent_service.asyncio.get_event_loop')
    def test_generate_response_simplified(self, mock_loop):
        """Test generating a simplified response."""
        # Mock the event loop and executor
        mock_executor = AsyncMock()
        mock_loop.return_value.run_in_executor = mock_executor

        # Mock the API response
        mock_response = Mock()
        mock_response.choices = [Mock()]
        mock_response.choices[0].message = Mock()
        mock_response.choices[0].message.content = "Simplified response."
        mock_executor.return_value = mock_response

        result = self.agent_service.generate_response("Test query", [], response_style="simplified")

        assert result == "Simplified response."
        mock_executor.assert_called_once()

    @patch('src.services.openai_agent_service.asyncio.get_event_loop')
    def test_generate_response_detailed(self, mock_loop):
        """Test generating a detailed response."""
        # Mock the event loop and executor
        mock_executor = AsyncMock()
        mock_loop.return_value.run_in_executor = mock_executor

        # Mock the API response
        mock_response = Mock()
        mock_response.choices = [Mock()]
        mock_response.choices[0].message = Mock()
        mock_response.choices[0].message.content = "Detailed response with more information."
        mock_executor.return_value = mock_response

        result = self.agent_service.generate_response("Test query", [], response_style="detailed")

        assert result == "Detailed response with more information."
        mock_executor.assert_called_once()

    @patch('src.services.openai_agent_service.asyncio.get_event_loop')
    def test_generate_response_with_context(self, mock_loop):
        """Test generating a response with context."""
        # Mock the event loop and executor
        mock_executor = AsyncMock()
        mock_loop.return_value.run_in_executor = mock_executor

        # Mock the API response
        mock_response = Mock()
        mock_response.choices = [Mock()]
        mock_response.choices[0].message = Mock()
        mock_response.choices[0].message.content = "Response based on context."
        mock_executor.return_value = mock_response

        context = [
            {"content": "Context 1", "source": "source1"},
            {"content": "Context 2", "source": "source2"}
        ]

        result = self.agent_service.generate_response("Test query", context)

        assert result == "Response based on context."
        mock_executor.assert_called_once()

    @patch('src.services.openai_agent_service.asyncio.get_event_loop')
    def test_generate_context_aware_response(self, mock_loop):
        """Test generating a context-aware response."""
        # Mock the event loop and executor
        mock_executor = AsyncMock()
        mock_loop.return_value.run_in_executor = mock_executor

        # Mock the API response
        mock_response = Mock()
        mock_response.choices = [Mock()]
        mock_response.choices[0].message = Mock()
        mock_response.choices[0].message.content = "Context-aware response."
        mock_executor.return_value = mock_response

        conversation_history = [
            {"query": "Previous query", "response": "Previous response"}
        ]
        context = [{"content": "Context", "source": "source"}]

        result = self.agent_service.generate_context_aware_response(
            "Current query",
            conversation_history,
            context
        )

        assert result == "Context-aware response."
        mock_executor.assert_called_once()

    def test_is_using_gemini_false(self):
        """Test that is_using_gemini returns False when not using Gemini."""
        assert self.agent_service.is_using_gemini() is False

    @patch('src.services.openai_agent_service.OpenAI')
    def test_init_with_gemini(self, mock_openai):
        """Test initializing with Gemini API."""
        mock_client = Mock()
        mock_openai.return_value = mock_client

        gemini_service = OpenAIAgentService(
            api_key="test-key",
            use_gemini=True
        )

        assert gemini_service.is_using_gemini() is True
        assert gemini_service.model_name == "gemini-1.5-flash"  # Default value


class TestOpenAIAgentServiceWithGemini:
    """
    Unit tests for OpenAIAgentService when using Gemini
    """
    def setup_method(self):
        """Set up test fixtures before each test method."""
        # Mock the OpenAI client
        with patch('src.services.openai_agent_service.OpenAI') as mock_openai:
            self.mock_client = Mock()
            mock_openai.return_value = self.mock_client

            # Mock the beta.assistants attribute
            self.mock_assistants = Mock()
            self.mock_client.beta.assistants = self.mock_assistants

            self.agent_service = OpenAIAgentService(
                api_key="test-key",
                use_gemini=True
            )

    def test_is_using_gemini_true(self):
        """Test that is_using_gemini returns True when using Gemini."""
        assert self.agent_service.is_using_gemini() is True

    def test_model_name_gemini(self):
        """Test that the correct model name is used for Gemini."""
        assert self.agent_service.model_name == "gemini-1.5-flash"


if __name__ == "__main__":
    pytest.main()