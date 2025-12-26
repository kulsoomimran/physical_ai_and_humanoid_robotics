"""
Integration tests for the RAG Agent implementation.
Tests the end-to-end workflow of the agent.
"""

import os
import unittest
from unittest.mock import Mock, patch, MagicMock
from backend.agent import RAGAgent, QdrantRetrievalTool


class TestRAGAgentIntegration(unittest.TestCase):
    """Integration tests for RAG Agent end-to-end workflow."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        # Set up required environment variables
        os.environ["OPENAI_API_KEY"] = "test_openai_key"
        os.environ["COHERE_API_KEY"] = "test_cohere_key"
        os.environ["QDRANT_URL"] = "http://test-qdrant-url"
        os.environ["QDRANT_API_KEY"] = "test_qdrant_key"

    @patch('backend.agent.OpenAI')
    @patch('backend.agent.cohere.Client')
    @patch('backend.agent.QdrantClient')
    def test_agent_initialization_and_basic_query(self, mock_qdrant_client, mock_cohere_client, mock_openai):
        """Test full initialization and basic query flow."""
        # Mock the OpenAI client and its components
        mock_openai_instance = Mock()
        mock_assistant = Mock()
        mock_assistant.id = "test_assistant_id"
        mock_openai_instance.beta.assistants.create.return_value = mock_assistant

        # Mock thread operations
        mock_thread = Mock()
        mock_thread.id = "test_thread_id"
        mock_openai_instance.beta.threads.create.return_value = mock_thread

        # Mock message operations
        mock_message = Mock()
        mock_message.content = [Mock()]
        mock_message.content[0].text = Mock()
        mock_message.content[0].text.value = "Test response from agent"
        mock_messages_list = Mock()
        mock_messages_list.data = [mock_message]
        mock_openai_instance.beta.threads.messages.list.return_value = mock_messages_list

        # Mock run operations
        mock_run = Mock()
        mock_run.status = "completed"
        mock_openai_instance.beta.threads.runs.create.return_value = mock_run
        mock_openai_instance.beta.threads.runs.retrieve.return_value = mock_run

        mock_openai.return_value = mock_openai_instance

        # Mock Cohere client
        mock_cohere_instance = Mock()
        mock_cohere_instance.embed.return_value = MagicMock(embeddings=[[0.1, 0.2, 0.3]])
        mock_cohere_client.return_value = mock_cohere_instance

        # Mock Qdrant client
        mock_qdrant_instance = Mock()
        mock_search_result = Mock()
        mock_point = Mock()
        mock_point.id = "test_point_id"
        mock_point.score = 0.9
        mock_point.payload = {"content": "Test content from Qdrant", "source_url": "http://example.com"}
        mock_search_result.points = [mock_point]
        mock_qdrant_instance.query_points.return_value = mock_search_result
        mock_qdrant_client.return_value = mock_qdrant_instance

        # Initialize the agent
        agent = RAGAgent()

        # Test the chat method
        response = agent.chat("Test query")

        # Verify the response
        self.assertIsNotNone(response)
        self.assertEqual(response.status, "completed")
        self.assertIn("Test response from agent", response.response)
        self.assertEqual(response.thread_id, "test_thread_id")

        # Verify that the necessary calls were made
        mock_openai_instance.beta.assistants.create.assert_called()
        mock_openai_instance.beta.threads.create.assert_called()
        mock_openai_instance.beta.threads.messages.create.assert_called()
        mock_openai_instance.beta.threads.runs.create.assert_called()

    @patch('backend.agent.OpenAI')
    @patch('backend.agent.cohere.Client')
    @patch('backend.agent.QdrantClient')
    def test_agent_with_tool_call(self, mock_qdrant_client, mock_cohere_client, mock_openai):
        """Test agent workflow with tool call execution."""
        # Mock the OpenAI client and its components
        mock_openai_instance = Mock()
        mock_assistant = Mock()
        mock_assistant.id = "test_assistant_id"
        mock_openai_instance.beta.assistants.create.return_value = mock_assistant

        # Mock thread operations
        mock_thread = Mock()
        mock_thread.id = "test_thread_id"
        mock_openai_instance.beta.threads.create.return_value = mock_thread

        # Mock message operations
        mock_message = Mock()
        mock_message.content = [Mock()]
        mock_message.content[0].text = Mock()
        mock_message.content[0].text.value = "Response using retrieved content"
        mock_messages_list = Mock()
        mock_messages_list.data = [mock_message]
        mock_openai_instance.beta.threads.messages.list.return_value = mock_messages_list

        # Mock run operations for tool call scenario
        mock_run_incomplete = Mock()
        mock_run_incomplete.status = "requires_action"
        mock_run_incomplete.required_action = Mock()
        mock_required_action = Mock()
        mock_tool_call = Mock()
        mock_tool_call.id = "tool_call_123"
        mock_tool_call.function = Mock()
        mock_tool_call.function.name = "search_book_content"
        mock_tool_call.function.arguments = '{"query": "test query", "top_k": 5}'
        mock_required_action.submit_tool_outputs = Mock()
        mock_required_action.submit_tool_outputs.tool_calls = [mock_tool_call]
        mock_run_incomplete.required_action = mock_required_action

        mock_run_completed = Mock()
        mock_run_completed.status = "completed"

        # Mock run retrieval to return incomplete first, then completed
        def run_retrieve_side_effect(thread_id, run_id):
            if run_id == "first_run":
                return mock_run_incomplete
            else:
                return mock_run_completed

        mock_openai_instance.beta.threads.runs.retrieve.side_effect = run_retrieve_side_effect

        # Mock initial run creation
        mock_initial_run = Mock()
        mock_initial_run.id = "first_run"
        mock_initial_run.status = "requires_action"
        mock_openai_instance.beta.threads.runs.create.return_value = mock_initial_run

        # Mock submit tool outputs
        mock_submit_run = Mock()
        mock_submit_run.id = "submit_run"
        mock_submit_run.status = "completed"
        mock_openai_instance.beta.threads.runs.submit_tool_outputs.return_value = mock_submit_run

        mock_openai.return_value = mock_openai_instance

        # Mock Cohere client
        mock_cohere_instance = Mock()
        mock_cohere_instance.embed.return_value = MagicMock(embeddings=[[0.1, 0.2, 0.3]])
        mock_cohere_client.return_value = mock_cohere_instance

        # Mock Qdrant client
        mock_qdrant_instance = Mock()
        mock_search_result = Mock()
        mock_point = Mock()
        mock_point.id = "test_point_id"
        mock_point.score = 0.9
        mock_point.payload = {"content": "Test content from Qdrant for tool call", "source_url": "http://example.com"}
        mock_search_result.points = [mock_point]
        mock_qdrant_instance.query_points.return_value = mock_search_result
        mock_qdrant_client.return_value = mock_qdrant_instance

        # Initialize the agent
        agent = RAGAgent()

        # Test the chat method with tool call
        response = agent.chat("Test query requiring tool call")

        # Verify the response
        self.assertIsNotNone(response)
        self.assertEqual(response.status, "completed")
        self.assertIn("Response using retrieved content", response.response)

    @patch('backend.agent.OpenAI')
    @patch('backend.agent.cohere.Client')
    @patch('backend.agent.QdrantClient')
    def test_agent_conversation_thread_persistence(self, mock_qdrant_client, mock_cohere_client, mock_openai):
        """Test conversation thread persistence across multiple queries."""
        # Mock the OpenAI client and its components
        mock_openai_instance = Mock()
        mock_assistant = Mock()
        mock_assistant.id = "test_assistant_id"
        mock_openai_instance.beta.assistants.create.return_value = mock_assistant

        # Mock thread operations
        mock_thread = Mock()
        mock_thread.id = "persistent_thread_id"
        mock_openai_instance.beta.threads.retrieve.return_value = mock_thread

        # Mock message operations
        mock_message = Mock()
        mock_message.content = [Mock()]
        mock_message.content[0].text = Mock()
        mock_message.content[0].text.value = "Response with context from previous query"
        mock_messages_list = Mock()
        mock_messages_list.data = [mock_message]
        mock_openai_instance.beta.threads.messages.list.return_value = mock_messages_list

        # Mock run operations
        mock_run = Mock()
        mock_run.status = "completed"
        mock_openai_instance.beta.threads.runs.create.return_value = mock_run
        mock_openai_instance.beta.threads.runs.retrieve.return_value = mock_run

        mock_openai.return_value = mock_openai_instance

        # Mock Cohere client
        mock_cohere_instance = Mock()
        mock_cohere_instance.embed.return_value = MagicMock(embeddings=[[0.1, 0.2, 0.3]])
        mock_cohere_client.return_value = mock_cohere_instance

        # Mock Qdrant client
        mock_qdrant_instance = Mock()
        mock_search_result = Mock()
        mock_point = Mock()
        mock_point.id = "test_point_id"
        mock_point.score = 0.9
        mock_point.payload = {"content": "Test content for follow-up", "source_url": "http://example.com"}
        mock_search_result.points = [mock_point]
        mock_qdrant_instance.query_points.return_value = mock_search_result
        mock_qdrant_client.return_value = mock_qdrant_instance

        # Initialize the agent
        agent = RAGAgent()

        # First query
        first_response = agent.chat("First query", thread_id="persistent_thread_id")

        # Second query using the same thread
        second_response = agent.chat("Follow-up query", thread_id="persistent_thread_id")

        # Verify both responses
        self.assertIsNotNone(first_response)
        self.assertIsNotNone(second_response)
        self.assertEqual(first_response.thread_id, "persistent_thread_id")
        self.assertEqual(second_response.thread_id, "persistent_thread_id")

        # Verify that thread retrieval was called for the second query
        mock_openai_instance.beta.threads.retrieve.assert_called_with("persistent_thread_id")


if __name__ == '__main__':
    unittest.main()