"""
Tests for content compliance validation to ensure responses use only retrieved content.
"""

import os
import unittest
from unittest.mock import Mock, patch, MagicMock
from backend.agent import RAGAgent, QdrantRetrievalTool


class TestContentCompliance(unittest.TestCase):
    """Tests to ensure responses are based only on retrieved content."""

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
    def test_response_based_on_retrieved_content(self, mock_qdrant_client, mock_cohere_client, mock_openai):
        """Test that agent responses are based on retrieved content."""
        # Mock the OpenAI client and its components
        mock_openai_instance = Mock()
        mock_assistant = Mock()
        mock_assistant.id = "test_assistant_id"
        mock_openai_instance.beta.assistants.create.return_value = mock_assistant

        # Mock thread operations
        mock_thread = Mock()
        mock_thread.id = "test_thread_id"
        mock_openai_instance.beta.threads.create.return_value = mock_thread

        # Mock message operations - return a response that includes the retrieved content
        mock_message = Mock()
        mock_message.content = [Mock()]
        mock_message.content[0].text = Mock()
        # This response should include information from the retrieved content
        mock_message.content[0].text.value = "Based on the retrieved content, humanoid robotics involves artificial agents with human-like characteristics."
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

        # Mock Qdrant client with specific content
        mock_qdrant_instance = Mock()
        mock_search_result = Mock()
        mock_point = Mock()
        mock_point.id = "test_point_id"
        mock_point.score = 0.9
        # This is the content that should be referenced in the response
        mock_point.payload = {
            "content": "Humanoid robotics involves artificial agents with human-like characteristics.",
            "source_url": "http://example.com/humanoid-robotics"
        }
        mock_search_result.points = [mock_point]
        mock_qdrant_instance.query_points.return_value = mock_search_result
        mock_qdrant_client.return_value = mock_qdrant_instance

        # Initialize the agent
        agent = RAGAgent()

        # Test with a query related to humanoid robotics
        response = agent.chat("What is humanoid robotics?")

        # Verify that the response contains information from the retrieved content
        self.assertIn("humanoid", response.response.lower())
        self.assertIn("human-like", response.response.lower())
        self.assertIn("artificial agents", response.response.lower())

    @patch('backend.agent.OpenAI')
    @patch('backend.agent.cohere.Client')
    @patch('backend.agent.QdrantClient')
    def test_response_when_no_relevant_content_found(self, mock_qdrant_client, mock_cohere_client, mock_openai):
        """Test agent behavior when no relevant content is found."""
        # Mock the OpenAI client and its components
        mock_openai_instance = Mock()
        mock_assistant = Mock()
        mock_assistant.id = "test_assistant_id"
        # Set up assistant with instructions to only use retrieved content
        mock_openai_instance.beta.assistants.create.return_value = mock_assistant

        # Mock thread operations
        mock_thread = Mock()
        mock_thread.id = "test_thread_id"
        mock_openai_instance.beta.threads.create.return_value = mock_thread

        # Mock message operations - response should indicate no information available
        mock_message = Mock()
        mock_message.content = [Mock()]
        mock_message.content[0].text = Mock()
        mock_message.content[0].text.value = "The information is not available in the book."
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

        # Mock Qdrant client returning no results
        mock_qdrant_instance = Mock()
        mock_search_result = Mock()
        mock_search_result.points = []  # No results found
        mock_qdrant_instance.query_points.return_value = mock_search_result
        mock_qdrant_client.return_value = mock_qdrant_instance

        # Initialize the agent
        agent = RAGAgent()

        # Test with a query that won't find relevant content
        response = agent.chat("What is the meaning of life according to the robotics book?")

        # Verify the response indicates lack of information rather than making things up
        self.assertIn("not available", response.response.lower())
        self.assertIn("information", response.response.lower())

    @patch('backend.agent.OpenAI')
    @patch('backend.agent.cohere.Client')
    @patch('backend.agent.QdrantClient')
    def test_response_does_not_hallucinate_information(self, mock_qdrant_client, mock_cohere_client, mock_openai):
        """Test that agent does not hallucinate information not in retrieved content."""
        # Mock the OpenAI client and its components
        mock_openai_instance = Mock()
        mock_assistant = Mock()
        mock_assistant.id = "test_assistant_id"
        # Set specific instructions to avoid hallucination
        mock_openai_instance.beta.assistants.create.return_value = mock_assistant

        # Mock thread operations
        mock_thread = Mock()
        mock_thread.id = "test_thread_id"
        mock_openai_instance.beta.threads.create.return_value = mock_thread

        # Mock message operations
        mock_message = Mock()
        mock_message.content = [Mock()]
        mock_message.content[0].text = Mock()
        mock_message.content[0].text.value = "According to the retrieved content, the specific details are not mentioned."
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

        # Mock Qdrant client with content that doesn't answer the specific question
        mock_qdrant_instance = Mock()
        mock_search_result = Mock()
        mock_point = Mock()
        mock_point.id = "test_point_id"
        mock_point.score = 0.7  # Lower score indicating less relevant content
        mock_point.payload = {
            "content": "This section discusses general robotics concepts but not the specific question asked.",
            "source_url": "http://example.com/general-robotics"
        }
        mock_search_result.points = [mock_point]
        mock_qdrant_instance.query_points.return_value = mock_search_result
        mock_qdrant_client.return_value = mock_qdrant_instance

        # Initialize the agent
        agent = RAGAgent()

        # Test with a specific question that the retrieved content doesn't answer
        response = agent.chat("What are the exact torque specifications for joint motor J5?")

        # Verify the response doesn't fabricate information
        self.assertNotIn("42 nm", response.response.lower())  # Made up value
        self.assertNotIn("3.14159", response.response.lower())  # Made up value
        # Should acknowledge lack of specific information
        self.assertIn("according to", response.response.lower())
        self.assertIn("retrieved content", response.response.lower())


if __name__ == '__main__':
    unittest.main()