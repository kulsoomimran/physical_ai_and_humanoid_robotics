"""
Unit tests for the RAG Agent implementation.
"""

import os
import unittest
from unittest.mock import Mock, patch, MagicMock
from backend.agent import RAGAgent, QdrantRetrievalTool, RetrievedChunk, ChatResponse


class TestQdrantRetrievalTool(unittest.TestCase):
    """Unit tests for QdrantRetrievalTool class."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        # Mock environment variables
        os.environ["COHERE_API_KEY"] = "test_key"
        os.environ["QDRANT_URL"] = "http://test-url"
        os.environ["QDRANT_API_KEY"] = "test_key"

    @patch('backend.agent.QdrantClient')
    @patch('backend.agent.cohere.Client')
    def test_initialization(self, mock_cohere_client, mock_qdrant_client):
        """Test initialization of QdrantRetrievalTool."""
        tool = QdrantRetrievalTool()

        self.assertIsNotNone(tool)
        self.assertEqual(tool.collection_name, "book_content")  # default
        self.assertEqual(tool.top_k, 5)  # default

    @patch('backend.agent.cohere.Client')
    @patch('backend.agent.QdrantClient')
    def test_generate_query_embedding(self, mock_qdrant_client, mock_cohere_client):
        """Test query embedding generation."""
        # Mock the Cohere client response
        mock_cohere_instance = Mock()
        mock_cohere_instance.embed.return_value = MagicMock(embeddings=[[0.1, 0.2, 0.3]])
        mock_cohere_client.return_value = mock_cohere_instance

        tool = QdrantRetrievalTool()
        result = tool.generate_query_embedding("test query")

        self.assertEqual(result, [0.1, 0.2, 0.3])
        mock_cohere_instance.embed.assert_called_once()

    @patch('backend.agent.cohere.Client')
    @patch('backend.agent.QdrantClient')
    def test_format_retrieved_content(self, mock_qdrant_client, mock_cohere_client):
        """Test formatting of retrieved content."""
        # Mock the Cohere client
        mock_cohere_instance = Mock()
        mock_cohere_client.return_value = mock_cohere_instance

        tool = QdrantRetrievalTool()

        # Create test chunks
        chunk1 = RetrievedChunk(
            id="1",
            text="This is a test chunk",
            score=0.8,
            vector_id="vec1",
            metadata={"source_url": "http://example.com"}
        )

        formatted = tool.format_retrieved_content([chunk1])

        self.assertIn("This is a test chunk", formatted)
        self.assertIn("Relevance: 0.80", formatted)
        self.assertIn("http://example.com", formatted)

    def test_format_retrieved_content_empty(self):
        """Test formatting of empty retrieved content."""
        # This test doesn't need Cohere or Qdrant clients
        tool = QdrantRetrievalTool(collection_name="test", top_k=5)

        formatted = tool.format_retrieved_content([])
        self.assertIn("No relevant content found", formatted)


class TestRAGAgent(unittest.TestCase):
    """Unit tests for RAGAgent class."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        os.environ["OPENAI_API_KEY"] = "test_key"
        os.environ["COHERE_API_KEY"] = "test_key"
        os.environ["QDRANT_URL"] = "http://test-url"
        os.environ["QDRANT_API_KEY"] = "test_key"

    @patch('backend.agent.OpenAI')
    @patch('backend.agent.cohere.Client')
    @patch('backend.agent.QdrantClient')
    def test_initialization(self, mock_qdrant_client, mock_cohere_client, mock_openai):
        """Test initialization of RAGAgent."""
        # Mock the OpenAI client
        mock_openai_instance = Mock()
        mock_assistant = Mock()
        mock_assistant.id = "test_assistant_id"
        mock_openai_instance.beta.assistants.create.return_value = mock_assistant
        mock_openai.return_value = mock_openai_instance

        # Mock the Cohere client
        mock_cohere_instance = Mock()
        mock_cohere_client.return_value = mock_cohere_instance

        agent = RAGAgent()

        self.assertIsNotNone(agent)
        self.assertIsNotNone(agent.client)
        self.assertIsNotNone(agent.retrieval_tool)
        self.assertIsNotNone(agent.assistant)

    @patch('backend.agent.OpenAI')
    @patch('backend.agent.cohere.Client')
    @patch('backend.agent.QdrantClient')
    def test_get_assistant_id(self, mock_qdrant_client, mock_cohere_client, mock_openai):
        """Test getting assistant ID."""
        # Mock the OpenAI client
        mock_openai_instance = Mock()
        mock_assistant = Mock()
        mock_assistant.id = "test_assistant_id_123"
        mock_openai_instance.beta.assistants.create.return_value = mock_assistant
        mock_openai.return_value = mock_openai_instance

        # Mock the Cohere client
        mock_cohere_instance = Mock()
        mock_cohere_client.return_value = mock_cohere_instance

        agent = RAGAgent()
        assistant_id = agent.get_assistant_id()

        self.assertEqual(assistant_id, "test_assistant_id_123")


class TestRetrievedChunk(unittest.TestCase):
    """Unit tests for RetrievedChunk data class."""

    def test_retrieved_chunk_creation(self):
        """Test creation of RetrievedChunk with valid data."""
        chunk = RetrievedChunk(
            id="1",
            text="Test content",
            score=0.8,
            vector_id="vec1",
            metadata={"source": "test"}
        )

        self.assertEqual(chunk.id, "1")
        self.assertEqual(chunk.text, "Test content")
        self.assertEqual(chunk.score, 0.8)
        self.assertEqual(chunk.vector_id, "vec1")
        self.assertEqual(chunk.metadata, {"source": "test"})

    def test_retrieved_chunk_validation_text(self):
        """Test validation of RetrievedChunk with empty text."""
        with self.assertRaises(ValueError):
            RetrievedChunk(
                id="1",
                text="",
                score=0.8,
                vector_id="vec1",
                metadata={"source": "test"}
            )

    def test_retrieved_chunk_validation_score(self):
        """Test validation of RetrievedChunk with invalid score."""
        with self.assertRaises(ValueError):
            RetrievedChunk(
                id="1",
                text="Test content",
                score=1.5,  # Invalid score > 1
                vector_id="vec1",
                metadata={"source": "test"}
            )

        with self.assertRaises(ValueError):
            RetrievedChunk(
                id="1",
                text="Test content",
                score=-0.5,  # Invalid score < 0
                vector_id="vec1",
                metadata={"source": "test"}
            )

    def test_retrieved_chunk_validation_metadata(self):
        """Test validation of RetrievedChunk with empty metadata."""
        with self.assertRaises(ValueError):
            RetrievedChunk(
                id="1",
                text="Test content",
                score=0.8,
                vector_id="vec1",
                metadata={}
            )


class TestChatResponse(unittest.TestCase):
    """Unit tests for ChatResponse data class."""

    def test_chat_response_creation(self):
        """Test creation of ChatResponse with valid data."""
        response = ChatResponse(
            response="Test response",
            thread_id="thread_123",
            status="completed"
        )

        self.assertEqual(response.response, "Test response")
        self.assertEqual(response.thread_id, "thread_123")
        self.assertEqual(response.status, "completed")

    def test_chat_response_validation_response(self):
        """Test validation of ChatResponse with empty response."""
        with self.assertRaises(ValueError):
            ChatResponse(
                response="",
                thread_id="thread_123",
                status="completed"
            )

    def test_chat_response_validation_status(self):
        """Test validation of ChatResponse with invalid status."""
        with self.assertRaises(ValueError):
            ChatResponse(
                response="Test response",
                thread_id="thread_123",
                status="invalid_status"
            )

    def test_chat_response_valid_statuses(self):
        """Test that valid statuses are accepted."""
        valid_statuses = ["completed", "error", "requires_action"]

        for status in valid_statuses:
            response = ChatResponse(
                response="Test response",
                thread_id="thread_123",
                status=status
            )
            self.assertEqual(response.status, status)


if __name__ == '__main__':
    unittest.main()