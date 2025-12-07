import pytest
from unittest.mock import Mock, MagicMock, patch
from src.services.rag_service import RAGService
from src.models.document_chunk import DocumentChunk
from uuid import UUID, uuid4
from typing import List, Dict, Any


class TestRAGService:
    """
    Unit tests for RAGService
    """

    def setup_method(self):
        """Set up test fixtures before each test method."""
        with patch('src.services.rag_service.QdrantClient') as mock_qdrant:
            self.mock_qdrant_client = Mock()
            mock_qdrant.return_value = self.mock_qdrant_client

            # Mock the get_collection method to not raise exception (simulating collection exists)
            self.mock_qdrant_client.get_collection = Mock()

            self.rag_service = RAGService(
                qdrant_url="http://test-url",
                qdrant_api_key="test-key"
            )

    def test_generate_embeddings(self):
        """Test that embeddings are generated correctly."""
        text = "This is a test sentence."

        # Call the method
        embedding = self.rag_service.generate_embeddings(text)

        # Check that the result is a list of floats
        assert isinstance(embedding, list)
        assert len(embedding) > 0
        assert all(isinstance(val, float) for val in embedding)

    def test_store_document_chunk(self):
        """Test storing a document chunk."""
        # Mock database session
        mock_db = Mock()
        mock_chunk = DocumentChunk(
            id=uuid4(),
            source_id=uuid4(),
            source_type="test",
            content="test content",
            chunk_order=0
        )
        mock_db.add = Mock()
        mock_db.commit = Mock()
        mock_db.refresh = Mock(return_value=mock_chunk)

        # Mock qdrant upsert
        self.mock_qdrant_client.upsert = Mock()

        source_id = uuid4()
        result = self.rag_service.store_document_chunk(
            mock_db,
            content="test content",
            source_id=source_id,
            source_type="book_content"
        )

        # Verify database operations
        mock_db.add.assert_called_once()
        mock_db.commit.assert_called_once()
        mock_db.refresh.assert_called_once()

        # Verify Qdrant operations
        self.mock_qdrant_client.upsert.assert_called_once()

    def test_retrieve_relevant_chunks(self):
        """Test retrieving relevant chunks."""
        query = "test query"

        # Mock search results
        mock_result = Mock()
        mock_result.id = str(uuid4())
        mock_result.payload = {
            "content": "test content",
            "source_id": str(uuid4()),
            "source_type": "book_content",
            "chunk_order": 0,
            "metadata": {}
        }
        mock_result.score = 0.8
        self.mock_qdrant_client.search = Mock(return_value=[mock_result])

        results = self.rag_service.retrieve_relevant_chunks(query)

        assert len(results) == 1
        assert results[0]['content'] == "test content"
        assert results[0]['relevance_score'] == 0.8

    def test_retrieve_context_for_query_book_content(self):
        """Test retrieving context for book content mode."""
        query = "test query"

        # Mock the retrieve_relevant_chunks method
        with patch.object(self.rag_service, 'retrieve_relevant_chunks', return_value=[{
            'id': uuid4(),
            'content': 'test content',
            'source_id': uuid4(),
            'source_type': 'book_content',
            'chunk_order': 0,
            'metadata': {},
            'relevance_score': 0.8
        }]):
            results = self.rag_service.retrieve_context_for_query(query, "book_content")

            assert len(results) == 1
            assert results[0]['content'] == "test content"

    def test_retrieve_context_for_query_user_text(self):
        """Test retrieving context for user text mode."""
        query = "test query"

        # Mock the retrieve_relevant_chunks method
        with patch.object(self.rag_service, 'retrieve_relevant_chunks', return_value=[{
            'id': uuid4(),
            'content': 'user text content',
            'source_id': uuid4(),
            'source_type': 'user_text',
            'chunk_order': 0,
            'metadata': {},
            'relevance_score': 0.8
        }]):
            results = self.rag_service.retrieve_context_for_query(query, "user_text")

            assert len(results) == 1
            assert results[0]['content'] == "user text content"

    def test_retrieve_context_for_query_mixed(self):
        """Test retrieving context for mixed mode."""
        query = "test query"

        # Mock the retrieve_relevant_chunks method for both calls
        book_results = [{
            'id': uuid4(),
            'content': 'book content',
            'source_id': uuid4(),
            'source_type': 'book_content',
            'chunk_order': 0,
            'metadata': {},
            'relevance_score': 0.8
        }]

        user_results = [{
            'id': uuid4(),
            'content': 'user content',
            'source_id': uuid4(),
            'source_type': 'user_text',
            'chunk_order': 0,
            'metadata': {},
            'relevance_score': 0.7
        }]

        with patch.object(self.rag_service, 'retrieve_relevant_chunks', side_effect=[book_results, user_results]):
            results = self.rag_service.retrieve_context_for_query(query, "mixed")

            assert len(results) == 2
            assert any(r['content'] == 'book content' for r in results)
            assert any(r['content'] == 'user content' for r in results)

    def test_store_document_chunks_batch(self):
        """Test storing document chunks in batch."""
        # Mock database session
        mock_db = Mock()
        mock_db.add = Mock()
        mock_db.commit = Mock()
        mock_db.refresh = Mock()

        # Mock qdrant upsert
        self.mock_qdrant_client.upsert = Mock()

        # Prepare test data
        chunks_data = [
            {
                'content': 'test content 1',
                'source_id': uuid4(),
                'source_type': 'book_content',
                'chunk_order': 0,
                'metadata': {}
            },
            {
                'content': 'test content 2',
                'source_id': uuid4(),
                'source_type': 'book_content',
                'chunk_order': 1,
                'metadata': {}
            }
        ]

        results = self.rag_service.store_document_chunks_batch(mock_db, chunks_data)

        # Verify database operations
        assert mock_db.add.call_count == len(chunks_data)
        mock_db.commit.assert_called_once()
        assert mock_db.refresh.call_count == len(chunks_data)

        # Verify Qdrant operations
        self.mock_qdrant_client.upsert.assert_called_once()


class TestDocumentIngestionService:
    """
    Unit tests for DocumentIngestionService
    """
    pass  # We'll implement these tests in a separate file


if __name__ == "__main__":
    pytest.main()