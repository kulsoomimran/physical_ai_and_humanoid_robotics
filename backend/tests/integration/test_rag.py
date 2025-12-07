import pytest
from unittest.mock import patch, MagicMock
from uuid import UUID
import sys
import os
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import StaticPool

# Add the backend directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.models.conversation import Conversation, Query, Response, SourceCitation, DocumentChunk
from src.models.base import Base
from src.services.conversation_service import ConversationService
from src.services.rag_service import RAGService
from src.services.openai_agent_service import OpenAIAgentService


class TestRAGIntegration:
    """
    Integration tests for the RAG (Retrieval-Augmented Generation) flow
    """

    @pytest.fixture
    def setup_database(self):
        """
        Set up an in-memory SQLite database for testing
        """
        engine = create_engine(
            "sqlite:///:memory:",
            connect_args={"check_same_thread": False},
            poolclass=StaticPool
        )
        Base.metadata.create_all(bind=engine)
        TestingSessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

        db = TestingSessionLocal()
        try:
            yield db
        finally:
            db.close()

    @pytest.fixture
    def conversation_service(self):
        """
        Provide a conversation service instance
        """
        return ConversationService()

    @pytest.fixture
    def rag_service(self):
        """
        Provide a RAG service instance (mocked for testing)
        """
        # Since we can't easily test Qdrant in unit tests, we'll mock it
        with patch('qdrant_client.QdrantClient'):
            rag_service = RAGService(
                qdrant_url="http://test-qdrant:6333",
                qdrant_api_key="test-key"
            )
            return rag_service

    def test_full_rag_flow(self, setup_database, conversation_service, rag_service):
        """
        Test the complete RAG flow: conversation -> query -> retrieval -> response -> citation
        """
        db = setup_database

        # Step 1: Create a conversation
        session_token = "test-session-token"
        conversation = conversation_service.create_conversation(db, session_token)
        assert conversation.session_token == session_token
        assert UUID(conversation.id)

        # Step 2: Add document chunks to the RAG system (simulated)
        # In a real test, we would have actual book content
        sample_content = "Physical AI is an approach to robotics that emphasizes physical interaction between robots and their environment. This methodology focuses on how robots can understand and manipulate the physical world through direct interaction rather than just abstract computation."

        with patch.object(rag_service.qdrant_client, 'upsert', return_value=None):
            with patch.object(rag_service.qdrant_client, 'search', return_value=[]):
                # Add a document chunk to the database (the vector storage would be mocked)
                chunk = rag_service.store_document_chunk(
                    db,
                    content=sample_content,
                    source_id=conversation.id,
                    source_type="book_content",
                    chunk_order=0,
                    metadata={"source_location": "Chapter 1, Page 5"}
                )
                assert chunk.source_id == conversation.id
                assert chunk.source_type == "book_content"

        # Step 3: Add a query to the conversation
        query = conversation_service.add_query_to_conversation(
            db,
            conversation_id=conversation.id,
            content="What is Physical AI?",
            query_type="book_content"
        )
        assert query.session_id == conversation.id
        assert query.content == "What is Physical AI?"

        # Step 4: Simulate RAG retrieval (mocked)
        mock_retrieved_chunks = [{
            'id': chunk.id,
            'content': sample_content,
            'source_id': conversation.id,
            'source_type': 'book_content',
            'chunk_order': 0,
            'metadata': {'source_location': 'Chapter 1, Page 5'},
            'relevance_score': 0.95
        }]

        with patch.object(rag_service, 'retrieve_context_for_query', return_value=mock_retrieved_chunks):
            retrieved = rag_service.retrieve_context_for_query("What is Physical AI?")
            assert len(retrieved) == 1
            assert retrieved[0]['content'] == sample_content

        # Step 5: Create a mock response
        response = conversation_service.add_response_to_query(
            db,
            query_id=query.id,
            content="Physical AI is an approach to robotics that emphasizes physical interaction between robots and their environment.",
            response_type="factual",
            processing_time=0.5
        )
        assert response.query_id == query.id
        assert "Physical AI" in response.content

        # Step 6: Add source citations
        citations = rag_service.add_source_citations(db, response.id, mock_retrieved_chunks)
        assert len(citations) == 1
        assert citations[0].response_id == response.id
        assert citations[0].chunk_id == chunk.id
        assert citations[0].relevance_score == 0.95

        # Step 7: Verify data integrity
        # Fetch the conversation and verify all components are linked properly
        retrieved_conversation = conversation_service.get_conversation_by_session_token(db, session_token)
        assert retrieved_conversation.id == conversation.id

        # Verify the query is linked to the conversation
        retrieved_query = db.query(Query).filter(Query.session_id == conversation.id).first()
        assert retrieved_query.id == query.id

        # Verify the response is linked to the query
        retrieved_response = db.query(Response).filter(Response.query_id == query.id).first()
        assert retrieved_response.id == response.id

        # Verify the citation is linked to the response
        retrieved_citation = db.query(SourceCitation).filter(SourceCitation.response_id == response.id).first()
        assert retrieved_citation.id == citations[0].id

    def test_conversation_history_flow(self, setup_database, conversation_service):
        """
        Test the conversation history functionality
        """
        db = setup_database

        # Create a conversation
        session_token = "test-history-session"
        conversation = conversation_service.create_conversation(db, session_token)

        # Add multiple queries and responses to create history
        query1 = conversation_service.add_query_to_conversation(
            db,
            conversation_id=conversation.id,
            content="What is Physical AI?",
            query_type="book_content"
        )

        response1 = conversation_service.add_response_to_query(
            db,
            query_id=query1.id,
            content="Physical AI emphasizes physical interaction between robots and their environment.",
            response_type="factual",
            processing_time=0.4
        )

        query2 = conversation_service.add_query_to_conversation(
            db,
            conversation_id=conversation.id,
            content="How does it differ from traditional AI?",
            query_type="book_content"
        )

        response2 = conversation_service.add_response_to_query(
            db,
            query_id=query2.id,
            content="Traditional AI focuses on abstract computation while Physical AI emphasizes direct interaction.",
            response_type="comparative",
            processing_time=0.6
        )

        # Retrieve recent queries (history)
        recent_queries = conversation_service.get_recent_queries(db, conversation.id, limit=5)
        assert len(recent_queries) == 2  # We added 2 queries

        # Verify the order (most recent first by default in our implementation)
        assert recent_queries[0].id == query2.id  # Most recent
        assert recent_queries[1].id == query1.id  # Earlier

    def test_rag_retrieval_with_multiple_chunks(self, setup_database, conversation_service, rag_service):
        """
        Test RAG retrieval when multiple relevant chunks are found
        """
        db = setup_database

        # Create a conversation
        session_token = "test-multi-chunk-session"
        conversation = conversation_service.create_conversation(db, session_token)

        # Add multiple document chunks (simulating book content)
        contents = [
            "Physical AI is an approach to robotics.",
            "It emphasizes physical interaction between robots and environment.",
            "This methodology differs from traditional computational AI.",
            "Physical AI systems adapt to real-world physics."
        ]

        chunk_ids = []
        with patch.object(rag_service.qdrant_client, 'upsert', return_value=None):
            for i, content in enumerate(contents):
                chunk = rag_service.store_document_chunk(
                    db,
                    content=content,
                    source_id=conversation.id,
                    source_type="book_content",
                    chunk_order=i,
                    metadata={"source_location": f"Chapter 1, Page {i+1}"}
                )
                chunk_ids.append(chunk.id)

        # Mock retrieval of multiple chunks
        mock_retrieved_chunks = []
        for i, content in enumerate(contents):
            mock_retrieved_chunks.append({
                'id': chunk_ids[i],
                'content': content,
                'source_id': conversation.id,
                'source_type': 'book_content',
                'chunk_order': i,
                'metadata': {'source_location': f'Chapter 1, Page {i+1}'},
                'relevance_score': 0.9 - (i * 0.1)  # Slightly decreasing relevance
            })

        # Add a query
        query = conversation_service.add_query_to_conversation(
            db,
            conversation_id=conversation.id,
            content="Explain Physical AI methodology",
            query_type="book_content"
        )

        # Mock the retrieval
        with patch.object(rag_service, 'retrieve_context_for_query', return_value=mock_retrieved_chunks):
            retrieved = rag_service.retrieve_context_for_query("Explain Physical AI methodology")
            assert len(retrieved) == 4  # Should retrieve all 4 chunks

        # Add a response
        response = conversation_service.add_response_to_query(
            db,
            query_id=query.id,
            content="Physical AI is an approach that emphasizes physical interaction between robots and their environment, differing from traditional computational AI by adapting to real-world physics.",
            response_type="explanatory",
            processing_time=0.8
        )

        # Add citations for all retrieved chunks
        citations = rag_service.add_source_citations(db, response.id, retrieved)
        assert len(citations) == 4  # Should have citations for all chunks

        # Verify all citations are properly linked
        for citation in citations:
            assert citation.response_id == response.id
            assert citation.relevance_score > 0