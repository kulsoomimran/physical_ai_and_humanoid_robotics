import pytest
from unittest.mock import Mock, MagicMock, patch
from src.services.conversation_service import ConversationService
from src.models.conversation import Conversation
from src.models.query import Query
from src.models.response import Response
from uuid import UUID, uuid4


class TestConversationService:
    """
    Unit tests for ConversationService
    """

    def setup_method(self):
        """Set up test fixtures before each test method."""
        self.conversation_service = ConversationService()

    def test_create_conversation(self):
        """Test creating a new conversation."""
        # Mock database session
        mock_db = Mock()
        mock_conversation = Mock()
        mock_conversation.id = uuid4()
        mock_conversation.session_token = "test-token"
        mock_conversation.user_id = None
        mock_db.add = Mock()
        mock_db.commit = Mock()
        mock_db.refresh = Mock(return_value=mock_conversation)

        session_token = "test-session-token"
        result = self.conversation_service.create_conversation(mock_db, session_token)

        # Verify database operations
        mock_db.add.assert_called_once()
        mock_db.commit.assert_called_once()
        mock_db.refresh.assert_called_once()

        # Check that the result is a conversation object
        assert hasattr(result, 'session_token')
        assert result.session_token == session_token

    def test_get_conversation_by_session_token(self):
        """Test retrieving a conversation by session token."""
        # Mock database session
        mock_db = Mock()
        mock_query = Mock()
        mock_filter = Mock()
        mock_first = Mock()

        mock_db.query = Mock(return_value=mock_query)
        mock_query.filter = Mock(return_value=mock_filter)
        mock_filter.first = Mock(return_value=mock_first)

        session_token = "test-session-token"
        result = self.conversation_service.get_conversation_by_session_token(mock_db, session_token)

        # Verify query operations
        mock_db.query.assert_called_once_with(Conversation)
        mock_query.filter.assert_called_once()
        mock_filter.first.assert_called_once()

        assert result == mock_first

    def test_get_conversation_by_id(self):
        """Test retrieving a conversation by ID."""
        # Mock database session
        mock_db = Mock()
        mock_query = Mock()
        mock_filter = Mock()
        mock_first = Mock()

        mock_db.query = Mock(return_value=mock_query)
        mock_query.filter = Mock(return_value=mock_filter)
        mock_filter.first = Mock(return_value=mock_first)

        conversation_id = uuid4()
        result = self.conversation_service.get_conversation_by_id(mock_db, conversation_id)

        # Verify query operations
        mock_db.query.assert_called_once_with(Conversation)
        mock_query.filter.assert_called_once()
        mock_filter.first.assert_called_once()

        assert result == mock_first

    def test_add_query_to_conversation(self):
        """Test adding a query to a conversation."""
        # Mock database session
        mock_db = Mock()
        mock_query = Mock()
        mock_query.id = uuid4()
        mock_query.session_id = uuid4()
        mock_query.content = "test query"
        mock_query.query_type = "test"
        mock_db.add = Mock()
        mock_db.commit = Mock()
        mock_db.refresh = Mock(return_value=mock_query)

        conversation_id = uuid4()
        content = "test query content"
        query_type = "test_type"

        result = self.conversation_service.add_query_to_conversation(
            mock_db,
            conversation_id,
            content,
            query_type
        )

        # Verify database operations
        mock_db.add.assert_called_once()
        mock_db.commit.assert_called_once()
        mock_db.refresh.assert_called_once()

        assert result.content == content
        assert result.query_type == query_type

    def test_add_response_to_query(self):
        """Test adding a response to a query."""
        # Mock database session
        mock_db = Mock()
        mock_response = Mock()
        mock_response.id = uuid4()
        mock_response.query_id = uuid4()
        mock_response.content = "test response"
        mock_response.response_type = "test"
        mock_db.add = Mock()
        mock_db.commit = Mock()
        mock_db.refresh = Mock(return_value=mock_response)

        query_id = uuid4()
        content = "test response content"
        response_type = "test_type"
        processing_time = 0.5

        result = self.conversation_service.add_response_to_query(
            mock_db,
            query_id,
            content,
            response_type,
            processing_time
        )

        # Verify database operations
        mock_db.add.assert_called_once()
        mock_db.commit.assert_called_once()
        mock_db.refresh.assert_called_once()

        assert result.content == content
        assert result.response_type == response_type

    def test_get_recent_queries(self):
        """Test getting recent queries from a conversation."""
        # Mock database session
        mock_db = Mock()
        mock_query = Mock()
        mock_filter = Mock()
        mock_order_by = Mock()
        mock_limit = Mock()
        mock_all = Mock(return_value=[Mock(), Mock()])

        mock_db.query = Mock(return_value=mock_query)
        mock_query.filter = Mock(return_value=mock_filter)
        mock_filter.order_by = Mock(return_value=mock_order_by)
        mock_order_by.limit = Mock(return_value=mock_limit)
        mock_limit.all = Mock(return_value=mock_all)

        conversation_id = uuid4()
        result = self.conversation_service.get_recent_queries(mock_db, conversation_id, limit=5)

        # Verify query operations
        mock_db.query.assert_called_once_with(Query)
        mock_query.filter.assert_called_once()
        mock_filter.order_by.assert_called_once()
        mock_order_by.limit.assert_called_once_with(5)
        mock_limit.all.assert_called_once()

        assert isinstance(result, list)

    def test_end_conversation_success(self):
        """Test ending a conversation successfully."""
        # Mock database session
        mock_db = Mock()
        mock_conversation = Mock()
        mock_conversation.active = True
        mock_query = Mock()
        mock_filter = Mock()

        mock_db.query = Mock(return_value=mock_query)
        mock_query.filter = Mock(return_value=mock_filter)
        mock_filter.first = Mock(return_value=mock_conversation)

        conversation_id = uuid4()
        result = self.conversation_service.end_conversation(mock_db, conversation_id)

        # Verify database operations
        mock_db.query.assert_called_once_with(Conversation)
        mock_query.filter.assert_called_once()
        mock_filter.first.assert_called_once()
        mock_db.commit.assert_called_once()

        assert result is True
        assert mock_conversation.active is False

    def test_end_conversation_not_found(self):
        """Test ending a conversation that doesn't exist."""
        # Mock database session
        mock_db = Mock()
        mock_query = Mock()
        mock_filter = Mock()

        mock_db.query = Mock(return_value=mock_query)
        mock_query.filter = Mock(return_value=mock_filter)
        mock_filter.first = Mock(return_value=None)

        conversation_id = uuid4()
        result = self.conversation_service.end_conversation(mock_db, conversation_id)

        # Verify database operations
        mock_db.query.assert_called_once_with(Conversation)
        mock_query.filter.assert_called_once()
        mock_filter.first.assert_called_once()

        assert result is False


if __name__ == "__main__":
    pytest.main()