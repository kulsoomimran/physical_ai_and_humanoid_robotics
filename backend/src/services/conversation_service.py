from typing import Optional, List
from sqlalchemy.orm import Session
from sqlalchemy import and_
from ..models.conversation import Conversation
from ..models.query import Query
from ..models.response import Response
from uuid import UUID


class ConversationService:
    """
    Service class for managing conversations, queries, and responses
    """

    def create_conversation(self, db: Session, session_token: str, user_id: Optional[UUID] = None) -> Conversation:
        """
        Create a new conversation
        """
        conversation = Conversation(
            session_token=session_token,
            user_id=user_id
        )
        db.add(conversation)
        db.commit()
        db.refresh(conversation)
        return conversation

    def get_conversation_by_session_token(self, db: Session, session_token: str) -> Optional[Conversation]:
        """
        Retrieve a conversation by session token
        """
        return db.query(Conversation).filter(Conversation.session_token == session_token).first()

    def get_conversation_by_id(self, db: Session, conversation_id: UUID) -> Optional[Conversation]:
        """
        Retrieve a conversation by ID
        """
        return db.query(Conversation).filter(Conversation.id == conversation_id).first()

    def add_query_to_conversation(self, db: Session, conversation_id: UUID, content: str,
                                query_type: str, user_preferences: Optional[dict] = None) -> Query:
        """
        Add a query to an existing conversation
        """
        query = Query(
            session_id=conversation_id,
            content=content,
            query_type=query_type,
            user_preferences_json=user_preferences
        )
        db.add(query)
        db.commit()
        db.refresh(query)
        return query

    def add_response_to_query(self, db: Session, query_id: UUID, content: str,
                            response_type: str, processing_time: float) -> Response:
        """
        Add a response to an existing query
        """
        response = Response(
            query_id=query_id,
            content=content,
            response_type=response_type,
            processing_time=str(processing_time)
        )
        db.add(response)
        db.commit()
        db.refresh(response)
        return response

    def get_recent_queries(self, db: Session, conversation_id: UUID, limit: int = 5) -> List[Query]:
        """
        Get recent queries from a conversation to maintain context
        """
        return db.query(Query)\
            .filter(Query.session_id == conversation_id)\
            .order_by(Query.created_at.desc())\
            .limit(limit)\
            .all()

    def end_conversation(self, db: Session, conversation_id: UUID) -> bool:
        """
        Mark a conversation as inactive
        """
        conversation = db.query(Conversation).filter(Conversation.id == conversation_id).first()
        if conversation:
            conversation.active = False
            db.commit()
            return True
        return False