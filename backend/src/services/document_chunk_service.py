from typing import Optional, List
from sqlalchemy.orm import Session
from sqlalchemy import and_, or_
from uuid import UUID
from datetime import datetime
from ..models.document_chunk import DocumentChunk
from ..models.embedding_vector import EmbeddingVector


class DocumentChunkService:
    """
    Service class for managing document chunks and their embedding vectors
    """

    def create_document_chunk(self, db: Session, source_id: UUID, source_type: str, content: str,
                            chunk_order: int = 0, metadata: Optional[dict] = None) -> DocumentChunk:
        """
        Create a new document chunk
        """
        chunk = DocumentChunk(
            source_id=source_id,
            source_type=source_type,
            content=content,
            chunk_order=chunk_order,
            metadata_json=metadata
        )
        db.add(chunk)
        db.commit()
        db.refresh(chunk)
        return chunk

    def get_document_chunk_by_id(self, db: Session, chunk_id: UUID) -> Optional[DocumentChunk]:
        """
        Retrieve a document chunk by ID
        """
        return db.query(DocumentChunk).filter(DocumentChunk.id == chunk_id).first()

    def get_document_chunks_by_source(self, db: Session, source_id: UUID) -> List[DocumentChunk]:
        """
        Retrieve all document chunks for a specific source
        """
        return db.query(DocumentChunk)\
            .filter(DocumentChunk.source_id == source_id)\
            .order_by(DocumentChunk.chunk_order)\
            .all()

    def get_document_chunks_by_source_type(self, db: Session, source_type: str) -> List[DocumentChunk]:
        """
        Retrieve all document chunks of a specific type (book_content, user_text, etc.)
        """
        return db.query(DocumentChunk)\
            .filter(DocumentChunk.source_type == source_type)\
            .order_by(DocumentChunk.created_at)\
            .all()

    def update_document_chunk_content(self, db: Session, chunk_id: UUID, content: str) -> bool:
        """
        Update the content of an existing document chunk
        """
        chunk = db.query(DocumentChunk).filter(DocumentChunk.id == chunk_id).first()
        if chunk:
            chunk.content = content
            chunk.updated_at = datetime.utcnow()
            db.commit()
            return True
        return False

    def delete_document_chunk(self, db: Session, chunk_id: UUID) -> bool:
        """
        Delete a document chunk by ID
        """
        chunk = db.query(DocumentChunk).filter(DocumentChunk.id == chunk_id).first()
        if chunk:
            db.delete(chunk)
            db.commit()
            return True
        return False

    def delete_document_chunks_by_source(self, db: Session, source_id: UUID) -> int:
        """
        Delete all document chunks for a specific source
        Returns the number of deleted chunks
        """
        chunks = db.query(DocumentChunk).filter(DocumentChunk.source_id == source_id).all()
        count = 0
        for chunk in chunks:
            db.delete(chunk)
            count += 1
        db.commit()
        return count

    def create_embedding_vector(self, db: Session, chunk_id: UUID, vector_db_id: str,
                              model_used: str) -> EmbeddingVector:
        """
        Create a new embedding vector record
        """
        embedding = EmbeddingVector(
            chunk_id=chunk_id,
            vector_db_id=vector_db_id,
            model_used=model_used
        )
        db.add(embedding)
        db.commit()
        db.refresh(embedding)
        return embedding

    def get_embedding_vector_by_chunk_id(self, db: Session, chunk_id: UUID) -> Optional[EmbeddingVector]:
        """
        Retrieve an embedding vector by chunk ID
        """
        return db.query(EmbeddingVector).filter(EmbeddingVector.chunk_id == chunk_id).first()

    def get_embedding_vector_by_db_id(self, db: Session, vector_db_id: str) -> Optional[EmbeddingVector]:
        """
        Retrieve an embedding vector by database ID
        """
        return db.query(EmbeddingVector).filter(EmbeddingVector.vector_db_id == vector_db_id).first()

    def delete_embedding_vector(self, db: Session, vector_db_id: str) -> bool:
        """
        Delete an embedding vector by database ID
        """
        embedding = db.query(EmbeddingVector).filter(EmbeddingVector.vector_db_id == vector_db_id).first()
        if embedding:
            db.delete(embedding)
            db.commit()
            return True
        return False