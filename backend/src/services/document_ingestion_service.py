import asyncio
import re
from typing import List, Dict, Any
from uuid import UUID
from sqlalchemy.orm import Session
from ..models.document_chunk import DocumentChunk
from ..services.rag_service import RAGService


class DocumentIngestionService:
    """
    Service class for ingesting documents and preparing them for RAG
    """

    def __init__(self, rag_service: RAGService):
        self.rag_service = rag_service

    def split_text_by_paragraphs(self, text: str) -> List[str]:
        """
        Split text into paragraphs
        """
        paragraphs = [p.strip() for p in text.split('\n\n') if p.strip()]
        return paragraphs

    def split_text_by_sentences(self, text: str) -> List[str]:
        """
        Split text into sentences
        """
        # Split by sentence endings
        sentences = re.split(r'[.!?]+\s+', text)
        sentences = [s.strip() for s in sentences if s.strip()]
        return sentences

    def semantic_chunk_text(self, text: str, max_chunk_size: int = 512) -> List[str]:
        """
        Split text into semantically coherent chunks
        """
        # First split by paragraphs
        paragraphs = self.split_text_by_paragraphs(text)

        chunks = []
        current_chunk = ""

        for paragraph in paragraphs:
            # If adding the paragraph would exceed the chunk size
            if len(current_chunk) + len(paragraph) > max_chunk_size and current_chunk:
                chunks.append(current_chunk.strip())
                current_chunk = paragraph
            else:
                current_chunk += "\n\n" + paragraph if current_chunk else paragraph

            # If the current chunk is still too large, try sentence-level splitting
            if len(current_chunk) > max_chunk_size:
                # Split the current chunk by sentences
                sentences = self.split_text_by_sentences(current_chunk)
                temp_chunk = ""

                for sentence in sentences:
                    if len(temp_chunk) + len(sentence) > max_chunk_size and temp_chunk:
                        chunks.append(temp_chunk.strip())
                        temp_chunk = sentence
                    else:
                        temp_chunk += " " + sentence if temp_chunk else sentence

                current_chunk = temp_chunk

        # Add the final chunk if it exists
        if current_chunk:
            chunks.append(current_chunk.strip())

        return [chunk for chunk in chunks if len(chunk) > 10]  # Filter out very small chunks

    async def ingest_book_content(self, db: Session, book_content: str,
                                book_id: UUID, book_title: str = None) -> List[DocumentChunk]:
        """
        Ingest book content and store it as document chunks
        """
        # Split the book content into chunks
        chunks = self.semantic_chunk_text(book_content)

        # Prepare chunks data for batch processing
        chunks_data = []
        for i, chunk_text in enumerate(chunks):
            metadata = {
                "source_title": book_title,
                "chunk_index": i,
                "total_chunks": len(chunks)
            }

            chunks_data.append({
                "content": chunk_text,
                "source_id": book_id,
                "source_type": "book_content",
                "chunk_order": i,
                "metadata": metadata
            })

        # Store all chunks in batch for better performance
        stored_chunks = self.rag_service.store_document_chunks_batch(db, chunks_data)

        return stored_chunks

    async def ingest_user_text(self, db: Session, user_text: str,
                             session_id: UUID, user_id: UUID = None) -> List[DocumentChunk]:
        """
        Ingest user-provided text and store it as document chunks
        """
        # For user text, we might use smaller chunks
        chunks = self.semantic_chunk_text(user_text, max_chunk_size=256)

        # Prepare chunks data for batch processing
        chunks_data = []
        for i, chunk_text in enumerate(chunks):
            metadata = {
                "source_type": "user_text",
                "chunk_index": i,
                "session_id": str(session_id)
            }

            if user_id:
                metadata["user_id"] = str(user_id)

            chunks_data.append({
                "content": chunk_text,
                "source_id": session_id,  # Using session ID as source for user text
                "source_type": "user_text",
                "chunk_order": i,
                "metadata": metadata
            })

        # Store all chunks in batch for better performance
        stored_chunks = self.rag_service.store_document_chunks_batch(db, chunks_data)

        return stored_chunks

    async def process_pdf_content(self, pdf_path: str) -> str:
        """
        Process a PDF file and extract text content
        """
        # This is a placeholder - in a real implementation, you'd use a library like PyPDF2 or pdfplumber
        # For now, we'll return an empty string
        # In practice, you'd extract text from the PDF file here
        with open(pdf_path, 'r', encoding='utf-8') as file:
            content = file.read()
        return content

    async def process_text_file(self, file_path: str) -> str:
        """
        Process a text file and extract content
        """
        with open(file_path, 'r', encoding='utf-8') as file:
            content = file.read()
        return content