import asyncio
from typing import List, Optional, Dict, Any
from uuid import UUID
import logging
from sentence_transformers import SentenceTransformer
from qdrant_client import QdrantClient
from qdrant_client.http import models
from ..models.document_chunk import DocumentChunk
from ..models.source_citation import SourceCitation
from sqlalchemy.orm import Session


class RAGService:
    """
    Service class for Retrieval-Augmented Generation functionality
    Handles document retrieval, embedding generation, and similarity search
    """

    def __init__(self, qdrant_url: str, qdrant_api_key: str, collection_name: str = "book_content", user_collection_name: str = "user_content"):
        # Ensure URL is properly formatted (QdrantClient expects URL without path)
        if qdrant_url and qdrant_url.endswith(':6333'):
            # Remove trailing slash if present
            qdrant_url = qdrant_url.rstrip('/')

        self.qdrant_client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key,
        )
        self.collection_name = collection_name
        self.user_collection_name = user_collection_name
        self.encoder = SentenceTransformer('all-MiniLM-L6-v2')  # Using a lightweight model initially

        # Ensure the collections exist
        self._ensure_collection_exists()
        self._ensure_user_collection_exists()

    def _ensure_collection_exists(self):
        """
        Ensure the main Qdrant collection exists with proper configuration
        """
        try:
            self.qdrant_client.get_collection(self.collection_name)
        except:
            # Create collection if it doesn't exist
            self.qdrant_client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=384,  # Size of the all-MiniLM-L6-v2 embeddings
                    distance=models.Distance.COSINE
                )
            )

    def _ensure_user_collection_exists(self):
        """
        Ensure the user content Qdrant collection exists with proper configuration
        """
        try:
            self.qdrant_client.get_collection(self.user_collection_name)
        except:
            # Create user collection if it doesn't exist
            self.qdrant_client.create_collection(
                collection_name=self.user_collection_name,
                vectors_config=models.VectorParams(
                    size=384,  # Size of the all-MiniLM-L6-v2 embeddings
                    distance=models.Distance.COSINE
                )
            )

    def generate_embeddings(self, text: str) -> List[float]:
        """
        Generate embeddings for a given text
        """
        embedding = self.encoder.encode(text)
        return embedding.tolist()

    def store_document_chunk(self, db: Session, content: str, source_id: UUID,
                           source_type: str, chunk_order: int = 0,
                           metadata: Optional[Dict] = None) -> DocumentChunk:
        """
        Store a document chunk in the database and vector store
        """
        # Create the document chunk in the database
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

        # Generate embedding
        embedding = self.generate_embeddings(content)

        # Determine which collection to use based on source type
        collection_name = self.user_collection_name if source_type == "user_text" else self.collection_name

        # Store in Qdrant
        self.qdrant_client.upsert(
            collection_name=collection_name,
            points=[
                models.PointStruct(
                    id=str(chunk.id),
                    vector=embedding,
                    payload={
                        "content": content,
                        "source_id": str(source_id),
                        "source_type": source_type,
                        "chunk_order": chunk_order,
                        "metadata": metadata or {}
                    }
                )
            ]
        )

        return chunk

    def store_document_chunks_batch(self, db: Session, chunks_data: List[Dict[str, Any]]) -> List[DocumentChunk]:
        """
        Store multiple document chunks in batch for better performance
        """
        stored_chunks = []
        qdrant_points = []

        # First, store all chunks in the database
        for chunk_data in chunks_data:
            chunk = DocumentChunk(
                source_id=chunk_data['source_id'],
                source_type=chunk_data['source_type'],
                content=chunk_data['content'],
                chunk_order=chunk_data.get('chunk_order', 0),
                metadata_json=chunk_data.get('metadata', {})
            )
            db.add(chunk)
            stored_chunks.append(chunk)

        # Commit all database changes
        db.commit()

        # Refresh to get the IDs after commit
        for chunk in stored_chunks:
            db.refresh(chunk)

        # Prepare all points for Qdrant
        for i, chunk in enumerate(stored_chunks):
            embedding = self.generate_embeddings(chunk.content)

            # Determine which collection to use based on source type
            collection_name = self.user_collection_name if chunk.source_type == "user_text" else self.collection_name

            qdrant_points.append(
                models.PointStruct(
                    id=str(chunk.id),
                    vector=embedding,
                    payload={
                        "content": chunk.content,
                        "source_id": str(chunk.source_id),
                        "source_type": chunk.source_type,
                        "chunk_order": chunk.chunk_order,
                        "metadata": chunk.metadata_json or {}
                    }
                )
            )

        # Store all points in Qdrant in batch by collection
        book_points = [point for point in qdrant_points if point.payload["source_type"] != "user_text"]
        user_points = [point for point in qdrant_points if point.payload["source_type"] == "user_text"]

        if book_points:
            self.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=book_points
            )

        if user_points:
            self.qdrant_client.upsert(
                collection_name=self.user_collection_name,
                points=user_points
            )

        return stored_chunks

    def retrieve_relevant_chunks(self, query: str, collection_name: str = None, limit: int = 5) -> List[Dict[str, Any]]:
        """
        Retrieve the most relevant document chunks for a given query
        """
        if collection_name is None:
            collection_name = self.collection_name

        query_embedding = self.generate_embeddings(query)

        search_results = self.qdrant_client.search(
            collection_name=collection_name,
            query_vector=query_embedding,
            limit=limit
        )

        relevant_chunks = []
        for result in search_results:
            relevant_chunks.append({
                'id': UUID(result.id),
                'content': result.payload['content'],
                'source_id': UUID(result.payload['source_id']),
                'source_type': result.payload['source_type'],
                'chunk_order': result.payload['chunk_order'],
                'metadata': result.payload.get('metadata', {}),
                'relevance_score': result.score
            })

        return relevant_chunks

    async def retrieve_context_for_query(self, user_query: str, context_mode: str = "book_content") -> List[Dict[str, Any]]:
        """
        Retrieve context for a user query based on the context mode
        """
        if context_mode == "book_content":
            # Search in book content
            return self.retrieve_relevant_chunks(user_query, self.collection_name)
        elif context_mode == "user_text":
            # Search in user-provided text
            return self.retrieve_relevant_chunks(user_query, self.user_collection_name)
        elif context_mode == "mixed":
            # Mixed mode - search both
            book_results = self.retrieve_relevant_chunks(user_query, self.collection_name, limit=3)
            user_results = self.retrieve_relevant_chunks(user_query, self.user_collection_name, limit=2)
            return book_results + user_results
        else:
            # Default to book content if context mode is unknown
            return self.retrieve_relevant_chunks(user_query, self.collection_name)

    def add_source_citations(self, db: Session, response_id: UUID,
                           retrieved_chunks: List[Dict[str, Any]]) -> List[SourceCitation]:
        """
        Add source citations for the retrieved chunks to the response
        """
        citations = []
        for chunk_data in retrieved_chunks:
            citation = SourceCitation(
                response_id=response_id,
                chunk_id=chunk_data['id'],
                relevance_score=chunk_data['relevance_score'],
                citation_text=chunk_data['content'],
                source_location=chunk_data['metadata'].get('source_location', 'Unknown')
            )
            db.add(citation)
            citations.append(citation)

        db.commit()
        return citations

    def delete_document_chunks(self, source_id: UUID, source_type: str = None):
        """
        Delete all document chunks associated with a source ID
        """
        # In Qdrant, we need to delete points by their IDs
        # This would require querying the database first to get all chunk IDs for the source
        # Then delete them from the appropriate Qdrant collection
        logging.info(f"Deleting document chunks for source_id: {source_id}, source_type: {source_type}")

        # For now, we'll just log this operation
        # In a full implementation, we would:
        # 1. Query the database for all chunks with this source_id
        # 2. Delete those points from the appropriate Qdrant collection
        # 3. Delete the database records
        pass

    def extract_examples_from_content(self, content: str, query: str) -> List[str]:
        """
        Extract relevant examples from content based on the query
        """
        import re

        examples = []

        # Look for common example indicators in the content
        example_patterns = [
            r'example[:\s\d]*[.:]?\s*([^.!?]*[.!?])',  # "Example: ..." or "Example 1: ..."
            r'for instance[:\s,]*\s*([^.!?]*[.!?])',     # "For instance ..."
            r'as an example[:\s,]*\s*([^.!?]*[.!?])',    # "As an example ..."
            r'illustration[:\s,]*[.:]?\s*([^.!?]*[.!?])', # "Illustration: ..."
            r'case study[:\s,]*[.:]?\s*([^.!?]*[.!?])',   # "Case study: ..."
            r'for example[:\s,]*\s*([^.!?]*[.!?])',      # "For example ..."
            r'such as[:\s,]*\s*([^.!?]*[.!?])',          # "Such as ..."
        ]

        for pattern in example_patterns:
            matches = re.findall(pattern, content, re.IGNORECASE)
            examples.extend([match.strip() for match in matches if match.strip()])

        # Look for code blocks or specific formatting that might contain examples
        code_block_pattern = r'```[\s\S]*?```'
        code_blocks = re.findall(code_block_pattern, content)
        for block in code_blocks:
            # Extract meaningful lines from code blocks
            lines = block.split('\n')
            for line in lines:
                if any(keyword in line.lower() for keyword in ['example', 'demo', 'sample', 'test']):
                    examples.append(line.strip())

        # Also look for numbered examples like "1. First example", "2. Second example", etc.
        numbered_examples = re.findall(r'\d+\.\s*([^.\n!?]+[.!?])', content)
        examples.extend([ex.strip() for ex in numbered_examples if ex.strip()])

        # Look for bullet points that might contain examples
        bullet_examples = re.findall(r'[-*]\s*([^.\n!?]*?example[^.\n!?]*[.!?])', content, re.IGNORECASE)
        examples.extend([be.strip() for be in bullet_examples if be.strip()])

        # Remove duplicates while preserving order
        seen = set()
        unique_examples = []
        for example in examples:
            # Clean up the example text
            clean_example = re.sub(r'\s+', ' ', example).strip()
            if clean_example and clean_example not in seen:
                seen.add(clean_example)
                unique_examples.append(clean_example)

        # Filter for relevance based on query
        query_keywords = [kw.lower() for kw in query.lower().split() if len(kw) > 2]  # Only consider words longer than 2 chars
        if query_keywords:
            relevant_examples = []
            for example in unique_examples:
                example_lower = example.lower()
                if any(keyword in example_lower for keyword in query_keywords):
                    relevant_examples.append(example)
            return relevant_examples[:5]  # Return at most 5 relevant examples
        else:
            return unique_examples[:5]  # Return at most 5 examples if no specific query keywords

    def get_examples_for_query(self, query: str, context_chunks: List[Dict[str, Any]]) -> List[str]:
        """
        Get examples from context chunks relevant to the query
        """
        all_examples = []
        for chunk in context_chunks:
            examples = self.extract_examples_from_content(chunk['content'], query)
            all_examples.extend(examples)

        # Remove duplicates while preserving order
        seen = set()
        unique_examples = []
        for example in all_examples:
            if example not in seen:
                seen.add(example)
                unique_examples.append(example)

        return unique_examples

    def delete_user_content_by_session(self, session_id: UUID):
        """
        Delete all user-provided content associated with a session
        """
        # This would delete all user text chunks for a specific session
        # from the user content collection in Qdrant
        logging.info(f"Deleting user content for session_id: {session_id}")

        # In a full implementation, we would:
        # 1. Query the database for all user chunks with this session_id as source_id
        # 2. Delete those points from the user Qdrant collection
        # 3. Delete the database records
        pass