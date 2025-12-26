import os
import logging
from typing import List, Dict, Optional, Union
import hashlib
from datetime import datetime
import time
from urllib.parse import urljoin, urlparse

# Import required libraries
import cohere
import qdrant_client
from qdrant_client.http import models
import requests
from bs4 import BeautifulSoup
from dotenv import load_dotenv


def get_env_variable(var_name: str, default: Union[str, None] = None) -> str:
    """
    Get an environment variable with proper validation.
    """
    value = os.getenv(var_name, default)
    if value is None:
        raise ValueError(f"Required environment variable '{var_name}' is not set")
    return value


class WebsiteIngestionPipeline:
    """
    A class to handle the complete process of website ingestion,
    embedding generation, and vector storage for RAG chatbot.
    """

    def __init__(self):
        """
        Initialize the WebsiteIngestionPipeline with required clients and configurations.
        """
        # Load environment variables from .env file
        load_dotenv()

        # Configuration parameters
        self.chunk_size = int(get_env_variable('CHUNK_SIZE', '1000'))
        self.chunk_overlap = int(get_env_variable('CHUNK_OVERLAP', '100'))
        self.batch_size = int(get_env_variable('BATCH_SIZE', '96'))
        self.collection_name = get_env_variable('COLLECTION_NAME', 'book_content')
        self.request_timeout = int(get_env_variable('REQUEST_TIMEOUT', '10'))

        # Validate parameters
        if self.chunk_size < 100 or self.chunk_size > 2000:
            raise ValueError("CHUNK_SIZE must be between 100 and 2000")
        if self.chunk_overlap < 0 or self.chunk_overlap > 500:
            raise ValueError("CHUNK_OVERLAP must be between 0 and 500")
        if self.batch_size < 1 or self.batch_size > 96:
            raise ValueError("BATCH_SIZE must be between 1 and 96")
        if self.request_timeout < 1 or self.request_timeout > 60:
            raise ValueError("REQUEST_TIMEOUT must be between 1 and 60")

        # Setup logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s'
        )
        self.logger = logging.getLogger(__name__)

        # Initialize Cohere and Qdrant clients
        self.cohere_client = self._initialize_cohere_client()
        self.qdrant_client = self._initialize_qdrant_client()

        # Ensure the collection exists
        self.create_collection(self.collection_name)

        self.logger.info("WebsiteIngestionPipeline initialized successfully")

    def _initialize_cohere_client(self):
        """
        Initialize Cohere client with API key from environment.
        """
        cohere_api_key = get_env_variable('COHERE_API_KEY')
        client = cohere.Client(cohere_api_key)
        self.logger.info("Cohere client initialized successfully")
        return client

    def _initialize_qdrant_client(self):
        """
        Initialize Qdrant client with URL and API key from environment.
        """
        qdrant_url = get_env_variable('QDRANT_URL')
        qdrant_api_key = get_env_variable('QDRANT_API_KEY')

        client = qdrant_client.QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key
        )
        self.logger.info("Qdrant client initialized successfully")
        return client

    def get_all_urls(self, base_url: str, max_urls: int = 1000) -> List[str]:
        """
        Retrieve all URLs from the target site with a limit to prevent infinite crawling.

        Args:
            base_url: The base URL of the site to crawl
            max_urls: Maximum number of URLs to crawl (prevents infinite loops)

        Returns:
            List of discovered URLs
        """
        # Validate the base URL
        parsed_base = urlparse(base_url)
        if not parsed_base.scheme or not parsed_base.netloc:
            raise ValueError(f"Invalid base URL: {base_url}")

        # Sets to track URLs - using sets for O(1) lookup to prevent infinite loops
        all_urls = set()
        urls_to_visit = {base_url}
        visited_urls = set()

        # Define patterns for non-documentation pages to exclude
        exclude_patterns = [
            '.pdf', '.jpg', '.jpeg', '.png', '.gif', '.zip',
            '.exe', '.dmg', '.deb', '.rpm', '.msi', '.svg',
            '/api/', '/tag/', '/category/', '#', 'mailto:', 'tel:'
        ]

        self.logger.info(f"Starting to crawl: {base_url} (max {max_urls} URLs)")

        # Process URLs in a queue, ensuring we don't process more than max_urls
        while urls_to_visit and len(visited_urls) < max_urls:
            # Get the next URL to process
            current_url = urls_to_visit.pop()

            # Skip if already visited
            if current_url in visited_urls:
                continue

            # Validate URL before processing
            try:
                parsed_url = urlparse(current_url)
                if not parsed_url.scheme or not parsed_url.netloc:
                    self.logger.warning(f"Skipping invalid URL: {current_url}")
                    continue
            except Exception:
                self.logger.warning(f"Skipping malformed URL: {current_url}")
                continue

            # Mark as visited
            visited_urls.add(current_url)

            try:
                # Make request to the current URL
                response = requests.get(
                    current_url,
                    timeout=self.request_timeout,
                    headers={'User-Agent': 'Mozilla/5.0 (compatible; RAGBot/1.0)'}
                )

                # Check if the request was successful
                if response.status_code == 200:
                    # Parse the HTML content
                    soup = BeautifulSoup(response.content, 'html.parser')

                    # Add current URL to the collection (if it's not an excluded type)
                    is_excluded = any(pattern in current_url.lower() for pattern in exclude_patterns)
                    if not is_excluded:
                        all_urls.add(current_url)
                        self.logger.info(f"Added URL: {current_url}")

                    # Find all links on the page with reasonable limits to prevent too many links
                    links = soup.find_all('a', href=True)[:100]  # Limit to 100 links per page

                    for link in links:
                        href = link.get('href', '')

                        # Skip if it's an anchor, external link, or already visited
                        if (href.startswith('#') or
                            '://' in href and urlparse(href).netloc != parsed_base.netloc or
                            href.startswith('mailto:') or href.startswith('tel:')):
                            continue

                        # Convert relative URLs to absolute URLs
                        absolute_url = urljoin(current_url, href)

                        # Validate and filter the URL
                        try:
                            parsed = urlparse(absolute_url)

                            # Only add URLs from the same domain and that haven't been visited or queued
                            if (parsed.netloc == parsed_base.netloc and
                                absolute_url not in visited_urls and
                                absolute_url not in urls_to_visit and
                                not any(pattern in absolute_url.lower() for pattern in exclude_patterns) and
                                len(visited_urls) < max_urls):

                                # Add to queue for processing
                                urls_to_visit.add(absolute_url)
                        except Exception:
                            # Skip invalid URLs
                            continue

                # Add a small delay to be respectful to the server
                time.sleep(0.1)

            except requests.exceptions.Timeout:
                self.logger.error(f"Timeout error accessing URL {current_url}")
                continue
            except requests.exceptions.ConnectionError:
                self.logger.error(f"Connection error accessing URL {current_url}")
                continue
            except requests.exceptions.HTTPError as e:
                self.logger.error(f"HTTP error accessing URL {current_url}: {e}")
                continue
            except requests.RequestException as e:
                self.logger.error(f"Request error accessing URL {current_url}: {e}")
                continue
            except Exception as e:
                self.logger.error(f"Unexpected error processing URL {current_url}: {e}")
                continue

        self.logger.info(f"Crawling completed. Found {len(all_urls)} URLs, visited {len(visited_urls)} pages")
        return list(all_urls)

    def extract_text_from_url(self, url: str) -> Dict[str, str]:
        """
        Extract clean text content from a URL.
        """
        try:
            response = requests.get(
                url,
                timeout=self.request_timeout,
                headers={'User-Agent': 'Mozilla/5.0 (compatible; RAGBot/1.0)'}
            )

            if response.status_code == 200:
                soup = BeautifulSoup(response.content, 'html.parser')

                # Extract page title
                title_tag = soup.find('title')
                page_title = title_tag.get_text().strip() if title_tag else "No Title"

                # Remove script and style elements
                for script in soup(["script", "style", "nav", "header", "footer", "aside"]):
                    script.decompose()

                # Try to find the main content area
                content_selectors = [
                    'main', '.main-wrapper', '.container', '.docItemContainer',
                    '.theme-doc-markdown', '.markdown', '.content', '.post',
                    '.article', 'article'
                ]

                content_element = None
                for selector in content_selectors:
                    content_element = soup.select_one(selector)
                    if content_element:
                        break

                # If no specific content area found, use the body
                if not content_element:
                    content_element = soup.find('body')

                # Extract text from the content element
                if content_element:
                    text = content_element.get_text(separator=' ')
                else:
                    text = soup.get_text(separator=' ')

                # Clean up the text
                import re
                lines = (line.strip() for line in text.splitlines())
                chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
                text = ' '.join(chunk for chunk in chunks if chunk)
                text = re.sub(r'\s+', ' ', text).strip()

                self.logger.info(f"Successfully extracted content from {url} - Title: {page_title}")

                return {
                    'url': url,
                    'title': page_title,
                    'content': text
                }
            else:
                self.logger.error(f"Failed to retrieve content from {url}. Status code: {response.status_code}")
                return {
                    'url': url,
                    'title': 'Error',
                    'content': ''
                }

        except Exception as e:
            self.logger.error(f"Error extracting content from {url}: {e}")
            return {
                'url': url,
                'title': 'Error',
                'content': ''
            }

    def chunk_text(self, text_content: str) -> List[Dict[str, any]]:
        """
        Split content into logical sections for embeddings.
        """
        if len(text_content) <= self.chunk_size:
            return [{
                'text': text_content,
                'start_pos': 0,
                'end_pos': len(text_content),
                'chunk_index': 0
            }]

        chunks = []
        start_idx = 0
        sentence_endings = '.!?;'

        while start_idx < len(text_content):
            end_idx = start_idx + self.chunk_size

            if end_idx >= len(text_content):
                # Final chunk
                chunk_text = text_content[start_idx:end_idx]
                chunks.append({
                    'text': chunk_text,
                    'start_pos': start_idx,
                    'end_pos': end_idx,
                    'chunk_index': len(chunks)
                })
                break

            # Look for sentence boundaries near the end of the chunk
            boundary_found = False
            search_start = end_idx - min(self.chunk_overlap, self.chunk_size // 2)

            # Look backwards for sentence ending
            for i in range(min(end_idx, len(text_content)) - 1, search_start - 1, -1):
                if text_content[i] in sentence_endings:
                    actual_end = i + 1
                    chunk_text = text_content[start_idx:actual_end]
                    chunks.append({
                        'text': chunk_text,
                        'start_pos': start_idx,
                        'end_pos': actual_end,
                        'chunk_index': len(chunks)
                    })
                    start_idx = actual_end - self.chunk_overlap
                    boundary_found = True
                    break

            # If no sentence boundary found, find a space
            if not boundary_found:
                fallback_end = end_idx
                for i in range(min(end_idx, len(text_content)) - 1, search_start - 1, -1):
                    if text_content[i] == ' ':
                        fallback_end = i
                        break

                chunk_text = text_content[start_idx:fallback_end]
                chunks.append({
                    'text': chunk_text,
                    'start_pos': start_idx,
                    'end_pos': fallback_end,
                    'chunk_index': len(chunks)
                })
                start_idx = fallback_end - self.chunk_overlap

        self.logger.info(f"Text chunked into {len(chunks)} chunks")
        return chunks

    def embed(self, text_chunks: List[Dict[str, any]]) -> List[List[float]]:
        """
        Generate embeddings for text chunks using Cohere API with batching and error handling.

        Args:
            text_chunks: List of text chunks (dictionaries with 'text' key)

        Returns:
            List of embedding vectors (each vector as a list of floats)
        """
        import time
        import math

        # Extract text content from chunks
        texts = [chunk['text'] for chunk in text_chunks]

        all_embeddings = []

        # Process in batches to respect API limits (max 96 texts per request)
        batch_size = min(self.batch_size, 96)  # Cohere's max batch size is 96

        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]

            max_retries = 3
            retry_count = 0

            while retry_count < max_retries:
                try:
                    # Generate embeddings using Cohere
                    response = self.cohere_client.embed(
                        texts=batch,
                        model='embed-multilingual-v2.0',  # Configured for technical documentation
                        input_type="search_document"  # Optimized for document retrieval use case
                    )

                    # Extract embeddings from response
                    batch_embeddings = [embedding for embedding in response.embeddings]
                    all_embeddings.extend(batch_embeddings)

                    self.logger.info(f"Generated embeddings for batch {i//batch_size + 1}, "
                                   f"size: {len(batch)} texts -> {len(batch_embeddings)} embeddings")

                    break  # Success, exit retry loop

                except Exception as e:
                    retry_count += 1
                    self.logger.error(f"Error generating embeddings for batch {i//batch_size + 1} "
                                    f"(attempt {retry_count}): {e}")

                    if retry_count < max_retries:
                        # Exponential backoff: wait 2^retry_count seconds
                        wait_time = 2 ** retry_count
                        self.logger.info(f"Retrying in {wait_time} seconds...")
                        time.sleep(wait_time)
                    else:
                        # If all retries failed, mark these embeddings as missing (None) so caller can skip or retry
                        all_embeddings.extend([None] * len(batch))
                        self.logger.error(f"All retries failed for batch {i//batch_size + 1}; skipping these {len(batch)} texts")

        self.logger.info(f"Successfully generated embeddings for {len(texts)} text chunks")
        return all_embeddings

    def create_collection(self, collection_name: str = None) -> bool:
        """
        Create a Qdrant collection for storing embeddings with proper configuration.

        Args:
            collection_name: Name of the collection to create (uses configured default if None)

        Returns:
            True if collection was created or already exists
        """
        from qdrant_client.http import models

        # Use the configured collection name if not provided
        collection_name = collection_name or self.collection_name

        try:
            # Guard recreate_collection behind explicit environment flag to avoid accidental data loss
            allow_recreate = os.getenv('ALLOW_RECREATE_COLLECTION', 'false').lower() == 'true'
            if allow_recreate:
                # Dangerous operation: recreate collection (drops existing data)
                self.qdrant_client.recreate_collection(
                    collection_name=collection_name,
                    vectors_config=models.VectorParams(
                        size=768,
                        distance=models.Distance.COSINE
                    ),
                    optimizers_config=models.OptimizersConfigDiff(
                        memmap_threshold=20000,
                        indexing_threshold=20000,
                    )
                )
                self.logger.info(f"Recreated collection '{collection_name}' (ALLOW_RECREATE_COLLECTION=true)")
            else:
                # Create collection only if it does not exist
                try:
                    _ = self.qdrant_client.get_collection(collection_name)
                    self.logger.info(f"Collection '{collection_name}' already exists; skipping recreate")
                except Exception:
                    # Not found; create it
                    self.qdrant_client.create_collection(
                        collection_name=collection_name,
                        vectors_config=models.VectorParams(
                            size=768,
                            distance=models.Distance.COSINE
                        ),
                        optimizers_config=models.OptimizersConfigDiff(
                            memmap_threshold=20000,
                            indexing_threshold=20000,
                        )
                    )
                    self.logger.info(f"Created collection '{collection_name}'")

            # Ensure payload indexes exist after collection creation/existence check (startup-time only)
            try:
                self.ensure_payload_indexes(collection_name)
            except Exception as e:
                self.logger.error(f"Failed to ensure payload indexes after creating/confirming collection '{collection_name}': {e}")
            return True

        except Exception as e:
            self.logger.error(f"Error creating collection '{collection_name}': {e}")
            return False

    def get_collection_vector_info(self, collection_name: str = None) -> (str, Optional[int]):
        """
        Return (vector_name, size) for the given collection.
        Falls back to ('vector', None) for legacy single-vector collections or on error.
        """
        collection_name = collection_name or self.collection_name
        try:
            info = self.qdrant_client.get_collection(collection_name)
            vectors = getattr(info, 'vectors', None)
            if isinstance(vectors, dict):
                # Named vectors: pick the first one (common single-vector use case)
                first_name = next(iter(vectors.keys()))
                v = vectors[first_name]
                size = None
                if isinstance(v, dict):
                    size = v.get('size')
                else:
                    size = getattr(v, 'size', None)
                return first_name, size
            else:
                # Legacy single-vector case
                size = None
                try:
                    size = getattr(info.vectors, 'size', None)
                except Exception:
                    pass
                return 'vector', size
        except Exception as e:
            self.logger.error(f"Could not fetch collection info for '{collection_name}': {e}")
            return 'vector', None

    def ensure_payload_indexes(self, collection_name: str = None) -> None:
        """
        Ensure payload indexes exist for fields used in filters:
        - source_url -> KEYWORD
        - chunk_index -> INTEGER

        Idempotent and safe to call on startup or immediately after collection creation.
        """
        collection_name = collection_name or self.collection_name

        # Create KEYWORD index for source_url
        try:
            self.qdrant_client.create_payload_index(
                collection_name=collection_name,
                field_name="source_url",
                field_schema=models.PayloadSchemaType.KEYWORD
            )
            self.logger.debug(f"Ensured payload index 'source_url' (KEYWORD) on '{collection_name}'")
        except Exception as e:
            # It's safe to ignore index-already-exists errors
            if "exists" in str(e).lower() or "already exists" in str(e).lower():
                self.logger.debug(f"Payload index 'source_url' already exists: {e}")
            else:
                self.logger.error(f"Failed to create payload index 'source_url': {e}")
                raise

        # Create INTEGER index for chunk_index
        try:
            self.qdrant_client.create_payload_index(
                collection_name=collection_name,
                field_name="chunk_index",
                field_schema=models.PayloadSchemaType.INTEGER
            )
            self.logger.debug(f"Ensured payload index 'chunk_index' (INTEGER) on '{collection_name}'")
        except Exception as e:
            if "exists" in str(e).lower() or "already exists" in str(e).lower():
                self.logger.debug(f"Payload index 'chunk_index' already exists: {e}")
            else:
                self.logger.error(f"Failed to create payload index 'chunk_index': {e}")
                raise

    def setup_schema_on_startup(self, collection_name: str = None) -> None:
        """
        Ensure collection exists and payload indexes exist on application startup.
        This method performs schema operations at startup only and is safe to call once.
        """
        collection_name = collection_name or self.collection_name
        try:
            # If collection exists, ensure payload indexes
            _ = self.qdrant_client.get_collection(collection_name)
            self.logger.info(f"Collection '{collection_name}' exists on startup; ensuring payload indexes")
            try:
                self.ensure_payload_indexes(collection_name)
            except Exception as e:
                self.logger.error(f"Failed to ensure payload indexes on startup for '{collection_name}': {e}")
        except Exception:
            # Collection not found; create it (respects ALLOW_RECREATE_COLLECTION env flag in create_collection)
            self.logger.info(f"Collection '{collection_name}' not found on startup; creating it")
            created = self.create_collection(collection_name)
            if not created:
                raise RuntimeError(f"Failed to create collection '{collection_name}' on startup")

    def make_chunk_point_id(self, source_url: str, chunk_index: int) -> str:
        """
        Deterministically generate a point id for a (source_url, chunk_index) pair.
        Using a shorter hash that fits Qdrant's requirements.
        """
        import uuid
        key = f"{source_url}|{chunk_index}"
        # Create a UUID5 based on the URL and chunk index for deterministic IDs
        namespace_uuid = uuid.uuid5(uuid.NAMESPACE_URL, "rag_pipeline")
        point_id = uuid.uuid5(namespace_uuid, key)
        return str(point_id)

    def generate_document_id(self, source_url: str) -> str:
        """
        Generate a deterministic document_id based on the source_url.
        This ensures consistency if the same URL is processed multiple times.
        """
        import hashlib
        if not source_url:
            # Generate a random UUID if no source_url is provided
            import uuid
            return str(uuid.uuid4())

        # Create a deterministic ID based on the source URL
        # Using SHA256 hash of the URL to create a consistent document ID
        hash_object = hashlib.sha256(source_url.encode('utf-8'))
        hex_dig = hash_object.hexdigest()
        # Take first 32 characters and format as UUID
        return str(uuid.UUID(hex_dig[:32]))

    def check_existing_chunk(self, source_url: str, chunk_index: int, collection_name: str = None) -> Optional[str]: 
        """
        Check if a chunk with the same (source_url, chunk_index) pair already exists in Qdrant.

        Args:
            source_url: The source URL of the chunk
            chunk_index: The index of the chunk within the document
            collection_name: Name of the collection to check (uses configured default if None)

        Returns:
            Point ID if exists, None otherwise
        """
        # Use the configured collection name if not provided
        collection_name = collection_name or self.collection_name

        try:
            # Search for points with matching source_url and chunk_index
            points, _ = self.qdrant_client.scroll(
                collection_name=collection_name,
                scroll_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="source_url",
                            match=models.MatchValue(value=source_url)
                        ),
                        models.FieldCondition(
                            key="chunk_index",
                            match=models.MatchValue(value=chunk_index)
                        )
                    ]
                ),
                limit=1
            )

            if points:  # If any points found
                return points[0].id  # Return the first point's ID
            return None
        except Exception as e:
            self.logger.error(f"Error checking for existing chunk: {e}")
            return None

    def find_and_remove_duplicates(self, collection_name: str = None) -> int:
        """
        Identify and remove duplicate points in Qdrant based on (source_url, chunk_index) pairs.

        Args:
            collection_name: Name of the collection to clean (uses configured default if None)

        Returns:
            Number of duplicates removed
        """
        # Use the configured collection name if not provided
        collection_name = collection_name or self.collection_name

        try:
            # Get all points in the collection using scroll with pagination
            all_points = []
            offset = None
            while True:
                points, next_offset = self.qdrant_client.scroll(
                    collection_name=collection_name,
                    limit=1000,  # Process in batches of 1000
                    offset=offset
                )
                all_points.extend(points)

                if next_offset is None:
                    break
                offset = next_offset

            # Create a mapping of unique identifiers to point IDs based on (source_url, chunk_index)
            unique_key_to_points = {}
            for point in all_points:
                payload = point.payload
                # Create a unique key based on (source_url, chunk_index) as specified in requirements
                unique_key = (
                    payload.get('source_url', ''),
                    payload.get('chunk_index', 0)
                )

                if unique_key not in unique_key_to_points:
                    unique_key_to_points[unique_key] = []
                unique_key_to_points[unique_key].append(point.id)

            # Identify duplicates (keys with more than one point ID)
            duplicates_to_remove = []
            for unique_key, point_ids in unique_key_to_points.items():
                if len(point_ids) > 1:
                    # Keep the first point, mark the rest for deletion
                    duplicates_to_remove.extend(point_ids[1:])

            # Remove duplicates
            if duplicates_to_remove:
                self.qdrant_client.delete(
                    collection_name=collection_name,
                    points_selector=models.PointIdsList(
                        points=duplicates_to_remove
                    )
                )
                self.logger.info(f"Removed {len(duplicates_to_remove)} duplicate points from collection '{collection_name}'")
                return len(duplicates_to_remove)
            else:
                self.logger.info(f"No duplicates found in collection '{collection_name}'")
                return 0

        except Exception as e:
            self.logger.error(f"Error finding and removing duplicates: {e}")
            return 0

    def save_chunk_to_qdrant(self, chunk_data: Dict[str, any], embedding: List[float], collection_name: str = None) -> bool:
        """
        Save a chunk with its embedding to Qdrant with metadata, preventing duplicates.

        Args:
            chunk_data: Dictionary containing chunk information (text, source_url, title, etc.)
            embedding: The embedding vector for the chunk
            collection_name: Name of the collection to save to (uses configured default if None)

        Returns:
            True if successfully saved or already exists, False otherwise
        """
        # Use the configured collection name if not provided
        collection_name = collection_name or self.collection_name

        # Extract source_url and chunk_index to check for duplicates
        source_url = chunk_data.get('url', '')
        chunk_index = chunk_data.get('chunk_index', 0)

        # Use a deterministic point id for idempotent upserts (UUID based on source_url|chunk_index)
        point_id = self.make_chunk_point_id(source_url, chunk_index)

        try:

            # Determine collection vector name and expected size
            vector_name, vector_size = self.get_collection_vector_info(collection_name)

            # Validate embedding length if we know expected size
            if vector_size is not None and len(embedding) != vector_size:
                self.logger.error(f"Embedding size mismatch for collection '{collection_name}': got {len(embedding)}, expected {vector_size}. Skipping upsert.")
                return False

            # Prepare the payload with content and metadata
            payload = {
                'content': chunk_data.get('text', ''),
                'source_url': chunk_data.get('url', ''),
                'page_title': chunk_data.get('title', ''),
                'section_hierarchy': chunk_data.get('section', ''),
                'chunk_index': chunk_data.get('chunk_index', 0),
                'start_pos': chunk_data.get('start_pos', 0),
                'end_pos': chunk_data.get('end_pos', 0),
                'created_at': chunk_data.get('created_at', str(chunk_data.get('timestamp', ''))),
                'document_id': self.generate_document_id(chunk_data.get('url', ''))
            }

            # Remove any None values from payload
            payload = {k: v for k, v in payload.items() if v is not None}

            # Build the PointStruct respecting named vectors if present
            if vector_name == 'vector':
                point = models.PointStruct(id=point_id, vector=embedding, payload=payload)
            else:
                point = models.PointStruct(id=point_id, vectors={vector_name: embedding}, payload=payload)

            # Upload the point to Qdrant
            self.qdrant_client.upsert(
                collection_name=collection_name,
                points=[point]
            )

            self.logger.info(f"Successfully saved chunk to Qdrant with ID: {point_id} (vector_name='{vector_name}', size={len(embedding)})")
            return True

        except Exception as e:
            # Check if it's a capacity-related error
            error_msg = str(e).lower()
            if 'capacity' in error_msg or 'limit' in error_msg or 'memory' in error_msg or 'quota' in error_msg:
                self.logger.error(f"Capacity limit reached in Qdrant: {e}")
            else:
                self.logger.error(f"Error saving chunk to Qdrant: {e}")
            return False

    def batch_upsert_chunks(self, chunks: List[Dict[str, any]], embeddings: List[Optional[List[float]]], collection_name: str = None) -> int:
        """
        Batch upsert a list of chunks with their corresponding embeddings.
        Skips chunks with missing or invalid embeddings.
        Returns number of points upserted.
        """
        collection_name = collection_name or self.collection_name

        # Determine vector name and expected size from collection metadata
        vector_name, vector_size = self.get_collection_vector_info(collection_name)

        points = []
        for chunk, emb in zip(chunks, embeddings):
            if emb is None:
                self.logger.warning(f"Skipping chunk {chunk.get('chunk_index', '?')} from {chunk.get('url', '?')} due to missing embedding")
                continue
            if vector_size is not None and len(emb) != vector_size:
                self.logger.warning(f"Skipping chunk {chunk.get('chunk_index', '?')} due to embedding size mismatch: got {len(emb)}, expected {vector_size}")
                continue

            point_id = self.make_chunk_point_id(chunk.get('url', ''), chunk.get('chunk_index', 0))
            payload = {
                'content': chunk.get('text', ''),
                'source_url': chunk.get('url', ''),
                'page_title': chunk.get('title', ''),
                'section_hierarchy': chunk.get('section', ''),
                'chunk_index': chunk.get('chunk_index', 0),
                'start_pos': chunk.get('start_pos', 0),
                'end_pos': chunk.get('end_pos', 0),
                'created_at': chunk.get('created_at', str(chunk.get('timestamp', ''))),
                'document_id': self.generate_document_id(chunk.get('url', ''))
            }
            payload = {k: v for k, v in payload.items() if v is not None}

            if vector_name == 'vector':
                point = models.PointStruct(id=point_id, vector=emb, payload=payload)
            else:
                point = models.PointStruct(id=point_id, vectors={vector_name: emb}, payload=payload)
            points.append(point)

        if not points:
            self.logger.info(f"No valid points to upsert for collection '{collection_name}'")
            return 0

        try:
            self.qdrant_client.upsert(collection_name=collection_name, points=points)
            self.logger.info(f"Batch upserted {len(points)} points to collection '{collection_name}'")
            return len(points)
        except Exception as e:
            self.logger.error(f"Batch upsert failed for collection '{collection_name}': {e}")
            return 0

    def process_document_with_progress(self, urls: List[str], progress_callback=None) -> Dict[str, any]:
        """
        Process a list of URLs with progress tracking and ETA estimation.

        Args:
            urls: List of URLs to process
            progress_callback: Optional callback function to report progress

        Returns:
            Dictionary containing processed content, embeddings, and metadata
        """
        import time
        from datetime import datetime, timedelta

        total_urls = len(urls)
        processed_count = 0
        results = {
            'processed_content': [],
            'embeddings': [],
            'metadata': [],
            'errors': []
        }

        start_time = time.time()

        # Process in batches to optimize performance
        for i, url in enumerate(urls):
            try:
                # Extract content from URL
                content_result = self.extract_text_from_url(url)

                if content_result['content']:  # Only process if content was successfully extracted
                    # Chunk the content
                    chunks = self.chunk_text(content_result['content'])

                    # Generate embeddings for chunks
                    embeddings = self.embed(chunks)

                    # Batch upsert chunks for this URL (no duplicate checks during ingestion)
                    chunks_with_url = [{**chunk, **content_result} for chunk in chunks]
                    upserted = self.batch_upsert_chunks(chunks_with_url, embeddings, collection_name=self.collection_name)

                    # Record results: treat chunks with None embeddings as failures
                    for chunk, embedding in zip(chunks, embeddings):
                        if embedding is None:
                            results['errors'].append({
                                'url': url,
                                'chunk_index': chunk.get('chunk_index', 0),
                                'error': 'Embedding generation failed'
                            })
                            continue
                        results['processed_content'].append(chunk)
                        results['embeddings'].append(embedding)
                        results['metadata'].append({
                            'url': url,
                            'chunk_index': chunk.get('chunk_index', 0),
                            'text_length': len(chunk.get('text', ''))
                        })

                processed_count += 1

                # Calculate progress and ETA
                elapsed_time = time.time() - start_time
                if processed_count > 0:
                    avg_time_per_url = elapsed_time / processed_count
                    eta_seconds = avg_time_per_url * (total_urls - processed_count)
                    eta = timedelta(seconds=int(eta_seconds))

                    progress_percent = (processed_count / total_urls) * 100
                    # Log progress every 10 URLs or at the end to reduce log spam
                    if processed_count % 10 == 0 or processed_count == total_urls:
                        self.logger.info(f"Progress: {progress_percent:.1f}% ({processed_count}/{total_urls}), ETA: {eta}")

                    # Call progress callback if provided
                    if progress_callback:
                        progress_callback(processed_count, total_urls, eta, progress_percent)
                else:
                    self.logger.info(f"Processing URL {processed_count + 1}/{total_urls}")

            except Exception as e:
                self.logger.error(f"Error processing URL {url}: {e}")
                results['errors'].append({
                    'url': url,
                    'error': str(e)
                })
                processed_count += 1  # Count as processed even if with error

        total_time = time.time() - start_time
        self.logger.info(f"Completed processing {total_urls} URLs in {timedelta(seconds=int(total_time))}")

        return results

    def batch_process_documents(self, urls: List[str], batch_size: int = 10) -> Dict[str, any]:
        """
        Process documents in optimized batches to meet performance criteria (30 min for 100 pages).

        Args:
            urls: List of URLs to process
            batch_size: Number of URLs to process in each batch

        Returns:
            Dictionary containing processed content, embeddings, and metadata
        """
        import time
        from datetime import timedelta

        total_urls = len(urls)
        all_results = {
            'processed_content': [],
            'embeddings': [],
            'metadata': [],
            'errors': []
        }

        start_time = time.time()

        # Process URLs in batches
        for i in range(0, total_urls, batch_size):
            batch_urls = urls[i:i + batch_size]
            self.logger.info(f"Processing batch {i//batch_size + 1}: {len(batch_urls)} URLs")

            # Process each URL in the batch
            for url in batch_urls:
                try:
                    # Extract content from URL
                    content_result = self.extract_text_from_url(url)

                    if content_result['content']:  # Only process if content was successfully extracted
                        # Chunk the content
                        chunks = self.chunk_text(content_result['content'])

                        # Generate embeddings for chunks
                        embeddings = self.embed(chunks)

                        # Batch upsert chunks for this URL (no duplicate checks during ingestion)
                        chunks_with_url = [{**chunk, **content_result} for chunk in chunks]
                        upserted = self.batch_upsert_chunks(chunks_with_url, embeddings, collection_name=self.collection_name)

                        for chunk, embedding in zip(chunks, embeddings):
                            if embedding is None:
                                all_results['errors'].append({
                                    'url': url,
                                    'chunk_index': chunk.get('chunk_index', 0),
                                    'error': 'Embedding generation failed'
                                })
                                continue

                            all_results['processed_content'].append(chunk)
                            all_results['embeddings'].append(embedding)
                            all_results['metadata'].append({
                                'url': url,
                                'chunk_index': chunk.get('chunk_index', 0),
                                'text_length': len(chunk.get('text', ''))
                            })

                except Exception as e:
                    self.logger.error(f"Error processing URL {url}: {e}")
                    all_results['errors'].append({
                        'url': url,
                        'error': str(e)
                    })

            # Log batch completion
            processed_so_far = min(i + batch_size, total_urls)
            elapsed_time = time.time() - start_time
            progress_percent = (processed_so_far / total_urls) * 100
            self.logger.info(f"Completed {processed_so_far}/{total_urls} URLs ({progress_percent:.1f}%)")

        total_time = time.time() - start_time
        self.logger.info(f"Batch processing completed in {timedelta(seconds=int(total_time))}")

        # Performance check
        if total_urls >= 100:
            time_per_100 = (total_time / total_urls) * 100
            self.logger.info(f"Performance: {timedelta(seconds=int(time_per_100))} per 100 pages")
            if time_per_100 <= 1800:  # 30 minutes = 1800 seconds
                self.logger.info("Performance target met: <= 30 minutes for 100 pages")
            else:
                self.logger.warning(f"Performance target missed: {timedelta(seconds=int(time_per_100))} per 100 pages (target: <= 30 minutes)")

        return all_results

    def clean_duplicates(self, collection_name: str = None) -> int:
        """
        Public method to identify and remove duplicate points in Qdrant.

        Args:
            collection_name: Name of the collection to clean (uses configured default if None)

        Returns:
            Number of duplicates removed
        """
        return self.find_and_remove_duplicates(collection_name)


def main():
    """
    Main entry point for the website ingestion pipeline
    Orchestrates: URL retrieval -> Content extraction -> Chunking -> Embedding -> Storage
    """
    import argparse

    # Set up command-line argument parsing
    parser = argparse.ArgumentParser(description='Website Ingestion Pipeline for RAG Chatbot')
    parser.add_argument('--url', type=str, help='Single URL to process')
    parser.add_argument('--base-url', type=str, help='Base URL of Docusaurus site to crawl')
    parser.add_argument('--collection-name', type=str, help='Qdrant collection name')
    parser.add_argument('--chunk-size', type=int, help='Size of text chunks')
    parser.add_argument('--overlap', type=int, help='Overlap between chunks')
    parser.add_argument('--batch-size', type=int, help='Batch size for processing')
    parser.add_argument('--timeout', type=int, help='Request timeout in seconds')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose logging')
    parser.add_argument('--clean-duplicates', action='store_true', help='Clean duplicate points from Qdrant collection')
    parser.add_argument('--process-url', type=str, help='Process a single URL with duplicate checking')

    args = parser.parse_args()

    try:
        pipeline = WebsiteIngestionPipeline()

        # Override configuration with command-line arguments if provided
        if args.collection_name:
            pipeline.collection_name = args.collection_name
        if args.chunk_size:
            pipeline.chunk_size = args.chunk_size
        if args.overlap:
            pipeline.chunk_overlap = args.overlap
        if args.batch_size:
            pipeline.batch_size = args.batch_size
        if args.timeout:
            pipeline.request_timeout = args.timeout

        # Set logging level based on verbose flag
        if args.verbose:
            import logging
            logging.getLogger().setLevel(logging.DEBUG)

        print(f"Website Ingestion Pipeline initialized successfully")
        print(f"Configuration: Collection='{pipeline.collection_name}', Chunk size={pipeline.chunk_size}, "
              f"Overlap={pipeline.chunk_overlap}, Batch size={pipeline.batch_size}")

        # If --clean-duplicates flag is provided, clean duplicates and exit
        if args.clean_duplicates:
            print(f"Cleaning duplicate points from collection '{pipeline.collection_name}'...")
            removed_count = pipeline.clean_duplicates()
            print(f"Removed {removed_count} duplicate points from collection '{pipeline.collection_name}'")
            import sys
            sys.exit(0)

        # If a single URL is provided, process just that URL
        if args.url or args.process_url:
            url_to_process = args.url or args.process_url
            print(f"Processing single URL: {url_to_process}")
            content_result = pipeline.extract_text_from_url(url_to_process)
            if content_result['content']:
                chunks = pipeline.chunk_text(content_result['content'])
                embeddings = pipeline.embed(chunks)
                chunks_with_url = [{**chunk, **content_result} for chunk in chunks]
                upserted = pipeline.batch_upsert_chunks(chunks_with_url, embeddings, collection_name=pipeline.collection_name)

                # Print per-chunk status based on embedding availability
                for chunk, embedding in zip(chunks, embeddings):
                    if embedding is None:
                        print(f"Skipped chunk {chunk.get('chunk_index', 0)} from {url_to_process}: embedding failed")
                    else:
                        print(f"Processed chunk {chunk.get('chunk_index', 0)} from {url_to_process}")
            else:
                print(f"Failed to extract content from {url_to_process}")

        # If a base URL is provided, crawl the site and process all URLs
        elif args.base_url:
            print(f"Crawling site: {args.base_url}")
            # Use a reasonable default max URLs to prevent infinite crawling
            max_urls = int(os.getenv('MAX_CRAWL_URLS', '1000'))
            urls = pipeline.get_all_urls(args.base_url, max_urls=max_urls)
            print(f"Found {len(urls)} URLs to process")

            # Process URLs in batches for better performance
            results = pipeline.batch_process_documents(urls, batch_size=args.batch_size or 10)
            print(f"Processing complete. Successfully processed {len(results['processed_content'])} content chunks, "
                  f"with {len(results['errors'])} errors.")

        else:
            print("No URL provided. Use --url for single URL or --base-url to crawl a site.")
            print("Use --clean-duplicates to remove duplicate points.")
            print("Use --help for more options.")

    except ValueError as e:
        print(f"Error initializing pipeline: {e}")
        print("Please ensure all required environment variables are set in the .env file")
    except Exception as e:
        print(f"Unexpected error: {e}")


if __name__ == "__main__":
    main()