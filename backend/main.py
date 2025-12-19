import os
import logging
from typing import List, Dict, Optional, Union
import uuid
from datetime import datetime

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

    Args:
        var_name: Name of the environment variable
        default: Default value if the variable is not found

    Returns:
        The value of the environment variable or the default value

    Raises:
        ValueError: If the variable is required but not found
    """
    value = os.getenv(var_name, default)
    if value is None:
        raise ValueError(f"Required environment variable '{var_name}' is not set")
    return value


def validate_config_parameter(param_name: str, value: Union[int, str], min_val: Union[int, None] = None, max_val: Union[int, None] = None) -> Union[int, str]:
    """
    Validate a configuration parameter with optional min/max constraints.

    Args:
        param_name: Name of the parameter for error messages
        value: The value to validate
        min_val: Minimum allowed value (for int parameters)
        max_val: Maximum allowed value (for int parameters)

    Returns:
        The validated value
    """
    if isinstance(value, int) and min_val is not None and value < min_val:
        raise ValueError(f"Configuration parameter '{param_name}' must be at least {min_val}, got {value}")

    if isinstance(value, int) and max_val is not None and value > max_val:
        raise ValueError(f"Configuration parameter '{param_name}' must be at most {max_val}, got {value}")

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

        # Configuration parameters with validation using utility functions
        self.chunk_size = validate_config_parameter('CHUNK_SIZE', int(get_env_variable('CHUNK_SIZE', '1000')), min_val=100, max_val=2000)
        self.chunk_overlap = validate_config_parameter('CHUNK_OVERLAP', int(get_env_variable('CHUNK_OVERLAP', '100')), min_val=0, max_val=500)
        self.batch_size = validate_config_parameter('BATCH_SIZE', int(get_env_variable('BATCH_SIZE', '96')), min_val=1, max_val=96)
        self.collection_name = get_env_variable('COLLECTION_NAME', 'rag_chatbot')
        self.request_timeout = validate_config_parameter('REQUEST_TIMEOUT', int(get_env_variable('REQUEST_TIMEOUT', '10')), min_val=1, max_val=60)

        # Setup logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s'
        )
        self.logger = logging.getLogger(__name__)

        # Initialize Cohere and Qdrant clients
        self.cohere_client = self._initialize_cohere_client()
        self.qdrant_client = self._initialize_qdrant_client()

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

    def get_all_urls(self, base_url: str) -> List[str]:
        """
        Retrieve all URLs from the target Docusaurus site using breadth-first search.

        Args:
            base_url: The base URL of the Docusaurus site to crawl

        Returns:
            List of all discovered URLs
        """
        from urllib.parse import urljoin, urlparse
        import time

        # Validate the base URL
        parsed_base = urlparse(base_url)
        if not parsed_base.scheme or not parsed_base.netloc:
            raise ValueError(f"Invalid base URL: {base_url}")

        # Set to store all discovered URLs
        all_urls = set()
        # Queue for BFS
        urls_to_visit = [base_url]
        # Set to track visited URLs
        visited_urls = set()

        # Define patterns for non-documentation pages to exclude
        exclude_patterns = [
            '.pdf', '.jpg', '.jpeg', '.png', '.gif', '.zip',
            '.exe', '.dmg', '.deb', '.rpm', '.msi', '.svg',
            '/api/', '/tag/', '/category/'
        ]

        self.logger.info(f"Starting to crawl: {base_url}")

        while urls_to_visit:
            current_url = urls_to_visit.pop(0)

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
                # Add a small delay to be respectful to the server
                time.sleep(0.1)

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

                    # Find all links on the page
                    for link in soup.find_all('a', href=True):
                        href = link['href']

                        # Convert relative URLs to absolute URLs
                        absolute_url = urljoin(current_url, href)

                        # Validate and filter the URL
                        try:
                            parsed = urlparse(absolute_url)

                            # Only add URLs from the same domain and that haven't been visited
                            if (parsed.netloc == parsed_base.netloc and
                                absolute_url not in visited_urls and
                                absolute_url not in urls_to_visit and
                                not any(pattern in absolute_url.lower() for pattern in exclude_patterns)):

                                # Add to queue for further exploration
                                urls_to_visit.append(absolute_url)
                        except Exception:
                            # Skip invalid URLs
                            continue

            except requests.exceptions.Timeout:
                self.logger.error(f"Timeout error accessing URL {current_url}")
                continue  # Continue with the next URL in the queue
            except requests.exceptions.ConnectionError:
                self.logger.error(f"Connection error accessing URL {current_url}")
                continue  # Continue with the next URL in the queue
            except requests.exceptions.HTTPError as e:
                self.logger.error(f"HTTP error accessing URL {current_url}: {e}")
                continue  # Continue with the next URL in the queue
            except requests.RequestException as e:
                self.logger.error(f"Request error accessing URL {current_url}: {e}")
                continue  # Continue with the next URL in the queue
            except Exception as e:
                self.logger.error(f"Unexpected error processing URL {current_url}: {e}")
                continue  # Continue with the next URL in the queue

        self.logger.info(f"Crawling completed. Found {len(all_urls)} URLs")
        return list(all_urls)

    def extract_text_from_url(self, url: str) -> Dict[str, str]:
        """
        Extract clean text content from a URL.

        Args:
            url: The URL to extract content from

        Returns:
            Dictionary containing the page title and extracted text content
        """
        try:
            # Make request to the URL with comprehensive error handling
            response = requests.get(
                url,
                timeout=self.request_timeout,
                headers={'User-Agent': 'Mozilla/5.0 (compatible; RAGBot/1.0)'}
            )

            # Check for successful response
            if response.status_code == 200:
                # Parse the HTML content
                soup = BeautifulSoup(response.content, 'html.parser')

                # Extract page title
                title_tag = soup.find('title')
                page_title = title_tag.get_text().strip() if title_tag else "No Title"

                # Remove script and style elements
                for script in soup(["script", "style", "nav", "header", "footer", "aside"]):
                    script.decompose()

                # Try to find the main content area using Docusaurus-specific selectors
                # Docusaurus typically uses these classes for main content
                content_selectors = [
                    'main',  # HTML5 main element
                    '.main-wrapper',  # Common Docusaurus wrapper
                    '.container',  # Common container class
                    '.docItemContainer',  # Docusaurus documentation container
                    '.theme-doc-markdown',  # Docusaurus markdown content
                    '.markdown',  # Markdown content
                    '.content',  # General content class
                    '.post',  # Blog post content
                    '.article',  # Article content
                    'article'  # HTML5 article element
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
                    # Get text and clean it up
                    text = content_element.get_text(separator=' ')
                else:
                    # If no content element found, get all text from the body
                    text = soup.get_text(separator=' ')

                # Clean up the text: remove extra whitespace and empty lines
                lines = (line.strip() for line in text.splitlines())
                chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
                text = ' '.join(chunk for chunk in chunks if chunk)

                # Sanitize the content to remove special characters
                # Remove extra whitespace and normalize
                import re
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

        except requests.exceptions.Timeout:
            self.logger.error(f"Timeout error accessing URL {url}")
            return {
                'url': url,
                'title': 'Error',
                'content': ''
            }
        except requests.exceptions.ConnectionError:
            self.logger.error(f"Connection error accessing URL {url}")
            return {
                'url': url,
                'title': 'Error',
                'content': ''
            }
        except requests.exceptions.HTTPError as e:
            self.logger.error(f"HTTP error accessing URL {url}: {e}")
            return {
                'url': url,
                'title': 'Error',
                'content': ''
            }
        except requests.RequestException as e:
            self.logger.error(f"Request error accessing URL {url}: {e}")
            return {
                'url': url,
                'title': 'Error',
                'content': ''
            }
        except Exception as e:
            self.logger.error(f"Unexpected error extracting content from {url}: {e}")
            return {
                'url': url,
                'title': 'Error',
                'content': ''
            }

    def chunk_text(self, text_content: str, chunk_size: Optional[int] = None, overlap: Optional[int] = None) -> List[Dict[str, any]]:
        """
        Split content into logical sections suitable for embeddings with overlap to maintain context.

        Args:
            text_content: The text content to be chunked
            chunk_size: Size of each chunk (uses configured default if None)
            overlap: Overlap size between chunks (uses configured default if None)

        Returns:
            List of dictionaries containing chunk text and metadata
        """
        # Use the configured defaults if not provided
        chunk_size = chunk_size or self.chunk_size
        overlap = overlap or self.chunk_overlap

        # Validate parameters
        if chunk_size <= 0:
            raise ValueError(f"Chunk size must be positive, got {chunk_size}")
        if overlap < 0 or overlap >= chunk_size:
            raise ValueError(f"Overlap must be non-negative and less than chunk_size, got overlap={overlap}, chunk_size={chunk_size}")

        # Handle very large documents by implementing a reasonable upper limit check
        # Though we don't split the document itself, we ensure the chunking algorithm can handle it
        if len(text_content) > 1000000:  # 1MB limit - extremely large for a single document
            self.logger.warning(f"Processing very large document of {len(text_content)} characters. This may take additional time.")

        # Define sentence ending characters for boundary detection
        sentence_endings = '.!?;'

        chunks = []
        start_idx = 0

        while start_idx < len(text_content):
            # Determine the end position
            end_idx = start_idx + chunk_size

            # If we're at the end of the text, create the final chunk
            if end_idx >= len(text_content):
                end_idx = len(text_content)
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
            search_start = end_idx - min(overlap, chunk_size // 2)  # Search in the last portion

            # Look backwards for sentence ending
            for i in range(min(end_idx, len(text_content)) - 1, search_start - 1, -1):
                if text_content[i] in sentence_endings:
                    # Found a sentence ending, make the cut after it
                    actual_end = i + 1
                    chunk_text = text_content[start_idx:actual_end]
                    chunks.append({
                        'text': chunk_text,
                        'start_pos': start_idx,
                        'end_pos': actual_end,
                        'chunk_index': len(chunks)
                    })
                    start_idx = actual_end - overlap  # Move start position with overlap
                    boundary_found = True
                    break

            # If no sentence boundary found, use a fallback approach
            if not boundary_found:
                # Find a space to avoid cutting words
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
                start_idx = fallback_end - overlap  # Move start position with overlap

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
                        # If all retries failed, add zero vectors as fallback and continue
                        fallback_embeddings = [[0.0] * 768 for _ in range(len(batch))]
                        all_embeddings.extend(fallback_embeddings)
                        self.logger.error(f"All retries failed for batch {i//batch_size + 1}, "
                                        f"using fallback zero vectors")

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
            # Create or recreate collection with 768-dimensional vectors (for Cohere multilingual embeddings)
            # and cosine similarity for semantic search - using recreate_collection will handle both creation and update
            self.qdrant_client.recreate_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(
                    size=768,  # Cohere multilingual embeddings are 768-dimensional
                    distance=models.Distance.COSINE  # Cosine similarity for text embeddings
                ),
                # Enable payload storage for metadata
                optimizers_config=models.OptimizersConfigDiff(
                    memmap_threshold=20000,
                    indexing_threshold=20000,
                )
            )

            self.logger.info(f"Created/updated collection '{collection_name}' with 768-dimensional vectors "
                           f"and cosine similarity distance function")
            return True

        except Exception as e:
            self.logger.error(f"Error creating collection '{collection_name}': {e}")
            return False

    def save_chunk_to_qdrant(self, chunk_data: Dict[str, any], embedding: List[float], collection_name: str = None) -> bool:
        """
        Save a chunk with its embedding to Qdrant with metadata.

        Args:
            chunk_data: Dictionary containing chunk information (text, source_url, title, etc.)
            embedding: The embedding vector for the chunk
            collection_name: Name of the collection to save to (uses configured default if None)

        Returns:
            True if successfully saved, False otherwise
        """
        import uuid

        # Use the configured collection name if not provided
        collection_name = collection_name or self.collection_name

        try:
            # Generate a unique UUID for this record
            point_id = str(uuid.uuid4())

            # Prepare the payload with content and metadata
            payload = {
                'content': chunk_data.get('text', ''),
                'source_url': chunk_data.get('url', ''),
                'page_title': chunk_data.get('title', ''),
                'section_hierarchy': chunk_data.get('section', ''),
                'chunk_index': chunk_data.get('chunk_index', 0),
                'start_pos': chunk_data.get('start_pos', 0),
                'end_pos': chunk_data.get('end_pos', 0),
                'created_at': chunk_data.get('created_at', str(chunk_data.get('timestamp', '')))
            }

            # Remove any None values from payload
            payload = {k: v for k, v in payload.items() if v is not None}

            # Check collection size before inserting to handle capacity limits
            try:
                collection_info = self.qdrant_client.get_collection(collection_name)
                current_points_count = collection_info.points_count

                # Log a warning if we're approaching potential capacity issues
                if current_points_count > 100000:  # Arbitrary threshold for logging
                    self.logger.info(f"Collection '{collection_name}' has {current_points_count} points. Current count: {current_points_count}")
            except Exception:
                # If we can't get collection info, continue with the operation
                pass

            # Upload the point to Qdrant
            self.qdrant_client.upsert(
                collection_name=collection_name,
                points=[
                    models.PointStruct(
                        id=point_id,
                        vector=embedding,
                        payload=payload
                    )
                ]
            )

            self.logger.info(f"Successfully saved chunk to Qdrant with ID: {point_id}")
            return True

        except Exception as e:
            # Check if it's a capacity-related error
            error_msg = str(e).lower()
            if 'capacity' in error_msg or 'limit' in error_msg or 'memory' in error_msg or 'quota' in error_msg:
                self.logger.error(f"Capacity limit reached in Qdrant: {e}")
            else:
                self.logger.error(f"Error saving chunk to Qdrant: {e}")
            return False

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
                    chunks = self.chunk_text(content_result)

                    # Generate embeddings for chunks
                    embeddings = self.embed(chunks)

                    # Save to Qdrant
                    for chunk, embedding in zip(chunks, embeddings):
                        # Add URL info to chunk
                        chunk_with_url = {**chunk, **content_result}
                        success = self.save_chunk_to_qdrant(chunk_with_url, embedding)

                        if success:
                            results['processed_content'].append(chunk)
                            results['embeddings'].append(embedding)
                            results['metadata'].append({
                                'url': url,
                                'chunk_index': chunk.get('chunk_index', 0),
                                'text_length': len(chunk.get('text', ''))
                            })
                        else:
                            results['errors'].append({
                                'url': url,
                                'chunk_index': chunk.get('chunk_index', 0),
                                'error': 'Failed to save to Qdrant'
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
                        chunks = self.chunk_text(content_result)

                        # Generate embeddings for chunks
                        embeddings = self.embed(chunks)

                        # Save to Qdrant
                        for chunk, embedding in zip(chunks, embeddings):
                            # Add URL info to chunk
                            chunk_with_url = {**chunk, **content_result}
                            success = self.save_chunk_to_qdrant(chunk_with_url, embedding)

                            if success:
                                all_results['processed_content'].append(chunk)
                                all_results['embeddings'].append(embedding)
                                all_results['metadata'].append({
                                    'url': url,
                                    'chunk_index': chunk.get('chunk_index', 0),
                                    'text_length': len(chunk.get('text', ''))
                                })
                            else:
                                all_results['errors'].append({
                                    'url': url,
                                    'chunk_index': chunk.get('chunk_index', 0),
                                    'error': 'Failed to save to Qdrant'
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

        # If a single URL is provided, process just that URL
        if args.url:
            print(f"Processing single URL: {args.url}")
            content_result = pipeline.extract_text_from_url(args.url)
            if content_result['content']:
                chunks = pipeline.chunk_text(content_result)
                embeddings = pipeline.embed(chunks)
                for chunk, embedding in zip(chunks, embeddings):
                    chunk_with_url = {**chunk, **content_result}
                    success = pipeline.save_chunk_to_qdrant(chunk_with_url, embedding)
                    if success:
                        print(f"Successfully processed and saved chunk {chunk.get('chunk_index', 0)} from {args.url}")
                    else:
                        print(f"Failed to save chunk {chunk.get('chunk_index', 0)} from {args.url}")
            else:
                print(f"Failed to extract content from {args.url}")

        # If a base URL is provided, crawl the site and process all URLs
        elif args.base_url:
            print(f"Crawling site: {args.base_url}")
            urls = pipeline.get_all_urls(args.base_url)
            print(f"Found {len(urls)} URLs to process")

            # Process URLs in batches for better performance
            results = pipeline.batch_process_documents(urls, batch_size=args.batch_size or 10)
            print(f"Processing complete. Successfully processed {len(results['processed_content'])} content chunks, "
                  f"with {len(results['errors'])} errors.")

        else:
            print("No URL provided. Use --url for single URL or --base-url to crawl a site.")
            print("Use --help for more options.")

    except ValueError as e:
        print(f"Error initializing pipeline: {e}")
        print("Please ensure all required environment variables are set in the .env file")
    except Exception as e:
        print(f"Unexpected error: {e}")


if __name__ == "__main__":
    main()