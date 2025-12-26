#!/usr/bin/env python3
"""
AI Agent with Retrieval-Augmented Generation (RAG) Capabilities

This module implements an AI agent using the OpenAI Agents SDK that integrates
with Qdrant for retrieval-augmented generation. The agent responds to user queries
using only retrieved book content.
"""

import os
import logging
import re
from typing import Dict, List, Any
from dataclasses import dataclass
from agents import Agent, Runner, SQLiteSession
from agents import set_tracing_disabled, function_tool
import cohere
from qdrant_client import QdrantClient
from openai import AsyncOpenAI
from agents import OpenAIChatCompletionsModel

# Load environment variables from .env file
from dotenv import load_dotenv
load_dotenv()

set_tracing_disabled(True)

OPEN_ROUTER_API_KEY = os.getenv("OPENROUTER_API_KEY")
client = AsyncOpenAI(
    api_key=(OPEN_ROUTER_API_KEY),
    base_url="https://openrouter.ai/api/v1"
)

third_model = OpenAIChatCompletionsModel(
    openai_client=client,
    model="mistralai/devstral-2512:free"
)

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


@dataclass
class RetrievedChunk:
    """Represents a single chunk of text retrieved from Qdrant."""
    id: str
    text: str
    score: float
    vector_id: str
    metadata: Dict[str, Any]

    def __post_init__(self):
        # Validation
        if not self.text.strip():
            raise ValueError("Text must not be empty")
        if not (0 <= self.score <= 1):
            raise ValueError("Score must be between 0 and 1")
        if not self.metadata:
            raise ValueError("Metadata must contain required source information")


@dataclass
class ChatResponse:
    """The response structure returned by the agent's chat method."""
    response: str
    thread_id: str
    status: str

    def __post_init__(self):
        # Validation
        if not self.response.strip():
            raise ValueError("Response must not be empty")
        if self.status not in ["completed", "error", "requires_action"]:
            raise ValueError(f"Status must be one of: 'completed', 'error', 'requires_action', got: {self.status}")


@function_tool
def search_book_content(query: str, top_k: int = 5) -> str:
    """
    Search through book content to find relevant information for answering user questions.

    Args:
        query: The search query to find relevant information in the book content
        top_k: Number of top results to retrieve (default: 5)

    Returns:
        Formatted string containing the retrieved content
    """
    # Input validation
    if not query or not isinstance(query, str):
        raise ValueError("Query must be a non-empty string")

    # Sanitize the query to prevent injection attacks
    sanitized_query = _sanitize_input(query)

    # Validate top_k parameter
    if not isinstance(top_k, int) or top_k <= 0 or top_k > 20:
        logger.warning(f"Invalid top_k value: {top_k}, defaulting to 5")
        top_k = 5

    # Initialize Qdrant client
    qdrant_client = _get_qdrant_client()

    # Initialize Cohere client
    cohere_api_key = os.getenv("COHERE_API_KEY")
    if not cohere_api_key:
        raise ValueError("COHERE_API_KEY environment variable not set")
    cohere_client = cohere.Client(cohere_api_key)

    # Generate embedding for the query
    query_embedding = generate_query_embedding(sanitized_query, cohere_client)

    # Perform similarity search in Qdrant
    collection_name = os.getenv("QDRANT_COLLECTION", "book_content")
    search_results = qdrant_client.query_points(
        collection_name=collection_name,
        query=query_embedding,
        limit=top_k,
        with_payload=True,
        with_vectors=False
    )

    # Process the search results into formatted content
    formatted_content = "Retrieved content from the book:\n\n"
    for i, result in enumerate(search_results.points, 1):
        # Extract payload
        payload = result.payload or {}

        # Extract content and metadata
        text = payload.get("content", "")
        score = result.score
        source_url = payload.get("source_url", "")

        formatted_content += f"--- Chunk {i} (Relevance: {score:.2f}) ---\n"
        formatted_content += f"{text}\n"
        if source_url:
            formatted_content += f"[Source: {source_url}]\n"
        formatted_content += "\n"

    logger.info(f"Found {len(search_results.points)} chunks for query: '{sanitized_query[:50]}...'")
    return formatted_content


def _get_qdrant_client() -> QdrantClient:
    """Create and return Qdrant client instance - supports both local and cloud instances"""
    try:
        # If QDRANT_URL is set (for cloud instances), use URL with API key
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if qdrant_url and qdrant_url.startswith('http'):
            if not qdrant_api_key:
                logger.warning("QDRANT_API_KEY is required for cloud instances")
            client = QdrantClient(
                url=qdrant_url,
                api_key=qdrant_api_key,
                prefer_grpc=False  # Using REST API for cloud instances
            )
        else:
            # Use host/port for local instances
            qdrant_host = os.getenv("QDRANT_HOST", "localhost")
            qdrant_port = int(os.getenv("QDRANT_PORT", 6333))
            client = QdrantClient(
                host=qdrant_host,
                port=qdrant_port
            )
        return client
    except Exception as e:
        logger.error(f"Failed to create Qdrant client: {str(e)}")
        raise


def generate_query_embedding(query_text: str, cohere_client) -> List[float]:
    """Generate embedding for a query using Cohere"""
    try:
        response = cohere_client.embed(
            texts=[query_text],
            model=os.getenv("COHERE_MODEL", "embed-multilingual-v2.0"),
            input_type="search_query"  # Use search_query for queries
        )
        embedding = response.embeddings[0]
        return embedding
    except Exception as e:
        logger.error(f"Failed to generate query embedding: {str(e)}")
        raise


def _sanitize_input(user_input: str) -> str:
    """
    Sanitize input to prevent injection attacks.
    This is a separate method for input sanitization that can be used by other methods too.
    """
    if not user_input:
        return user_input

    # Remove potentially dangerous characters/sequences
    # This is a basic sanitization - in production, use more comprehensive validation
    sanitized = re.sub(r'[<>"\';]', '', user_input)  # Remove potential SQL/script injection chars
    sanitized = sanitized.strip()
    return sanitized



class RAGAgent:
    """
    The main agent class that orchestrates the interaction between the OpenAI Assistant API
    and the Qdrant retrieval system.
    """

    def __init__(self, assistant_instructions: str = None):
        """
        Initialize the RAG Agent.

        Args:
            assistant_instructions: Custom instructions for the assistant's behavior
        """
        # Set up assistant instructions
        if assistant_instructions:
            self.instructions = assistant_instructions
        else:
            self.instructions = (
                "You are a helpful assistant for robotics textbooks. "
                "Answer questions based only on the provided content from the book. "
                "Do not make up information that is not in the provided content. "
                "If the provided content doesn't contain the answer to a question, "
                "please state that the information is not available in the book."
            )

        # Create the agent with retrieval tool
        self.agent = self._create_agent()
        logger.info(f"RAG Agent initialized")

    def _create_agent(self):
        """
        Create an instance of the Agent with the search_book_content tool.
        """
        # Create the agent with the search tool
        agent = Agent(
            name="RAGRoboticsAssistant",
            instructions=self.instructions,
            tools=[search_book_content],
            model=third_model
        )
        return agent


    async def chat(self, user_message: str, thread_id: str = None) -> ChatResponse:
        """
        Process user message and return response.

        Args:
            user_message: The user's message/query
            thread_id: ID of an existing conversation thread (optional)

        Returns:
            ChatResponse object with the AI response
        """
        try:
            # Input validation and sanitization
            if not user_message or not isinstance(user_message, str):
                raise ValueError("User message must be a non-empty string")

            # Sanitize user input to prevent injection attacks
            sanitized_message = _sanitize_input(user_message)

            # Use SQLiteSession for conversation history if thread_id is provided
            session = None
            if thread_id:
                session = SQLiteSession(thread_id)
            else:
                # Generate a new session ID if not provided
                import uuid
                session = SQLiteSession(str(uuid.uuid4()))

            # Run the agent with the user message using async runner
            result = await Runner.run(
                self.agent,
                input=sanitized_message,
                session=session
            )

            # Get the assistant response from the result
            assistant_response = result.final_output

            # Create and return the ChatResponse
            response = ChatResponse(
                response=assistant_response,
                thread_id=session.session_id,
                status="completed"
            )

            return response

        except ValueError as e:
            # Handle validation errors
            logger.error(f"Validation error in chat method: {str(e)}")
            return ChatResponse(
                response=f"Invalid input: {str(e)}",
                thread_id=thread_id or "",
                status="error"
            )
        except Exception as e:
            # Handle all other errors
            logger.error(f"Error in chat method: {str(e)}")
            return ChatResponse(
                response=f"An error occurred while processing your request: {str(e)}",
                thread_id=thread_id or "",
                status="error"
            )


    def get_agent_id(self) -> str:
        """Return the agent ID."""
        return self.agent.name

    def cleanup(self):
        """Clean up resources."""
        # In a real implementation, you might perform cleanup here
        # For now, we'll just log that cleanup was called
        logger.info(f"Cleaning up RAG Agent with name: {self.agent.name}")


def main():
    """Main function to run the RAG agent from command line."""
    import argparse
    import asyncio

    parser = argparse.ArgumentParser(description='RAG Agent with OpenAI Agents SDK and Qdrant')
    parser.add_argument('--query', type=str, help='Single query to ask the agent')
    parser.add_argument('--interactive', action='store_true', help='Start interactive chat session')

    args = parser.parse_args()

    if args.query:
        # Run single query
        agent = RAGAgent()

        # Run the async chat method in an event loop
        async def run_single_query():
            response = await agent.chat(args.query)
            print(f"Response: {response.response}")

        asyncio.run(run_single_query())

    elif args.interactive:
        # Start interactive session
        print("Starting interactive RAG Agent session. Type 'quit' to exit.")
        agent = RAGAgent()
        thread_id = None

        async def run_interactive_session():
            nonlocal thread_id
            while True:
                user_input = input("\nYou: ")
                if user_input.lower() in ['quit', 'exit', 'q']:
                    break

                response = await agent.chat(user_input, thread_id)
                print(f"Agent: {response.response}")
                thread_id = response.thread_id

        asyncio.run(run_interactive_session())
    else:
        parser.print_help()


if __name__ == "__main__":
    main()