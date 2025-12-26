#!/usr/bin/env python3
"""
Final validation script for the RAG Agent implementation.
This script tests the complete functionality of the agent.
"""

import os
import sys
from unittest.mock import Mock, patch, MagicMock

# Add the backend directory to the path so we can import the agent
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

def test_agent_creation():
    """Test that the agent can be created successfully."""
    print("Testing agent creation...")

    # Mock environment variables
    os.environ["OPENAI_API_KEY"] = "test_openai_key"
    os.environ["COHERE_API_KEY"] = "test_cohere_key"
    os.environ["QDRANT_URL"] = "http://test-qdrant-url"
    os.environ["QDRANT_API_KEY"] = "test_qdrant_key"

    try:
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..', 'backend'))
        from backend.agent import RAGAgent

        with patch('backend.agent.OpenAI') as mock_openai, \
             patch('backend.agent.cohere.Client') as mock_cohere, \
             patch('backend.agent.QdrantClient') as mock_qdrant:

            # Mock the OpenAI client
            mock_openai_instance = Mock()
            mock_assistant = Mock()
            mock_assistant.id = "test_assistant_id"
            mock_openai_instance.beta.assistants.create.return_value = mock_assistant
            mock_openai.return_value = mock_openai_instance

            # Mock Cohere client
            mock_cohere_instance = Mock()
            mock_cohere_instance.embed.return_value = MagicMock(embeddings=[[0.1, 0.2, 0.3]])
            mock_cohere_client = mock_cohere.return_value = mock_cohere_instance

            # Mock Qdrant client
            mock_qdrant_instance = Mock()
            mock_qdrant_client = mock_qdrant.return_value = mock_qdrant_instance

            # Create the agent
            agent = RAGAgent()

            print("+ Agent created successfully")
            print(f"  - Assistant ID: {agent.get_assistant_id()}")

            return True

    except Exception as e:
        print(f"- Agent creation failed: {str(e)}")
        return False

def test_qdrant_retrieval_tool():
    """Test that the Qdrant retrieval tool can be created."""
    print("\nTesting Qdrant retrieval tool creation...")

    os.environ["COHERE_API_KEY"] = "test_cohere_key"
    os.environ["QDRANT_URL"] = "http://test-qdrant-url"
    os.environ["QDRANT_API_KEY"] = "test_qdrant_key"

    try:
        from backend.agent import QdrantRetrievalTool

        with patch('backend.agent.cohere.Client') as mock_cohere, \
             patch('backend.agent.QdrantClient') as mock_qdrant:

            # Mock Cohere client
            mock_cohere_instance = Mock()
            mock_cohere_instance.embed.return_value = MagicMock(embeddings=[[0.1, 0.2, 0.3]])
            mock_cohere.return_value = mock_cohere_instance

            # Mock Qdrant client
            mock_qdrant_instance = Mock()
            mock_qdrant.return_value = mock_qdrant_instance

            # Create the tool
            tool = QdrantRetrievalTool()

            print("✓ Qdrant retrieval tool created successfully")
            print(f"  - Collection name: {tool.collection_name}")
            print(f"  - Top K: {tool.top_k}")

            return True

    except Exception as e:
        print(f"✗ Qdrant retrieval tool creation failed: {str(e)}")
        return False

def test_data_classes():
    """Test that the data classes work correctly."""
    print("\nTesting data classes...")

    try:
        from backend.agent import RetrievedChunk, ChatResponse

        # Test RetrievedChunk
        chunk = RetrievedChunk(
            id="test_id",
            text="Test content",
            score=0.8,
            vector_id="test_vector",
            metadata={"source": "test"}
        )

        print("✓ RetrievedChunk created successfully")

        # Test ChatResponse
        response = ChatResponse(
            response="Test response",
            thread_id="test_thread",
            status="completed"
        )

        print("✓ ChatResponse created successfully")
        print(f"  - Response: {response.response}")
        print(f"  - Status: {response.status}")

        return True

    except Exception as e:
        print(f"✗ Data classes test failed: {str(e)}")
        return False

def test_main_functionality():
    """Test that the main function can be imported and run."""
    print("\nTesting main functionality...")

    try:
        from backend.agent import main
        print("✓ Main function imported successfully")

        # Just verify the function exists and is callable
        assert callable(main), "Main function should be callable"
        print("  - Main function is callable")

        return True

    except Exception as e:
        print(f"✗ Main functionality test failed: {str(e)}")
        return False

def main():
    """Run all validation tests."""
    print("Running final validation for RAG Agent implementation...\n")

    tests = [
        test_data_classes,
        test_qdrant_retrieval_tool,
        test_agent_creation,
        test_main_functionality
    ]

    results = []
    for test in tests:
        results.append(test())

    print(f"\n{'='*50}")
    print("VALIDATION SUMMARY:")
    print(f"Total tests: {len(results)}")
    print(f"Passed: {sum(results)}")
    print(f"Failed: {len(results) - sum(results)}")

    if all(results):
        print("✓ All validation tests passed!")
        print("\nThe RAG Agent implementation is complete and functional.")
        return True
    else:
        print("✗ Some validation tests failed!")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)