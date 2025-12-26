# Quickstart Guide: Agent with Retrieval-Augmented Generation

## Overview

This guide provides instructions for quickly setting up and using the AI agent with retrieval-augmented generation capabilities. The agent uses the OpenAI Assistant API and integrates with Qdrant to answer questions based only on retrieved book content.

## Prerequisites

### Environment Setup
1. Ensure you have Python 3.8+ installed
2. Install required dependencies: `pip install -r requirements.txt`
3. Set up your environment variables in a `.env` file (or ensure they're available in your environment)

### Required Environment Variables
```bash
# OpenAI API key for the Assistant API
OPENAI_API_KEY=your_openai_api_key_here

# Qdrant configuration for vector storage
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION=book_content  # or your collection name

# Cohere API key for embeddings
COHERE_API_KEY=your_cohere_api_key_here
```

### Data Requirements
- Book content must already be indexed in Qdrant
- The collection should contain properly embedded content chunks
- Metadata including source URLs should be available

## Installation

1. **Install dependencies**
   ```bash
   pip install openai qdrant-client cohere python-dotenv
   ```

2. **Verify environment configuration**
   ```bash
   python -c "import openai, qdrant_client, cohere; print('Dependencies installed successfully')"
   ```

## Quick Usage

### 1. Basic Agent Initialization
```python
from backend.agent import RAGAgent

# Initialize the agent
agent = RAGAgent()

# Ask a question about the book content
result = agent.chat("What is the main concept discussed in the book?")
print(result['response'])
```

### 2. Single Query Command Line
```bash
# Ask a single question
python backend/agent.py --query "What is humanoid robotics?"
```

### 3. Interactive Chat Session
```bash
# Start an interactive session
python backend/agent.py --interactive
```

## Configuration Options

### Custom Assistant Instructions
```python
custom_instructions = "You are a helpful assistant for robotics textbooks. Answer questions based only on the provided content."
agent = RAGAgent(assistant_instructions=custom_instructions)
```

### Environment Variables for Configuration
- `QDRANT_COLLECTION`: Name of the Qdrant collection to search (default: "book_content")
- `DEFAULT_TOP_K`: Number of results to retrieve (default: 5)
- `OPENAI_MODEL`: Model to use for the assistant (default: "gpt-4-turbo")

## Expected Output

When querying the agent, you should receive:
- `response`: The AI-generated response to your question
- `thread_id`: ID for continuing the conversation
- `status`: Status of the operation

Example:
```python
{
    "response": "The book defines humanoid robotics as...",
    "thread_id": "thread_abc123...",
    "status": "completed"
}
```

## Troubleshooting

### Common Issues

1. **API Key Errors**
   - Verify all required API keys are set in your environment
   - Check for typos in the API key values

2. **Qdrant Connection Issues**
   - Verify QDRANT_URL and QDRANT_API_KEY are correct
   - Ensure the Qdrant service is running and accessible
   - Check that the specified collection exists

3. **Empty Results**
   - Verify that book content has been properly indexed in Qdrant
   - Check that your query matches the content in the database

4. **Rate Limiting**
   - If encountering rate limit errors, implement appropriate delays
   - Consider using a higher-tier API plan if needed

## Next Steps

1. **Integration**: Integrate the agent into your application
2. **Customization**: Modify the assistant instructions for your specific use case
3. **Testing**: Run the full test suite to verify functionality
4. **Production**: Deploy with proper security and monitoring

## Example Use Cases

- **Book Q&A**: Answer questions about specific book content
- **Documentation Assistant**: Help users navigate technical documentation
- **Educational Tool**: Assist students in understanding textbook content
- **Content Retrieval**: Find specific information within large document collections