# Quickstart: FastAPI Backend for RAG Chatbot

## Prerequisites

- Python 3.11 or higher
- pip package manager
- OpenAI API key (if using OpenAI agents)

## Setup

1. **Clone the repository** (if applicable):
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Navigate to the backend directory**:
   ```bash
   cd backend
   ```

3. **Create a virtual environment**:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

4. **Install dependencies**:
   ```bash
   pip install fastapi uvicorn python-dotenv openai pydantic
   ```

5. **Create environment file**:
   ```bash
   cp .env.example .env
   ```

6. **Configure environment variables**:
   Edit the `.env` file and add your OpenAI API key:
   ```
   OPENAI_API_KEY=your_openai_api_key_here
   ```

## Running the Application

1. **Start the development server**:
   ```bash
   uvicorn main:app --reload
   ```

2. **The API will be available at**:
   - Base URL: `https://kulsoomimran-rag-chatbot.hf.space/`
   - API Documentation: `https://kulsoomimran-rag-chatbot.hf.space/docs`
   - Alternative Documentation: `https://kulsoomimran-rag-chatbot.hf.space/redoc`

## Testing the Endpoints

### Health Check
```bash
curl https://kulsoomimran-rag-chatbot.hf.space/health
```

### Chat Endpoint
```bash
curl -X POST https://kulsoomimran-rag-chatbot.hf.space/chat \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What are the key components of a humanoid robot?",
    "selected_text": "Humanoid robots typically have a head, torso, arms, and legs..."
  }'
```

## Example Responses

### Health Check Response
```json
{
  "status": "healthy",
  "timestamp": "2025-12-26T10:00:00Z",
  "version": "1.0.0"
}
```

### Chat Response
```json
{
  "response": "The key components of a humanoid robot include sensors, actuators, control systems, and a physical structure...",
  "thread_id": "thread_abc123",
  "timestamp": "2025-12-26T10:00:00Z",
  "metadata": {
    "source_documents": ["doc1.pdf", "doc2.pdf"]
  }
}
```

## Environment Variables

- `OPENAI_API_KEY`: Your OpenAI API key for accessing the agents
- `DEBUG`: Set to "True" for debug mode (default: "False")
- `LOG_LEVEL`: Logging level (default: "INFO")
- `PORT`: Port number for the server (default: 8000)

## Troubleshooting

1. **Port already in use**: Change the port in your run command or kill the process using the port
2. **API key issues**: Ensure your OpenAI API key is correctly set in the environment
3. **Dependency issues**: Make sure all required packages are installed in your virtual environment