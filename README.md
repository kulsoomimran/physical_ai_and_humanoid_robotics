# RAG Chatbot for Physical AI & Humanoid Robotics Book

An embedded Retrieval-Augmented Generation (RAG) chatbot that allows users to interact with the Physical AI & Humanoid Robotics book content through natural language queries.

## Features

- **RAG-based Q&A**: Ask questions about the book content and get contextually relevant answers
- **User Text Integration**: Query both book content and user-provided text
- **Interactive Learning**: Get explanations with examples, simplified content, or detailed technical information
- **Conversation History**: Maintains context across multiple interactions
- **Source Citations**: All answers include citations to specific book locations

## Prerequisites

- Python 3.8+
- PostgreSQL database
- Qdrant vector database (cloud or self-hosted)
- OpenAI API key (mandatory - used via OpenAI-compatible interface with Google Gemini as fallback)

## Environment Configuration

This project uses environment variables to manage sensitive data and configuration. See [ENVIRONMENT_SETUP.md](./ENVIRONMENT_SETUP.md) for detailed setup instructions.

1. Copy the example environment file:
   ```bash
   cp .env.example .env
   ```

2. Update the `.env` file with your actual configuration:
   - `DATABASE_URL`: PostgreSQL connection string
   - `OPENAI_API_KEY`: Your OpenAI API key (mandatory - used via OpenAI-compatible interface)
   - `QDRANT_URL` and `QDRANT_API_KEY`: Qdrant vector database credentials
   - `GEMINI_API_KEY`: Google Gemini API key (for cost-effective alternative)

## Installation

1. Install backend dependencies:
   ```bash
   cd backend
   pip install -r requirements.txt
   ```

2. Set up the database:
   ```bash
   # Run database migrations
   python -m alembic upgrade head
   ```

## Running the Application

1. Start the backend API:
   ```bash
   cd backend
   uvicorn main:app --reload --port 8000
   ```

2. The API will be available at `http://localhost:8000`

## API Endpoints

- `POST /chat/start` - Start a new chat session
- `POST /chat/{session_token}/query` - Submit a query to the chatbot
- `GET /chat/{session_token}/history` - Get conversation history

## Architecture

- **FastAPI**: Backend web framework
- **OpenAI Assistant API**: Advanced AI agent functionality
- **Qdrant**: Vector database for semantic search
- **PostgreSQL**: Relational database for conversation storage
- **Sentence Transformers**: Text embedding generation

## Security

- All sensitive data is stored in environment variables
- API keys are never hardcoded in the source code
- Proper input validation and sanitization
- JWT-based authentication (if enabled)

## Deployment

### Using Docker Compose (Recommended)

1. Ensure you have Docker and Docker Compose installed
2. Configure your environment variables in `.env`
3. Run the application:
   ```bash
   docker-compose up --build
   ```

### Using the Deployment Script

A deployment script is provided for convenience:

```bash
./deploy.sh
```

This script will build and start all services, run migrations, and verify the deployment.

### Manual Deployment

1. Backend:
   ```bash
   cd backend
   pip install -r requirements.txt
   uvicorn src.main:app --host 0.0.0.0 --port 8000
   ```

2. Frontend:
   ```bash
   cd content
   npm install
   npm start  # For development
   npm run build && npm run serve  # For production
   ```

## Monitoring and Observability

The application includes:
- Health check endpoints at `/health`
- Metrics endpoint at `/metrics`
- Structured logging with JSON format
- Error tracking and reporting
- API call monitoring

## CI/CD Pipeline

The project includes a GitHub Actions workflow for automated testing, building, and deployment. See `.github/workflows/ci-cd.yml` for details.

## License

[Specify your license here]