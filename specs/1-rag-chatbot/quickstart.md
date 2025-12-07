# Quickstart Guide: Embedded RAG Chatbot

**Feature**: 1-rag-chatbot
**Date**: 2025-12-07

## Development Environment Setup

### Prerequisites
- Python 3.11+
- Node.js 18+ (for Docusaurus integration)
- Access to Qdrant Cloud
- Gemini API key
- PostgreSQL-compatible database (Neon)

### Backend Setup
1. Navigate to the backend directory
2. Install Python dependencies:
   ```bash
   pip install fastapi uvicorn sentence-transformers qdrant-client psycopg2-binary python-multipart python-jose[cryptography] passlib[bcrypt] python-dotenv
   ```
3. Set up environment variables:
   ```bash
   export GEMINI_API_KEY="your-gemini-api-key"
   export DATABASE_URL="your-neon-postgres-connection-string"
   export QDRANT_URL="your-qdrant-cloud-url"
   export QDRANT_API_KEY="your-qdrant-api-key"
   ```
4. Run database migrations
5. Start the backend server:
   ```bash
   uvicorn main:app --reload
   ```

### Frontend/Docusaurus Integration
1. Navigate to the Docusaurus directory
2. Install dependencies:
   ```bash
   npm install
   ```
3. Add the chatbot component to your Docusaurus pages
4. Configure the API endpoint for the chatbot
5. Build and serve:
   ```bash
   npm run build
   npm run serve
   ```

## API Usage Examples

### Starting a new chat session:
```bash
curl -X POST http://localhost:8000/chat/start \
  -H "Content-Type: application/json" \
  -d '{}'
```

### Querying the chatbot:
```bash
curl -X POST http://localhost:8000/chat/{session_token}/query \
  -H "Content-Type: application/json" \
  -d '{
    "session_token": "{session_token}",
    "query": "Explain how ROS 2 differs from ROS 1 in robotics development?",
    "context_mode": "book_content",
    "response_style": "detailed"
  }'
```

## Key Integration Points

### With context7 MCP Server
- The system connects to the context7 MCP server for orchestration (if configured)
- Configuration is in `.specify/memory/constitution.md`
- Services are orchestrated via the MCP framework

### With Docusaurus
- The chatbot UI component is integrated as a React component in `content/src/components/Chatbot.jsx`
- The component is automatically added to all book pages via `content/src/theme/Layout.js`
- The chatbot widget appears in the bottom-right corner of book pages (except homepage and blog)

### With Qdrant Vector Database
- Document chunks are stored as vectors in Qdrant
- Use semantic similarity search for retrieving relevant content
- Maintain synchronization between Postgres metadata and Qdrant vectors

## Running the Application

### Backend
1. Set up your environment variables in a `.env` file (copy from `.env.example`)
2. Install dependencies: `pip install -r requirements.txt`
3. Run the backend: `uvicorn src.main:app --reload --port 8000`

### Frontend
1. Navigate to the content directory: `cd content`
2. Install dependencies: `npm install`
3. Start the development server: `npm start`
4. The Docusaurus site will be available at `http://localhost:3000`

## Validation Steps
1. Ensure the backend is running on `http://localhost:8000`
2. Ensure the frontend is running on `http://localhost:3000`
3. Visit the frontend and verify the chatbot appears on book pages
4. Test the API endpoints using the curl examples above
5. Verify that queries return relevant responses from the book content