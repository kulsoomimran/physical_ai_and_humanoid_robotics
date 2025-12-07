# Setup and Deployment Guide

## Prerequisites

- Python 3.9+
- Node.js 18+
- PostgreSQL database (Neon recommended)
- Qdrant vector database
- Google Cloud account with Vertex AI enabled
- Docusaurus prerequisites

## Environment Setup

Create a `.env` file in the project root with the following variables:

```env
# Backend Configuration
DATABASE_URL="postgresql://username:password@host:port/database"
QDRANT_URL="https://your-qdrant-instance.qdrant.tech:6333"
QDRANT_API_KEY="your-qdrant-api-key"
GOOGLE_API_KEY="your-google-api-key"
GEMINI_MODEL="gemini-1.5-flash"
PROJECT_ID="your-project-id"
LOCATION="your-location"
EMBEDDING_MODEL="text-embedding-005"

# Frontend Configuration
REACT_APP_API_BASE_URL="http://localhost:8000"
```

## Backend Setup

1. Navigate to the backend directory:
   ```bash
   cd backend
   ```

2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Set up environment variables:
   ```bash
   cp .env.example .env
   # Edit .env with your configuration
   ```

4. Run database migrations:
   ```bash
   python -m src.db.migrate
   ```

5. Start the backend server:
   ```bash
   uvicorn src.main:app --reload --port 8000
   ```

## Frontend Setup

1. Navigate to the content directory:
   ```bash
   cd content
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Build the site:
   ```bash
   npm run build
   ```

4. Start the development server:
   ```bash
   npm run start
   ```

## Docker Deployment

### Building Docker Images

Backend:
```bash
docker build -t rag-chatbot-backend -f backend/Dockerfile .
```

Frontend:
```bash
docker build -t rag-chatbot-frontend -f content/Dockerfile .
```

### Running with Docker Compose

```yaml
version: '3.8'
services:
  backend:
    image: rag-chatbot-backend
    ports:
      - "8000:8000"
    environment:
      - DATABASE_URL=postgresql://user:pass@db:5432/chatbot
      - QDRANT_URL=http://qdrant:6333
    depends_on:
      - db
      - qdrant

  frontend:
    image: rag-chatbot-frontend
    ports:
      - "3000:3000"
    depends_on:
      - backend

  db:
    image: postgres:15
    environment:
      - POSTGRES_DB=chatbot
      - POSTGRES_USER=user
      - POSTGRES_PASSWORD=pass

  qdrant:
    image: qdrant/qdrant
    ports:
      - "6333:6333"
```

## Production Deployment

### Backend Deployment
1. Use a cloud platform (AWS, GCP, Azure) or container orchestration
2. Configure environment variables securely
3. Set up health checks and monitoring
4. Configure SSL termination

### Frontend Deployment
1. Build the static site: `npm run build`
2. Deploy to CDN or static hosting (Netlify, Vercel, S3)
3. Configure custom domain and SSL
4. Set up cache invalidation strategies

## Configuration

### Environment Variables

**Required:**
- `DATABASE_URL`: PostgreSQL connection string
- `QDRANT_URL`: Qdrant vector database URL
- `GOOGLE_API_KEY`: Google Cloud API key
- `PROJECT_ID`: Google Cloud project ID

**Optional:**
- `PORT`: Backend server port (default: 8000)
- `LOG_LEVEL`: Logging level (default: INFO)
- `MAX_CONTEXT_LENGTH`: Maximum context length for RAG (default: 4096)