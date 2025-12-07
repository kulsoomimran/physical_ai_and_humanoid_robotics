# Architecture Documentation

## Overview
This document describes the architecture of the Physical AI & Humanoid Robotics RAG Chatbot system. The system is built using a microservices architecture with a FastAPI backend and a Docusaurus frontend.

## System Architecture

### Backend (FastAPI)
- **API Layer**: FastAPI endpoints in `/backend/src/api/`
- **Service Layer**: Business logic in `/backend/src/services/`
- **Model Layer**: Pydantic models in `/backend/src/models/`
- **Data Layer**: PostgreSQL with Neon and Qdrant vector store

### Frontend (Docusaurus)
- **Theme Integration**: Chatbot component integrated via Layout wrapper
- **React Components**: Chatbot UI in `/content/src/components/`
- **Styling**: Custom CSS in `/content/src/css/custom.css`

## Components

### Backend Components
- `ConversationService`: Manages conversation sessions
- `RAGService`: Handles retrieval augmented generation
- `DocumentChunkService`: Manages document chunks and embeddings
- `OpenAIAgentService`: Processes queries using OpenAI-compatible interface with Google Gemini

### Frontend Components
- `Chatbot.jsx`: Main chatbot component with message handling
- `Layout.js`: Docusaurus theme wrapper for chatbot integration
- `Chatbot.css`: Styling for the chatbot interface

## API Endpoints

### Chat Endpoints
- `POST /chat/start`: Initialize a new chat session
- `POST /chat/{session_id}/query`: Process a query in a session

### Document Endpoints
- `POST /documents/ingest`: Ingest documents for RAG
- `GET /documents/{doc_id}`: Retrieve document information

## Data Flow

1. User starts a session via `/chat/start`
2. User submits queries via `/chat/{session_id}/query`
3. RAGService retrieves relevant context from vector store
4. OpenAIAgentService processes query with context
5. Response with citations is returned to frontend