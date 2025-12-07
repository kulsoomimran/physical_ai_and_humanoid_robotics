#!/bin/bash

# Deployment script for RAG Chatbot application

set -e  # Exit on any error

echo "Starting deployment of RAG Chatbot application..."

# Check if docker and docker-compose are installed
if ! [ -x "$(command -v docker)" ]; then
  echo "Error: docker is not installed." >&2
  exit 1
fi

if ! [ -x "$(command -v docker-compose)" ]; then
  echo "Error: docker-compose is not installed." >&2
  exit 1
fi

# Load environment variables
if [ -f .env ]; then
    export $(cat .env | xargs)
fi

echo "Building and starting services..."
docker-compose up -d --build

echo "Waiting for services to be ready..."
sleep 10

# Run database migrations (if needed)
echo "Running database migrations..."
docker-compose exec backend python -m src.db.migrate

echo "Deployment completed successfully!"
echo "Services are running:"
docker-compose ps

echo ""
echo "Access the services:"
echo "  - Backend API: http://localhost:8000"
echo "  - Frontend: http://localhost:3000"
echo "  - Health check: http://localhost:8000/health"
echo "  - API docs: http://localhost:8000/docs"

# Optional: Run tests to verify deployment
echo ""
echo "Running smoke tests..."
sleep 5
if curl -f http://localhost:8000/health > /dev/null 2>&1; then
    echo "✓ Health check passed"
else
    echo "✗ Health check failed"
    exit 1
fi

echo ""
echo "Deployment verification completed!"