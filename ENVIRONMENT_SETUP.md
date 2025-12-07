# Environment Configuration

This project uses environment variables to manage sensitive data and configuration settings. All sensitive information is stored in `.env` files and should never be committed to version control.

## Required Environment Variables

### Database Configuration
- `DATABASE_URL`: PostgreSQL database connection string
  - Example: `postgresql+asyncpg://username:password@localhost:5432/rag_chatbot`

### OpenAI Configuration
- `OPENAI_API_KEY`: Your OpenAI API key for the assistant functionality
  - Get it from: https://platform.openai.com/api-keys
- `OPENAI_ASSISTANT_ID` (Optional): Pre-existing assistant ID if you have one

### Qdrant Vector Database Configuration
- `QDRANT_URL`: URL to your Qdrant instance
  - Example: `https://your-qdrant-cluster.qdrant.tech:6333`
- `QDRANT_API_KEY`: Your Qdrant API key

### Security Configuration
- `SECRET_KEY`: Secret key for JWT token signing (use a strong random key in production)
- `ALGORITHM`: Token encryption algorithm (default: HS256)
- `ACCESS_TOKEN_EXPIRE_MINUTES`: Token expiration time (default: 30)

### Application Configuration
- `ENVIRONMENT`: Environment mode (development, production, test)
- `DEBUG`: Enable debug mode (true/false)
- `LOG_LEVEL`: Logging level (INFO, DEBUG, ERROR)

## Setup Instructions

1. Copy the `.env.example` file to create your own `.env` file:
   ```bash
   cp .env.example .env
   ```

2. Update the values in the `.env` file with your actual configuration

3. The application will automatically load these environment variables when it starts

## Security Notes

- Never commit `.env` files to version control
- The `.gitignore` file is configured to exclude all `.env*` files except `.env.example`
- Always use strong, unique keys in production
- Rotate your API keys regularly
- Ensure your `.env` file has appropriate file permissions (e.g., `chmod 600 .env`)