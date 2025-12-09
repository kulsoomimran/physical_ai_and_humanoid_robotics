from fastapi import FastAPI
import logging
import os

# Set up basic logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create FastAPI app without any complex initialization
app = FastAPI(
    title="RAG Chatbot API - Minimal Version",
    description="Minimal version for Render deployment testing",
    version="1.0.0"
)

@app.get("/")
def read_root():
    return {"message": "Minimal API is running!"}

@app.get("/health")
def health_check():
    return {"status": "healthy", "message": "Server is running"}

# Only add basic logging
logger.info("Minimal application initialized")

if __name__ == "__main__":
    import uvicorn
    port = int(os.environ.get("PORT", 8000))
    uvicorn.run("main_minimal:app", host="0.0.0.0", port=port, reload=True)