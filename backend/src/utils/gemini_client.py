import os
import google.generativeai as genai
from typing import Optional, List, Dict, Any
from src.core.config import settings
from src.utils.exceptions import ConfigurationError
import logging

logger = logging.getLogger(__name__)

class GeminiClient:
    """
    Client for interacting with the Gemini API
    """
    def __init__(self):
        api_key = settings.GEMINI_API_KEY
        if not api_key:
            raise ConfigurationError("GEMINI_API_KEY environment variable is not set")

        genai.configure(api_key=api_key)
        self.model = genai.GenerativeModel('gemini-pro')

    def generate_response(self, prompt: str, context: Optional[str] = None) -> str:
        """
        Generate a response from the Gemini API based on the prompt and optional context
        """
        try:
            if context:
                full_prompt = f"Context: {context}\n\nQuestion: {prompt}"
            else:
                full_prompt = prompt

            response = self.model.generate_content(full_prompt)
            return response.text
        except Exception as e:
            logger.error(f"Error generating response from Gemini API: {str(e)}")
            raise

    def embed_content(self, text: str) -> List[float]:
        """
        Generate embeddings for the given text using Gemini's embedding capabilities
        """
        try:
            result = genai.embed_content(
                model="models/embedding-001",
                content=text,
                task_type="retrieval_document",
                title="RAG Chatbot Document"
            )
            return result['embedding']
        except Exception as e:
            logger.error(f"Error generating embeddings from Gemini API: {str(e)}")
            raise

# Create a global instance
gemini_client = None

def get_gemini_client() -> GeminiClient:
    """
    Get or create the Gemini client instance
    """
    global gemini_client
    if gemini_client is None:
        gemini_client = GeminiClient()
    return gemini_client