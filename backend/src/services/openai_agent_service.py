import asyncio
from typing import List, Dict, Any
import os
from openai import OpenAI
from ..models.source_citation import SourceCitation
from sqlalchemy.orm import Session


class OpenAIAgentService:
    """
    Service class for interacting with OpenAI's API or Google's Gemini API for response generation using agents
    Supports both OpenAI API and Google's Gemini API through OpenAI-compatible interface
    """

    def __init__(self, api_key: str = None, assistant_id: str = None, use_gemini: bool = False):
        # If not provided explicitly, try to get from settings/config, then from environment
        if not api_key:
            from ..core.config import settings
            if use_gemini:
                # Use Gemini API key when using Google's service
                api_key = api_key or settings.GEMINI_API_KEY or os.getenv("GEMINI_API_KEY")
            else:
                # Use OpenAI API key for OpenAI service
                api_key = api_key or settings.OPENAI_API_KEY or os.getenv("OPENAI_API_KEY")

        if not api_key:
            raise ValueError("API key is required")

        # Set up the client based on whether we're using Gemini or OpenAI
        if use_gemini:
            # Use Google's API with OpenAI-compatible endpoint
            self.client = OpenAI(
                api_key=api_key,
                base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
            )
            self.use_gemini = True
            # Get model from settings/config
            from ..core.config import settings
            self.model_name = settings.GEMINI_MODEL or os.getenv("GEMINI_MODEL", "gemini-1.5-flash")
        else:
            self.client = OpenAI(api_key=api_key)
            self.use_gemini = False
            self.model_name = os.getenv("OPENAI_MODEL", "gpt-3.5-turbo")

        # Use the provided assistant_id or try to get from settings
        self.assistant_id = assistant_id or os.getenv("OPENAI_ASSISTANT_ID")
        self._assistant = None

        # Initialize the assistant (only if not using Gemini, since Gemini doesn't have assistants in the same way)
        if self.use_gemini:
            # For Gemini, we'll use chat completions instead of assistants
            self._assistant = None
        elif self.assistant_id:
            # Use existing assistant
            try:
                self._assistant = self.client.beta.assistants.retrieve(self.assistant_id)
            except Exception as e:
                print(f"Error retrieving existing assistant: {str(e)}")
                print("Creating a new assistant instead...")
                self._assistant = self._create_assistant()
        else:
            # Create a new assistant
            self._assistant = self._create_assistant()

    def _create_assistant(self):
        """
        Create an OpenAI Assistant with instructions specific to the book content
        """
        try:
            assistant = self.client.beta.assistants.create(
                name="Physical AI & Humanoid Robotics Expert",
                description="An expert assistant that answers questions about Physical AI and Humanoid Robotics based on the book content",
                model="gpt-3.5-turbo",
                instructions="You are an expert on Physical AI and Humanoid Robotics. Answer questions based on the provided knowledge. Be helpful, accurate, and cite your sources when possible. If the information is not in your knowledge base, say so."
            )
            return assistant
        except Exception as e:
            print(f"Error creating OpenAI assistant: {str(e)}")
            return None

    async def generate_response(self, query: str, context: List[Dict[str, Any]],
                              response_style: str = "standard") -> str:
        """
        Generate a response using OpenAI or Google Gemini based on the query and context
        """
        # Format the context for the prompt
        context_text = ""
        if context:
            context_text = "Relevant information from the book:\n"
            for i, chunk in enumerate(context[:5]):  # Use top 5 chunks
                context_text += f"{i+1}. {chunk['content']}\n\n"

        # Build the prompt based on response style
        if response_style == "simplified":
            prompt = f"""
            Please provide a simplified explanation of the following query based on the provided context:

            Query: {query}

            {context_text}

            Provide a clear, simple explanation that's easy to understand.
            """
        elif response_style == "detailed":
            prompt = f"""
            Please provide a detailed explanation of the following query based on the provided context:

            Query: {query}

            {context_text}

            Provide a comprehensive, in-depth explanation with technical details.
            """
        elif response_style == "examples":
            prompt = f"""
            Please provide practical examples related to the following query based on the provided context:

            Query: {query}

            {context_text}

            Provide concrete examples that illustrate the concepts.
            """
        else:
            prompt = f"""
            Please answer the following query based on the provided context:

            Query: {query}

            {context_text}

            Provide a clear, accurate answer based on the context provided.
            """

        try:
            response = await self._make_chat_completion_request(prompt)
            # Return the text of the response
            return response.choices[0].message.content if response.choices[0].message.content else "I couldn't generate a response based on the provided context."
        except Exception as e:
            # Handle any errors from the API
            return f"Sorry, I encountered an error while generating a response: {str(e)}"

    async def generate_context_aware_response(self, query: str, conversation_history: List[Dict[str, str]],
                                           context: List[Dict[str, Any]], response_style: str = "standard") -> str:
        """
        Generate a response that takes into account the conversation history
        """
        # Format conversation history
        history_text = ""
        if conversation_history:
            history_text = "Previous conversation:\n"
            for exchange in conversation_history[-3:]:  # Use last 3 exchanges for context
                history_text += f"Q: {exchange['query']}\nA: {exchange['response']}\n\n"

        # Format the context for the prompt
        context_text = ""
        if context:
            context_text = "Relevant information from the book:\n"
            for i, chunk in enumerate(context[:5]):  # Use top 5 chunks
                context_text += f"{i+1}. {chunk['content']}\n\n"

        # Build the prompt based on response style
        if response_style == "simplified":
            prompt = f"""
            Please provide a simplified explanation of the following query based on the provided context and conversation history:

            {history_text}

            Query: {query}

            {context_text}

            Provide a clear, simple explanation that's easy to understand, taking into account the previous conversation.
            """
        elif response_style == "detailed":
            prompt = f"""
            Please provide a detailed explanation of the following query based on the provided context and conversation history:

            {history_text}

            Query: {query}

            {context_text}

            Provide a comprehensive, in-depth explanation with technical details, taking into account the previous conversation.
            """
        elif response_style == "examples":
            prompt = f"""
            Please provide practical examples related to the following query based on the provided context and conversation history:

            {history_text}

            Query: {query}

            {context_text}

            Provide concrete examples that illustrate the concepts, taking into account the previous conversation.
            """
        else:
            prompt = f"""
            Please answer the following query based on the provided context and conversation history:

            {history_text}

            Query: {query}

            {context_text}

            Provide a clear, accurate answer based on the context provided, taking into account the previous conversation.
            """

        try:
            response = await self._make_chat_completion_request(prompt, include_history=True)
            # Return the text of the response
            return response.choices[0].message.content if response.choices[0].message.content else "I couldn't generate a response based on the provided context."
        except Exception as e:
            # Handle any errors from the API
            return f"Sorry, I encountered an error while generating a response: {str(e)}"

    def get_assistant(self):
        """
        Return the persistent assistant instance
        """
        return self._assistant

    def is_using_gemini(self):
        """
        Return whether the service is using Gemini API
        """
        return self.use_gemini

    async def _make_chat_completion_request(self, prompt: str, include_history: bool = False):
        """
        Helper method to make chat completion requests, reducing duplicate code
        """
        system_content = "You are a helpful assistant that answers questions based on provided context."
        if include_history:
            system_content += " Take into account the previous conversation history."

        return await asyncio.get_event_loop().run_in_executor(
            None,
            lambda: self.client.chat.completions.create(
                model=self.model_name,
                messages=[
                    {"role": "system", "content": system_content},
                    {"role": "user", "content": prompt}
                ],
                max_tokens=1000,
                temperature=0.7
            )
        )

    async def create_thread_and_run_with_history(self, query: str, conversation_history: List[Dict[str, str]],
                                               context: List[Dict[str, Any]], response_style: str = "standard"):
        """
        Create a thread and run with the assistant for more complex interactions using conversation history
        For Gemini API, fall back to chat completions since it doesn't support the full Assistant API
        """
        if self.use_gemini:
            # For Gemini, use the chat completion approach instead of Assistant API
            return await self.generate_context_aware_response(query, conversation_history, context, response_style)
        else:
            # Use OpenAI Assistant API for OpenAI
            try:
                # Format conversation history
                history_text = ""
                if conversation_history:
                    history_text = "Previous conversation:\n"
                    for exchange in conversation_history[-3:]:  # Use last 3 exchanges for context
                        history_text += f"Q: {exchange['query']}\nA: {exchange['response']}\n\n"

                # Format context based on response style
                if response_style == "simplified":
                    context_text = f"Please provide a simplified explanation of the following query based on the provided context and conversation history:\n\n{history_text}\n\nContext:\n"
                elif response_style == "detailed":
                    context_text = f"Please provide a detailed explanation of the following query based on the provided context and conversation history:\n\n{history_text}\n\nContext:\n"
                elif response_style == "examples":
                    context_text = f"Please provide practical examples related to the following query based on the provided context and conversation history:\n\n{history_text}\n\nContext:\n"
                else:
                    context_text = f"Please answer the following query based on the provided context and conversation history:\n\n{history_text}\n\nContext:\n"

                for i, chunk in enumerate(context[:5]):  # Use top 5 chunks
                    context_text += f"{i+1}. {chunk['content']}\n\n"

                context_text += f"\nQuestion: {query}\n\n"

                # Create a thread with the query, context, and history
                thread = await asyncio.get_event_loop().run_in_executor(
                    None,
                    lambda: self.client.beta.threads.create(
                        messages=[
                            {
                                "role": "user",
                                "content": context_text
                            }
                        ]
                    )
                )

                # Run the thread with the persistent assistant
                run = await asyncio.get_event_loop().run_in_executor(
                    None,
                    lambda: self.client.beta.threads.runs.create(
                        thread_id=thread.id,
                        assistant_id=self._assistant.id
                    )
                )

                # Wait for the run to complete
                import time
                while True:
                    run_status = await asyncio.get_event_loop().run_in_executor(
                        None,
                        lambda: self.client.beta.threads.runs.retrieve(
                            thread_id=thread.id,
                            run_id=run.id
                        )
                    )

                    if run_status.status in ['completed', 'failed', 'cancelled']:
                        break
                    time.sleep(1)

                # Retrieve the messages
                messages = await asyncio.get_event_loop().run_in_executor(
                    None,
                    lambda: self.client.beta.threads.messages.list(
                        thread_id=thread.id,
                        order="asc"
                    )
                )

                # Extract the assistant's response
                for message in messages.data:
                    if message.role == "assistant":
                        return " ".join([item.text.value for item in message.content if item.type == "text"])

                return "I couldn't generate a response based on the provided context."
            except Exception as e:
                return f"Sorry, I encountered an error while generating a response: {str(e)}"

    async def create_thread_and_run(self, query: str, context: List[Dict[str, Any]], response_style: str = "standard"):
        """
        Create a thread and run with the assistant for more complex interactions using the persistent assistant
        For Gemini API, fall back to chat completions since it doesn't support the full Assistant API
        """
        if self.use_gemini:
            # For Gemini, use the chat completion approach instead of Assistant API
            # Create a simplified conversation history with just the current query
            conversation_history = []
            return await self.generate_context_aware_response(query, conversation_history, context, response_style)
        else:
            # Use OpenAI Assistant API for OpenAI
            try:
                # Format context based on response style
                if response_style == "simplified":
                    context_text = f"Please provide a simplified explanation of the following query based on the provided context:\n\nContext:\n"
                elif response_style == "detailed":
                    context_text = f"Please provide a detailed explanation of the following query based on the provided context:\n\nContext:\n"
                elif response_style == "examples":
                    context_text = f"Please provide practical examples related to the following query based on the provided context:\n\nContext:\n"
                else:
                    context_text = f"Please answer the following query based on the provided context:\n\nContext:\n"

                for i, chunk in enumerate(context[:5]):  # Use top 5 chunks
                    context_text += f"{i+1}. {chunk['content']}\n\n"

                context_text += f"\nQuestion: {query}\n\n"

                # Create a thread with the query and context
                thread = await asyncio.get_event_loop().run_in_executor(
                    None,
                    lambda: self.client.beta.threads.create(
                        messages=[
                            {
                                "role": "user",
                                "content": context_text
                            }
                        ]
                    )
                )

                # Run the thread with the persistent assistant
                run = await asyncio.get_event_loop().run_in_executor(
                    None,
                    lambda: self.client.beta.threads.runs.create(
                        thread_id=thread.id,
                        assistant_id=self._assistant.id
                    )
                )

                # Wait for the run to complete
                import time
                while True:
                    run_status = await asyncio.get_event_loop().run_in_executor(
                        None,
                        lambda: self.client.beta.threads.runs.retrieve(
                            thread_id=thread.id,
                            run_id=run.id
                        )
                    )

                    if run_status.status in ['completed', 'failed', 'cancelled']:
                        break
                    time.sleep(1)

                # Retrieve the messages
                messages = await asyncio.get_event_loop().run_in_executor(
                    None,
                    lambda: self.client.beta.threads.messages.list(
                        thread_id=thread.id,
                        order="asc"
                    )
                )

                # Extract the assistant's response
                for message in messages.data:
                    if message.role == "assistant":
                        return " ".join([item.text.value for item in message.content if item.type == "text"])

                return "I couldn't generate a response based on the provided context."
            except Exception as e:
                return f"Sorry, I encountered an error while generating a response: {str(e)}"
