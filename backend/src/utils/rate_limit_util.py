"""
Rate limiting utilities for the RAG Chatbot application
"""
import time
from typing import Dict, Optional
from collections import defaultdict
import threading


class RateLimiter:
    """
    Simple in-memory rate limiter
    Note: In production, use Redis or similar for distributed rate limiting
    """
    def __init__(self):
        self.requests = defaultdict(list)  # session_token -> list of timestamps
        self.lock = threading.Lock()  # Thread safety for the in-memory store

    def is_allowed(self, session_token: str, max_requests: int, window_seconds: int) -> bool:
        """
        Check if a request is allowed based on rate limits
        """
        with self.lock:
            current_time = time.time()

            # Clean up old requests outside the window
            self.requests[session_token] = [
                req_time for req_time in self.requests[session_token]
                if current_time - req_time < window_seconds
            ]

            # Check if we're under the limit
            if len(self.requests[session_token]) < max_requests:
                # Add current request
                self.requests[session_token].append(current_time)
                return True

            return False

    def check_session_limits(self, session_token: str) -> Dict[str, bool]:
        """
        Check multiple rate limits for a session
        """
        return {
            "per_minute": self.is_allowed(session_token, max_requests=30, window_seconds=60),
            "per_hour": self.is_allowed(session_token, max_requests=500, window_seconds=3600)
        }


# Global rate limiter instance
rate_limiter = RateLimiter()


def check_rate_limit(session_token: str) -> bool:
    """
    Check if the session has exceeded rate limits
    """
    limits = rate_limiter.check_session_limits(session_token)
    return limits["per_minute"] and limits["per_hour"]