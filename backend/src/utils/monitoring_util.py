"""
Monitoring and observability utilities for the RAG Chatbot application
"""
import time
import logging
from typing import Dict, Any, Optional
from dataclasses import dataclass
from datetime import datetime
import json
import psutil  # For system metrics
import os


@dataclass
class RequestMetrics:
    """
    Data class to hold request metrics
    """
    request_id: str
    endpoint: str
    method: str
    response_time: float
    status_code: int
    timestamp: datetime
    user_id: Optional[str] = None
    session_token: Optional[str] = None


@dataclass
class SystemMetrics:
    """
    Data class to hold system metrics
    """
    timestamp: datetime
    cpu_percent: float
    memory_percent: float
    disk_usage_percent: float
    network_io: Dict[str, int]


class MonitoringService:
    """
    Service class for application monitoring and observability
    """
    def __init__(self):
        self.request_logger = logging.getLogger("rag_chatbot.requests")
        self.system_logger = logging.getLogger("rag_chatbot.system")
        self.metrics_logger = logging.getLogger("rag_chatbot.metrics")

        # Set up formatters
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )

        # Console handler for development
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)

        # Add handlers if not already added
        if not self.request_logger.handlers:
            self.request_logger.addHandler(console_handler)
            self.request_logger.setLevel(logging.INFO)

        if not self.system_logger.handlers:
            self.system_logger.addHandler(console_handler)
            self.system_logger.setLevel(logging.INFO)

        if not self.metrics_logger.handlers:
            self.metrics_logger.addHandler(console_handler)
            self.metrics_logger.setLevel(logging.INFO)

    def log_request(self, request_id: str, endpoint: str, method: str,
                   response_time: float, status_code: int,
                   user_id: Optional[str] = None, session_token: Optional[str] = None):
        """
        Log request metrics
        """
        metrics = RequestMetrics(
            request_id=request_id,
            endpoint=endpoint,
            method=method,
            response_time=response_time,
            status_code=status_code,
            timestamp=datetime.utcnow(),
            user_id=user_id,
            session_token=session_token
        )

        # Log to metrics logger
        self.metrics_logger.info(json.dumps({
            "type": "request",
            "request_id": metrics.request_id,
            "endpoint": metrics.endpoint,
            "method": metrics.method,
            "response_time_ms": round(metrics.response_time * 1000, 2),
            "status_code": metrics.status_code,
            "timestamp": metrics.timestamp.isoformat(),
            "user_id": metrics.user_id,
            "session_token": metrics.session_token
        }))

    def log_system_metrics(self):
        """
        Log system metrics (CPU, memory, disk usage)
        """
        cpu_percent = psutil.cpu_percent(interval=1)
        memory_percent = psutil.virtual_memory().percent
        disk_usage_percent = psutil.disk_usage('/').percent

        # Get network I/O if possible
        try:
            net_io = psutil.net_io_counters()
            network_io = {
                "bytes_sent": net_io.bytes_sent,
                "bytes_recv": net_io.bytes_recv,
                "packets_sent": net_io.packets_sent,
                "packets_recv": net_io.packets_recv
            }
        except:
            network_io = {"bytes_sent": 0, "bytes_recv": 0, "packets_sent": 0, "packets_recv": 0}

        metrics = SystemMetrics(
            timestamp=datetime.utcnow(),
            cpu_percent=cpu_percent,
            memory_percent=memory_percent,
            disk_usage_percent=disk_usage_percent,
            network_io=network_io
        )

        # Log to system logger
        self.system_logger.info(json.dumps({
            "type": "system",
            "timestamp": metrics.timestamp.isoformat(),
            "cpu_percent": metrics.cpu_percent,
            "memory_percent": metrics.memory_percent,
            "disk_usage_percent": metrics.disk_usage_percent,
            "network_io": metrics.network_io
        }))

    def log_error(self, error_type: str, error_message: str,
                 request_id: Optional[str] = None,
                 additional_info: Optional[Dict[str, Any]] = None):
        """
        Log application errors
        """
        error_data = {
            "type": "error",
            "error_type": error_type,
            "error_message": error_message,
            "timestamp": datetime.utcnow().isoformat(),
            "request_id": request_id,
            "additional_info": additional_info or {}
        }

        self.request_logger.error(json.dumps(error_data))

    def log_api_call(self, api_type: str, duration: float,
                    query_length: int, response_length: int,
                    success: bool = True):
        """
        Log external API calls (OpenAI/Gemini)
        """
        api_data = {
            "type": "api_call",
            "api_type": api_type,
            "duration_ms": round(duration * 1000, 2),
            "query_length": query_length,
            "response_length": response_length,
            "success": success,
            "timestamp": datetime.utcnow().isoformat()
        }

        self.metrics_logger.info(json.dumps(api_data))

    def log_db_operation(self, operation: str, duration: float,
                        table: str, success: bool = True,
                        rows_affected: Optional[int] = None):
        """
        Log database operations
        """
        db_data = {
            "type": "db_operation",
            "operation": operation,
            "table": table,
            "duration_ms": round(duration * 1000, 2),
            "success": success,
            "rows_affected": rows_affected,
            "timestamp": datetime.utcnow().isoformat()
        }

        self.metrics_logger.info(json.dumps(db_data))

    def get_health_status(self) -> Dict[str, Any]:
        """
        Get overall health status of the application
        """
        try:
            # Check system resources
            cpu_percent = psutil.cpu_percent(interval=0.1)
            memory_percent = psutil.virtual_memory().percent
            disk_usage_percent = psutil.disk_usage('/').percent

            # Simple health check - if resources are not critically high, return healthy
            is_healthy = (
                cpu_percent < 90 and
                memory_percent < 90 and
                disk_usage_percent < 95
            )

            return {
                "status": "healthy" if is_healthy else "unhealthy",
                "timestamp": datetime.utcnow().isoformat(),
                "system_metrics": {
                    "cpu_percent": cpu_percent,
                    "memory_percent": memory_percent,
                    "disk_usage_percent": disk_usage_percent
                }
            }
        except Exception as e:
            return {
                "status": "error",
                "error": str(e),
                "timestamp": datetime.utcnow().isoformat()
            }


# Global monitoring service instance
monitoring_service = MonitoringService()


def get_monitoring_service() -> MonitoringService:
    """
    Get the global monitoring service instance
    """
    return monitoring_service