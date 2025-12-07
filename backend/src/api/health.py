"""
Health check and monitoring API endpoints
"""
from fastapi import APIRouter
from typing import Dict, Any
from ..utils.monitoring_util import monitoring_service
from pydantic import BaseModel
import datetime


class HealthResponse(BaseModel):
    """
    Health check response model
    """
    status: str
    timestamp: str
    system_metrics: Dict[str, float]
    details: Dict[str, Any] = {}


class MetricsResponse(BaseModel):
    """
    Metrics response model
    """
    status: str
    timestamp: str
    metrics: Dict[str, Any]


router = APIRouter()


@router.get("/health")
async def health_check():
    """
    Health check endpoint
    """
    health_status = monitoring_service.get_health_status()
    return health_status


@router.get("/health/ready")
async def readiness_check():
    """
    Readiness check endpoint - indicates if the app is ready to serve requests
    """
    health_status = monitoring_service.get_health_status()

    # For readiness, we might have different criteria than health
    is_ready = health_status["status"] == "healthy"

    return {
        "status": "ready" if is_ready else "not_ready",
        "timestamp": health_status["timestamp"]
    }


@router.get("/health/live")
async def liveness_check():
    """
    Liveness check endpoint - indicates if the app is alive and running
    """
    # Basic liveness check - just return that the app is running
    return {
        "status": "alive",
        "timestamp": monitoring_service.get_health_status()["timestamp"]
    }


@router.get("/metrics")
async def get_metrics():
    """
    Get application metrics
    """
    # Get system metrics
    import psutil

    system_metrics = {
        "cpu_percent": psutil.cpu_percent(interval=0.1),
        "memory_percent": psutil.virtual_memory().percent,
        "disk_usage_percent": psutil.disk_usage('/').percent,
        "timestamp": datetime.datetime.utcnow().isoformat()
    }

    # In a real implementation, you'd gather more detailed metrics
    # from various parts of your application
    return {
        "status": "success",
        "timestamp": datetime.datetime.utcnow().isoformat(),
        "metrics": {
            "system": system_metrics,
            "application": {
                "uptime_seconds": 0,  # Would track actual uptime
                "active_connections": 0,  # Would track active connections
                "requests_per_minute": 0,  # Would track request rate
            }
        }
    }


@router.get("/monitoring/status")
async def get_monitoring_status():
    """
    Get detailed monitoring status
    """
    health_status = monitoring_service.get_health_status()

    # Add additional monitoring info
    monitoring_info = {
        "health": health_status,
        "monitoring_enabled": True,
        "log_level": "INFO",  # Would come from config
        "timestamp": datetime.datetime.utcnow().isoformat()
    }

    return monitoring_info