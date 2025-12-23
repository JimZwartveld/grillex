"""API routes for Grillex web application."""

from .tools import router as tools_router
from .events import router as events_router
from .chat import router as chat_router

__all__ = ["tools_router", "events_router", "chat_router"]
