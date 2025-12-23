"""FastAPI application entry point for Grillex web application.

This module sets up the FastAPI app with all routes and middleware.
"""

import sys
import os

# Add the src directory to path for importing grillex
src_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../src"))
if src_path not in sys.path:
    sys.path.insert(0, src_path)

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from .config import settings
from .routes import tools_router, events_router, chat_router


def create_app() -> FastAPI:
    """Create and configure the FastAPI application.

    Returns:
        Configured FastAPI application.
    """
    app = FastAPI(
        title=settings.app_name,
        description="Web interface for Grillex FEM solver with LLM integration",
        version="0.1.0",
        debug=settings.debug,
    )

    # Configure CORS for local development
    app.add_middleware(
        CORSMiddleware,
        allow_origins=settings.cors_origins,
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # Include routers
    app.include_router(tools_router)
    app.include_router(events_router)
    app.include_router(chat_router)

    @app.get("/")
    async def root():
        """Root endpoint with API information."""
        return {
            "name": settings.app_name,
            "version": "0.1.0",
            "docs": "/docs",
            "endpoints": {
                "tools": "/api/tools/{tool_name}",
                "model_state": "/api/model/state",
                "events": "/api/events",
                "chat": "/api/chat",
                "health": "/api/health",
            },
        }

    return app


# Create the app instance
app = create_app()


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        "webapp.backend.main:app",
        host=settings.host,
        port=settings.port,
        reload=settings.debug,
    )
