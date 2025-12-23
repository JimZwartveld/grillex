"""Configuration settings for the Grillex web application backend."""

from typing import List, Optional
from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    """Application settings loaded from environment variables.

    Attributes:
        app_name: Name of the application.
        debug: Enable debug mode.
        cors_origins: List of allowed CORS origins.
        anthropic_api_key: API key for Anthropic Claude API.
        host: Host to bind the server to.
        port: Port to bind the server to.
    """

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
    )

    app_name: str = "Grillex Web Application"
    debug: bool = True

    # CORS settings
    cors_origins: List[str] = [
        "http://localhost:3000",
        "http://localhost:5173",
        "http://127.0.0.1:3000",
        "http://127.0.0.1:5173",
    ]

    # Anthropic API
    anthropic_api_key: Optional[str] = None

    # Server settings
    host: str = "0.0.0.0"
    port: int = 8000


# Global settings instance
settings = Settings()


def get_settings() -> Settings:
    """Get the global settings instance.

    Returns:
        The Settings instance.
    """
    return settings
